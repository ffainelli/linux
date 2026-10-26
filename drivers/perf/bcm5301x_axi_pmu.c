// SPDX-License-Identifier: GPL-2.0-only
/* Broadcom BCM5301X AXI Trace Module performance counters. */

#include <linux/bitops.h>
#include <linux/cpu.h>
#include <linux/cpuhotplug.h>
#include <linux/hrtimer.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/perf_event.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/slab.h>

#define ATM_CONFIG		0x00
#define ATM_STATUS		0x04
#define ATM_CMD			0x0c
#define ATM_CTRL_LOCAL		BIT(0)
#define ATM_CMDS_EN		BIT(4)
#define ATM_BEATS_EN		BIT(6)
#define ATM_BUSY_EN		BIT(8)
#define ATM_OUTS_EN		BIT(14)
#define ATM_COUNT_ENABLE		(ATM_CMDS_EN | ATM_BEATS_EN | \
				 ATM_BUSY_EN | ATM_OUTS_EN)
#define ATM_STATE_ENABLE		(BIT(9) | BIT(10) | BIT(11) | BIT(13))
#define ATM_STATE_MASK		GENMASK(13, 9)
#define ATM_CLEAR		0
#define ATM_STOP			1
#define ATM_START		2

/* At most one increment per AXI cycle for the exposed counters. */
#define ATM_POLL_INTERVAL	ms_to_ktime(100)
#define ATM_NR_COUNTERS		16

static enum cpuhp_state bcm5301x_axi_hp_state;

struct bcm5301x_axi_pmu {
	struct pmu pmu;
	void __iomem *base;
	raw_spinlock_t lock;
	struct hrtimer timer;
	struct hlist_node hp_node;
	struct perf_event *events[ATM_NR_COUNTERS];
	unsigned int nr_active;
	int cpu;
};

/* Event IDs select fixed counters, not programmable event selectors. */
static const u16 bcm5301x_axi_offsets[ATM_NR_COUNTERS] = {
	0x10, 0x14, 0x18, 0x1c, 0x20, 0x24, 0x28, 0x2c,
	0x30, 0x34, 0x38, 0x3c, 0x58, 0x5c, 0x60, 0x64,
};

#define ATM_EVENT(_name, _id) \
	static struct perf_pmu_events_attr event_attr_##_name = { \
		.attr = __ATTR(_name, 0444, perf_event_sysfs_show, NULL), \
		.id = _id, \
		.event_str = "event=" __stringify(_id), \
	}

ATM_EVENT(write_commands, 0);
ATM_EVENT(read_commands, 1);
ATM_EVENT(write_address_cycles, 2);
ATM_EVENT(read_address_cycles, 3);
ATM_EVENT(write_data_cycles, 4);
ATM_EVENT(read_data_cycles, 5);
ATM_EVENT(write_response_cycles, 6);
ATM_EVENT(write_address_stall_cycles, 7);
ATM_EVENT(read_address_stall_cycles, 8);
ATM_EVENT(write_data_stall_cycles, 9);
ATM_EVENT(read_data_stall_cycles, 10);
ATM_EVENT(write_response_stall_cycles, 11);
ATM_EVENT(write_beats, 12);
ATM_EVENT(read_beats, 13);
ATM_EVENT(write_outstanding_cycles, 14);
ATM_EVENT(read_outstanding_cycles, 15);

static struct attribute *bcm5301x_axi_event_attrs[] = {
	&event_attr_write_commands.attr.attr,
	&event_attr_read_commands.attr.attr,
	&event_attr_write_address_cycles.attr.attr,
	&event_attr_read_address_cycles.attr.attr,
	&event_attr_write_data_cycles.attr.attr,
	&event_attr_read_data_cycles.attr.attr,
	&event_attr_write_response_cycles.attr.attr,
	&event_attr_write_address_stall_cycles.attr.attr,
	&event_attr_read_address_stall_cycles.attr.attr,
	&event_attr_write_data_stall_cycles.attr.attr,
	&event_attr_read_data_stall_cycles.attr.attr,
	&event_attr_write_response_stall_cycles.attr.attr,
	&event_attr_write_beats.attr.attr,
	&event_attr_read_beats.attr.attr,
	&event_attr_write_outstanding_cycles.attr.attr,
	&event_attr_read_outstanding_cycles.attr.attr,
	NULL,
};

static const struct attribute_group bcm5301x_axi_events_group = {
	.name = "events",
	.attrs = bcm5301x_axi_event_attrs,
};

PMU_FORMAT_ATTR(event, "config:0-3");

static struct attribute *bcm5301x_axi_format_attrs[] = {
	&format_attr_event.attr,
	NULL,
};

static const struct attribute_group bcm5301x_axi_format_group = {
	.name = "format",
	.attrs = bcm5301x_axi_format_attrs,
};

static struct bcm5301x_axi_pmu *to_axi_pmu(struct pmu *pmu)
{
	return container_of(pmu, struct bcm5301x_axi_pmu, pmu);
}

static ssize_t cpumask_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct bcm5301x_axi_pmu *axi = to_axi_pmu(dev_get_drvdata(dev));

	return sysfs_emit(buf, "%*pbl\n", cpumask_pr_args(cpumask_of(READ_ONCE(axi->cpu))));
}
static DEVICE_ATTR_RO(cpumask);

static struct attribute *bcm5301x_axi_attrs[] = {
	&dev_attr_cpumask.attr,
	NULL,
};

static const struct attribute_group bcm5301x_axi_group = {
	.attrs = bcm5301x_axi_attrs,
};

static const struct attribute_group *bcm5301x_axi_attr_groups[] = {
	&bcm5301x_axi_events_group,
	&bcm5301x_axi_format_group,
	&bcm5301x_axi_group,
	NULL,
};

static u32 bcm5301x_axi_read_counter(struct bcm5301x_axi_pmu *axi, int idx)
{
	return readl(axi->base + bcm5301x_axi_offsets[idx]);
}

/* All counter state, including prev_count, is serialized by axi->lock. */
static void bcm5301x_axi_update(struct bcm5301x_axi_pmu *axi,
			       struct perf_event *event)
{
	u32 now = bcm5301x_axi_read_counter(axi, event->hw.idx);
	u32 prev = local64_read(&event->hw.prev_count);

	local64_set(&event->hw.prev_count, now);
	local64_add((u32)(now - prev), &event->count);
}

static enum hrtimer_restart bcm5301x_axi_timer(struct hrtimer *timer)
{
	struct bcm5301x_axi_pmu *axi = container_of(timer, typeof(*axi), timer);
	enum hrtimer_restart ret = HRTIMER_NORESTART;
	unsigned long flags;
	int i;

	raw_spin_lock_irqsave(&axi->lock, flags);
	for (i = 0; i < ATM_NR_COUNTERS; i++) {
		struct perf_event *event = axi->events[i];

		if (event && !(event->hw.state & PERF_HES_STOPPED))
			bcm5301x_axi_update(axi, event);
	}
	if (axi->nr_active) {
		hrtimer_forward_now(timer, ATM_POLL_INTERVAL);
		ret = HRTIMER_RESTART;
	}
	raw_spin_unlock_irqrestore(&axi->lock, flags);

	return ret;
}

static int bcm5301x_axi_event_init(struct perf_event *event)
{
	struct bcm5301x_axi_pmu *axi = to_axi_pmu(event->pmu);
	struct perf_event *sibling, *leader = event->group_leader;
	unsigned long used = 0;

	if (event->attr.type != event->pmu->type)
		return -ENOENT;
	if (is_sampling_event(event) || event->attach_state & PERF_ATTACH_TASK ||
	    event->cpu < 0)
		return -EOPNOTSUPP;
	if (event->attr.config >= ATM_NR_COUNTERS || event->attr.config1 ||
	    event->attr.config2)
		return -EINVAL;

	/* A group cannot allocate the same fixed counter twice. */
	if (!is_software_event(leader)) {
		if (leader->pmu != event->pmu)
			return -EINVAL;
		__set_bit(leader->attr.config, &used);
	}
	for_each_sibling_event(sibling, leader) {
		if (is_software_event(sibling))
			continue;
		if (sibling->pmu != event->pmu ||
		    test_and_set_bit(sibling->attr.config, &used))
			return -EINVAL;
	}
	if (event != leader && test_bit(event->attr.config, &used))
		return -EINVAL;

	event->cpu = READ_ONCE(axi->cpu);
	event->hw.idx = -1;
	return 0;
}

static void bcm5301x_axi_start(struct perf_event *event, int flags)
{
	struct bcm5301x_axi_pmu *axi = to_axi_pmu(event->pmu);
	unsigned long irqflags;

	raw_spin_lock_irqsave(&axi->lock, irqflags);
	if (event->hw.state & PERF_HES_STOPPED) {
		local64_set(&event->hw.prev_count,
			    bcm5301x_axi_read_counter(axi, event->hw.idx));
		event->hw.state = 0;
		/* Unpinned: the timer can migrate when the managing CPU goes offline. */
		if (!axi->nr_active++)
			hrtimer_start(&axi->timer, ATM_POLL_INTERVAL, HRTIMER_MODE_REL);
	}
	raw_spin_unlock_irqrestore(&axi->lock, irqflags);
}

static void bcm5301x_axi_stop(struct perf_event *event, int flags)
{
	struct bcm5301x_axi_pmu *axi = to_axi_pmu(event->pmu);
	unsigned long irqflags;

	raw_spin_lock_irqsave(&axi->lock, irqflags);
	if (!(event->hw.state & PERF_HES_STOPPED)) {
		bcm5301x_axi_update(axi, event);
		event->hw.state = PERF_HES_STOPPED | PERF_HES_UPTODATE;
		axi->nr_active--;
	}
	/* The timer stops itself when no events are running. */
	raw_spin_unlock_irqrestore(&axi->lock, irqflags);
}

static void bcm5301x_axi_read(struct perf_event *event)
{
	struct bcm5301x_axi_pmu *axi = to_axi_pmu(event->pmu);
	unsigned long flags;

	raw_spin_lock_irqsave(&axi->lock, flags);
	if (!(event->hw.state & PERF_HES_STOPPED))
		bcm5301x_axi_update(axi, event);
	raw_spin_unlock_irqrestore(&axi->lock, flags);
}

static int bcm5301x_axi_add(struct perf_event *event, int flags)
{
	struct bcm5301x_axi_pmu *axi = to_axi_pmu(event->pmu);
	unsigned int idx = event->attr.config;
	unsigned long irqflags;

	raw_spin_lock_irqsave(&axi->lock, irqflags);
	if (axi->events[idx]) {
		raw_spin_unlock_irqrestore(&axi->lock, irqflags);
		return -EAGAIN;
	}
	axi->events[idx] = event;
	event->hw.idx = idx;
	event->hw.state = PERF_HES_STOPPED | PERF_HES_UPTODATE;
	raw_spin_unlock_irqrestore(&axi->lock, irqflags);

	if (flags & PERF_EF_START)
		bcm5301x_axi_start(event, PERF_EF_RELOAD);
	perf_event_update_userpage(event);
	return 0;
}

static void bcm5301x_axi_del(struct perf_event *event, int flags)
{
	struct bcm5301x_axi_pmu *axi = to_axi_pmu(event->pmu);
	unsigned long irqflags;

	bcm5301x_axi_stop(event, PERF_EF_UPDATE);
	raw_spin_lock_irqsave(&axi->lock, irqflags);
	axi->events[event->hw.idx] = NULL;
	event->hw.idx = -1;
	raw_spin_unlock_irqrestore(&axi->lock, irqflags);
	perf_event_update_userpage(event);
}

static int bcm5301x_axi_offline(unsigned int cpu, struct hlist_node *node)
{
	struct bcm5301x_axi_pmu *axi = hlist_entry(node, typeof(*axi), hp_node);
	unsigned int target;

	if (cpu != READ_ONCE(axi->cpu))
		return 0;
	target = cpumask_any_but(cpu_online_mask, cpu);
	if (target >= nr_cpu_ids)
		return 0;
	perf_pmu_migrate_context(&axi->pmu, cpu, target);
	WRITE_ONCE(axi->cpu, target);
	return 0;
}

static void bcm5301x_axi_hw_stop(struct bcm5301x_axi_pmu *axi)
{
	writel(ATM_STOP, axi->base + ATM_CMD);
	/* Flush the posted command before the resource is unmapped. */
	readl(axi->base + ATM_STATUS);
}

static int bcm5301x_axi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bcm5301x_axi_pmu *axi;
	struct resource *res;
	char *name;
	u32 status;
	int ret;

	axi = devm_kzalloc(dev, sizeof(*axi), GFP_KERNEL);
	if (!axi)
		return -ENOMEM;
	axi->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(axi->base))
		return PTR_ERR(axi->base);
	if (resource_size(res) < SZ_4K)
		return -EINVAL;
	name = devm_kasprintf(dev, GFP_KERNEL, "bcm5301x_axi_%llx",
			      (unsigned long long)res->start);
	if (!name)
		return -ENOMEM;

	raw_spin_lock_init(&axi->lock);
	hrtimer_setup(&axi->timer, bcm5301x_axi_timer, CLOCK_MONOTONIC,
		      HRTIMER_MODE_REL);
	axi->pmu = (struct pmu) {
		.module = THIS_MODULE,
		.parent = dev,
		.task_ctx_nr = perf_invalid_context,
		.capabilities = PERF_PMU_CAP_NO_EXCLUDE | PERF_PMU_CAP_NO_INTERRUPT,
		.attr_groups = bcm5301x_axi_attr_groups,
		.event_init = bcm5301x_axi_event_init,
		.add = bcm5301x_axi_add,
		.del = bcm5301x_axi_del,
		.start = bcm5301x_axi_start,
		.stop = bcm5301x_axi_stop,
		.read = bcm5301x_axi_read,
	};

	/* Take exclusive local ownership; disable tracing, filtering and saturation. */
	writel(ATM_CTRL_LOCAL, axi->base + ATM_CONFIG);
	writel(ATM_STOP, axi->base + ATM_CMD);
	ret = readl_poll_timeout(axi->base + ATM_STATUS, status,
				!(status & ATM_STATE_MASK), 1, 1000);
	if (ret)
		return dev_err_probe(dev, ret, "Counters did not stop\n");
	writel(ATM_CLEAR, axi->base + ATM_CMD);
	writel(ATM_CTRL_LOCAL | ATM_COUNT_ENABLE, axi->base + ATM_CONFIG);
	writel(ATM_START, axi->base + ATM_CMD);
	ret = readl_poll_timeout(axi->base + ATM_STATUS, status,
				(status & ATM_STATE_MASK) == ATM_STATE_ENABLE, 1, 1000);
	if (ret) {
		bcm5301x_axi_hw_stop(axi);
		return dev_err_probe(dev, ret, "Counters did not start\n");
	}
	/*
	 * Keep fixed counters free-running. Per-event baselines isolate sessions
	 * without clearing or stopping counters used by another perf event.
	 */
	cpus_read_lock();
	axi->cpu = cpumask_first(cpu_online_mask);
	ret = perf_pmu_register(&axi->pmu, name, -1);
	if (!ret) {
		ret = cpuhp_state_add_instance_nocalls_cpuslocked(bcm5301x_axi_hp_state,
							       &axi->hp_node);
		if (ret)
			perf_pmu_unregister(&axi->pmu);
	}
	cpus_read_unlock();
	if (ret) {
		hrtimer_cancel(&axi->timer);
		bcm5301x_axi_hw_stop(axi);
		return ret;
	}
	platform_set_drvdata(pdev, axi);
	return 0;
}

static void bcm5301x_axi_remove(struct platform_device *pdev)
{
	struct bcm5301x_axi_pmu *axi = platform_get_drvdata(pdev);

	cpuhp_state_remove_instance_nocalls(bcm5301x_axi_hp_state, &axi->hp_node);
	perf_pmu_unregister(&axi->pmu);
	hrtimer_cancel(&axi->timer);
	bcm5301x_axi_hw_stop(axi);
}

static const struct of_device_id bcm5301x_axi_of_match[] = {
	{ .compatible = "brcm,bcm5301x-axi-pmu" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm5301x_axi_of_match);

static struct platform_driver bcm5301x_axi_driver = {
	.probe = bcm5301x_axi_probe,
	.remove = bcm5301x_axi_remove,
	.driver = {
		.name = "bcm5301x-axi-pmu",
		.of_match_table = bcm5301x_axi_of_match,
		.suppress_bind_attrs = true,
	},
};

static int __init bcm5301x_axi_init(void)
{
	int ret;

	ret = cpuhp_setup_state_multi(CPUHP_AP_ONLINE_DYN, "perf/bcm5301x_axi:online",
				     NULL, bcm5301x_axi_offline);
	if (ret < 0)
		return ret;
	bcm5301x_axi_hp_state = ret;
	ret = platform_driver_register(&bcm5301x_axi_driver);
	if (ret)
		cpuhp_remove_multi_state(bcm5301x_axi_hp_state);
	return ret;
}
module_init(bcm5301x_axi_init);

static void __exit bcm5301x_axi_exit(void)
{
	platform_driver_unregister(&bcm5301x_axi_driver);
	cpuhp_remove_multi_state(bcm5301x_axi_hp_state);
}
module_exit(bcm5301x_axi_exit);

MODULE_DESCRIPTION("Broadcom BCM5301X AXI Trace Module PMU");
MODULE_LICENSE("GPL");
