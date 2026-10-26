.. SPDX-License-Identifier: GPL-2.0-only

Broadcom BCM5301X AXI Trace Module PMU
====================================

The BCM5301X AXI Trace Modules observe the two PL310 master interfaces and
the Accelerator Coherency Port (ACP). Each instance registers a PMU named
``bcm5301x_axi_<physical-base>``, for example ``bcm5301x_axi_19003000`` for
PL310 M0, ``bcm5301x_axi_19004000`` for PL310 M1, and
``bcm5301x_axi_19007000`` for ACP.

Enable CONFIG_PERF_EVENTS and CONFIG_BCM5301X_AXI_PMU and provide an enabled
device-tree node for each instance. Load ``bcm5301x_axi_pmu`` if built as a
module. The driver owns the module exclusively; firmware or debug tools
must not change its configuration while the driver is bound.

Events and scope
----------------

The ``events`` and ``format`` directories under
``/sys/bus/event_source/devices/bcm5301x_axi_*`` describe the ABI. Event IDs
0 through 15 select fixed counters:

==== ====================================
ID   Event
==== ====================================
0    write_commands
1    read_commands
2    write_address_cycles
3    read_address_cycles
4    write_data_cycles
5    read_data_cycles
6    write_response_cycles
7    write_address_stall_cycles
8    read_address_stall_cycles
9    write_data_stall_cycles
10   read_data_stall_cycles
11   write_response_stall_cycles
12   write_beats
13   read_beats
14   write_outstanding_cycles
15   read_outstanding_cycles
==== ====================================

Active channel cycles count VALID asserted; stall cycles count VALID
asserted while READY is deasserted. Outstanding cycles count cycles with
more than zero outstanding commands. These are cycle counts, not the sum
of outstanding requests. Counts cover all traffic at the observed port;
PL310 hits do not necessarily generate traffic at its master interfaces.
ACP can remain idle unless an accelerator uses that interface.

All measurements are unfiltered and system-wide. The ``cpumask`` file
identifies the CPU managing each PMU, not a CPU traffic filter. Per-task
counting, privilege exclusions, sampling, configurable AXI filters,
latency measurements, and ATB transaction trace capture are unsupported.
Beat counts are reported as beats, without assuming a byte conversion.

Each fixed counter can be allocated once at a time. Duplicate requests
in a single group are rejected; separate events can be multiplexed by
perf. Different counters on one PMU can run together. Groups spanning
different hardware PMUs are rejected.

The hardware counters run continuously while the driver is bound, with
per-event baselines excluding periods when an event is stopped. Starting
one event does not reset another event's hardware counter. There is no
overflow interrupt. A timer extends the wrapping 32-bit counters to 64 bits
every 100 ms while events run. At 500 MHz a counter incrementing once per
cycle wraps in about 8.6 seconds. Accuracy requires servicing the timer
before a full wrap; long interrupt stalls and suspend intervals can lose
counts. Stop measurements before suspending the system.

Testing
-------

Check discovery and count commands and beats over a memory workload::

  modprobe bcm5301x_axi_pmu
  ls /sys/bus/event_source/devices/bcm5301x_axi_*
  perf list | grep bcm5301x_axi
  perf stat -a -e '{bcm5301x_axi_19003000/read_commands/,bcm5301x_axi_19003000/read_beats/}' -- sleep 10

Repeat during a workload whose working set exceeds the PL310 cache, such
as a memory stress test. Compare with idle, test M1 separately, and expect
ACP activity only with an ACP user. Network DMA traffic alone need not
cross the monitored PL310 ports. Use ``perf stat -a -I 1000`` for interval
counts and run for at least a minute under load to exercise counter wrap.

Check that adding or removing a second, different counter leaves the first
counter accumulating. Two requests for the same event in one group must
fail. ``perf record`` and per-task ``perf stat`` must fail. If CPU hotplug
is available, offline the CPU listed in ``cpumask`` while counting and
verify that management moves to another online CPU and counting continues.
