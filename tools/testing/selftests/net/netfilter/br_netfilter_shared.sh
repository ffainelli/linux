#!/bin/bash
# SPDX-License-Identifier: GPL-2.0
#
# Test br_netfilter handling of broadcast/multicast packets when
# cloned or multi-referenced skbs reach br_nf_local_in() with
# non-exclusive conntrack entries.

source lib.sh

checktool "nft --version" "run test without nft tool"

read t < /proc/sys/kernel/tainted
if [ "$t" -ne 0 ]; then
	echo "SKIP: kernel is tainted"
	exit $ksft_skip
fi

cleanup() {
	cleanup_all_ns
}

trap cleanup EXIT

setup_ns ns0 ns1 ns2 ns3

ret=0

ip netns exec "$ns0" sysctl -q net.ipv4.conf.all.rp_filter=0
ip netns exec "$ns0" sysctl -q net.ipv4.conf.default.rp_filter=0

if ! ip link add veth1 netns "$ns0" type veth peer name eth0 netns "$ns1"; then
	echo "SKIP: Can't create veth device"
	exit $ksft_skip
fi

ip link add veth2 netns "$ns0" type veth peer name eth0 netns "$ns2"
ip link add veth3 netns "$ns0" type veth peer name eth0 netns "$ns3"

for i in $(seq 1 3); do
	ip -net "$ns0" link set "veth$i" up
done

if ! ip -net "$ns0" link add br0 type bridge stp_state 0 forward_delay 0 \
	nf_call_iptables 1 nf_call_ip6tables 1 nf_call_arptables 1; then
	echo "SKIP: Can't create bridge br0"
	exit $ksft_skip
fi

for i in $(seq 1 3); do
	ip -net "$ns0" link set "veth$i" master br0
done

ip -net "$ns0" link set br0 up
ip -net "$ns0" addr add 10.0.0.1/24 dev br0

modprobe -q br_netfilter
if ! ip netns exec "$ns0" sysctl -q net.bridge.bridge-nf-call-iptables=1; then
	echo "SKIP: bridge netfilter not available"
	exit $ksft_skip
fi

ip netns exec "$ns0" sysctl -q net.ipv4.icmp_echo_ignore_broadcasts=0

# enable conntrack in ns0 for bridge local-in
ip netns exec "$ns0" nft -f - <<EOF
table ip filter {
	chain input {
		type filter hook input priority 1; policy accept
		iifname br0 counter
		ct state new accept
	}
}
EOF
if [ "$?" -ne 0 ]; then
	echo "SKIP: could not add nftables ruleset"
	exit $ksft_skip
fi

for i in $(seq 1 3); do
	eval ip -net \$ns"$i" link set eth0 up
	eval ip -net \$ns"$i" addr add "10.0.0.1$i/24" dev eth0
done

# Add queuing delay on egress bridge ports so cloned skbs remain in flight/queued
# while the local frame is delivered to br_nf_local_in()
ip -net "$ns0" qdisc add dev veth2 root netem delay 50ms 2>/dev/null || true
ip -net "$ns0" qdisc add dev veth3 root netem delay 50ms 2>/dev/null || true

# Send broadcast ping probes from ns1
for i in $(seq 1 100); do
	ip netns exec "$ns1" ping -q -f -b -c 1 10.0.0.255 > /dev/null 2>&1 || true
done

read t < /proc/sys/kernel/tainted
if [ "$t" -eq 0 ]; then
	echo "PASS: kernel not tainted"
else
	echo "ERROR: kernel is tainted"
	dmesg
	ret=1
fi

exit $ret
