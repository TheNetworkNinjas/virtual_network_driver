#!/bin/bash

sudo ip netns add v0
sudo ip netns add v1
sudo ip netns add v2

sudo iw phy phy$1 set netns name v0
sudo iw phy phy$2 set netns name v1
sudo iw phy phy$3 set netns name v2

sudo ip netns exec v0 ip link set vif0 up
sudo ip netns exec v1 ip link set vif1 up
sudo ip netns exec v2 ip link set vif2 up

sudo ip netns exec v0 hostapd -B hostapd.conf

sudo ip netns exec v0 ip addr add 10.10.10.1/24 dev vif0
sudo ip netns exec v1 ip addr add 10.10.10.2/24 dev vif1
sudo ip netns exec v2 ip addr add 10.10.10.3/24 dev vif2
