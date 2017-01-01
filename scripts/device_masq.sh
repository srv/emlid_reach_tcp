#!/bin/bash
NETWORK_INTERFACE=enp4s0
DEVICE_INTERFACE=enp0s20u1

iptables --table nat --append POSTROUTING --out-interface  $NETWORK_INTERFACE -j MASQUERADE
iptables --append FORWARD --in-interface $DEVICE_INTERFACE -j ACCEPT
