#!/bin/bash
NETWORK_INTERFACE=enx7cdd90736a75
DEVICE_INTERFACE=usb0

iptables -A PREROUTING -t nat -i $NETWORK_INTERFACE -p tcp --dport 80 -j DNAT --to 192.168.2.15:80
iptables -A FORWARD -o $DEVICE_INTERFACE -p tcp -d 192.168.2.15 --dport 80 -j ACCEPT

iptables -A PREROUTING -t nat -i $NETWORK_INTERFACE -p tcp --dport 9000 -j DNAT --to 192.168.2.15:9000
iptables -A FORWARD -o $DEVICE_INTERFACE -p tcp -d 192.168.2.15 --dport 9000 -j ACCEPT
