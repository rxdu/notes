---
title: Share Internet through the USB Port on Pocketbeagle
description: pocketbeagle ethernet-over-usb
---

On the host computer, there are two interfaces invoved: 

* the USB connection to pocketbeagle, e.g. "enx60640568c683"
* the other network adapter that has Internet access, e.g. "eno1"

then on the host computer, do the following

```bash
$ sudo -i
$ echo "1" > /proc/sys/net/ipv4/ip_forward
$ iptables --table nat --append POSTROUTING --out-interface eno1 -j MASQUERADE
$ iptables --append FORWARD --in-interface enx60640568c683 --out-interface eno1 -j ACCEPT
$ iptables --append FORWARD --in-interface eno1 --out-interface enx60640568c683 -m state --state RELATED,ESTABLISHED -j ACCEPT
$ iptables --append FORWARD --in-interface enx60640568c683 -j ACCEPT
```

On the pocketbeagle, do the following

```bash
$ sudo route add default gw 192.168.7.1
$ sudo bash -c 'echo "nameserver 8.8.8.8" >> /etc/resolv.conf'
```

## Reference

* [1] https://wiki.analog.com/resources/tools-software/linux-drivers/iio-inertial-measurement-units/adis16475#trigger_management