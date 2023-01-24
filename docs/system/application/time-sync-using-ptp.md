---
title: Time Synchronization using PTP/IEEE1588
description: PTP/IEEE1588 protocol for time synchronization
---

Recently I've been using an Ouster OS1-64 lidar on a mobile robot for 3D mapping and navigation. To get the right timestamp for the pointcloud to be used with other parts of ROS, a PTP grandmaster needs to be configured.

<!-- This note records useful information about how you can  -->

## Install Packages

```bash
## install relevant packages
$ sudo apt install ethtool linuxptp chrony
```

The following list is taken from the Ouster documentation to give you a brief idea of the functions of the packages:

* **ethtool** - A tool to query the hardware and driver capabilities of a given Ethernet interface.
* **linuxptp** - Linux PTP package with the following components:
    - **ptp4l** daemon to manage hardware and participate as a PTP node
    - **phc2sys** to synchronize the Ethernet controller’s hardware clock to the Linux system clock or shared memory region
    - **pmc** to query the PTP nodes on the network.
* **chrony** - A NTP and PTP time synchronization daemon. It can be configured to listen to both NTP time sources via the Internet and a PTP master clock such as one provided by a GPS with PTP support. This will validate the time configuration makes sense given multiple time sources.

## Useful commands

```bash
## check synchronization between PHC and system time
$ sudo phc_ctl eth0 cmp
```

## Useful Documentation

Materials covering how PTP works:

* [AlliedTelesis Doc: Precision Time Protocol & Transparent Clock](https://www.alliedtelesis.com/sites/default/files/ptp_feature_overview_guide_rev_a.pdf)
* [NetTimeLogic Doc: PTP Basics](https://www.nettimelogic.com/resources/PTP%20Basics.pdf)

How to configure linuxptp:

* [Redhat Doc: Configuring PTP Using ptp4l](https://access.redhat.com/documentation/en-us/red_hat_enterprise_linux/6/html/deployment_guide/ch-configuring_ptp_using_ptp4l)
* [ptp4l (manual)](https://manpages.ubuntu.com/manpages/focal/en/man8/ptp4l.8.html)
* [phc2sys (manual)](https://manpages.ubuntu.com/manpages/focal/en/man8/phc2sys.8.html)

## Relevant Projects

* [TSN Documentation Project for Linux](https://tsn.readthedocs.io/index.html)

## Reference

* [Ouster Sensor Docs (wiki)](https://static.ouster.dev/sensor-docs/index.html)
* [Ouster Lidar Software User Manual](https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2p0.pdf)