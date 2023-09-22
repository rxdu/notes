# Time Synchronization using PTP/IEEE1588

For a complicated robotic system, there may be multiple onboard computers and sensors working together to have the whole system functional. To ensure the data from different devices are synchronized in time, you may need to consider time synchronization among the devices. PTP is a synchronization protocol designed for this purpose and it allows sub-microsecond accuracy if properly configured. One example usage of PTP time synchronization is setting up an Ouster OS1-64 Lidar on a mobile robot for 3D mapping and navigation. To get the right timestamp for the pointcloud to be used with other parts of ROS stack, such as mapping and localization, a PTP grandmaster can be configured on the navigation computer to synchronize the time between Lidar and the computer.

## How PTP works

Here are some good references:

* [AlliedTelesis Doc: Precision Time Protocol & Transparent Clock](https://www.alliedtelesis.com/sites/default/files/ptp_feature_overview_guide_rev_a.pdf)
* [NetTimeLogic Doc: PTP Basics](https://www.nettimelogic.com/resources/PTP%20Basics.pdf)

You can either use a dedicated PTP grand master hardware or set up a Linux computer to act as the master. In this note, we mainly consider the latter case. 

## Relevant Packages

```bash
## install the packages
$ sudo apt install ethtool linuxptp chrony
```

The following list is taken from the Ouster documentation to give you a brief idea of the functions of the packages:

* **ethtool** - A tool to query the hardware and driver capabilities of a given Ethernet interface.
* **linuxptp** - Linux PTP package with the following components:
    - **ptp4l** daemon to manage hardware and participate as a PTP node
    - **phc2sys** to synchronize the Ethernet controller’s hardware clock to the Linux system clock or shared memory region
    - **pmc** to query the PTP nodes on the network.
* **chrony** - A NTP and PTP time synchronization daemon. It can be configured to listen to both NTP time sources via the Internet and a PTP master clock such as one provided by a GPS with PTP support. This will validate the time configuration makes sense given multiple time sources.

## Configure Linux PTP 

### Tutorials & Manual

* [Redhat Doc: Configuring PTP Using ptp4l](https://access.redhat.com/documentation/en-us/red_hat_enterprise_linux/6/html/deployment_guide/ch-configuring_ptp_using_ptp4l)
* [ptp4l (manual)](https://manpages.ubuntu.com/manpages/focal/en/man8/ptp4l.8.html)
* [phc2sys (manual)](https://manpages.ubuntu.com/manpages/focal/en/man8/phc2sys.8.html)
* [Ouster PTP Reference](https://static.ouster.dev/sensor-docs/image_route1/image_route3/appendix/ptp-quickstart.html)

### Command Reference

* Check for hardware and driver support

```bash
$ ethtool -T eth0
```

Ideally the ethernet adapter and driver should support hardware timestamp. You may get something like this:

```
Time stamping parameters for eth0:
Capabilities:
	hardware-transmit     (SOF_TIMESTAMPING_TX_HARDWARE)
	software-transmit     (SOF_TIMESTAMPING_TX_SOFTWARE)
	hardware-receive      (SOF_TIMESTAMPING_RX_HARDWARE)
	software-receive      (SOF_TIMESTAMPING_RX_SOFTWARE)
	software-system-clock (SOF_TIMESTAMPING_SOFTWARE)
	hardware-raw-clock    (SOF_TIMESTAMPING_RAW_HARDWARE)
PTP Hardware Clock: 0
Hardware Transmit Timestamp Modes:
	off                   (HWTSTAMP_TX_OFF)
	on                    (HWTSTAMP_TX_ON)
	one-step-sync         (HWTSTAMP_TX_ONESTEP_SYNC)
Hardware Receive Filter Modes:
	none                  (HWTSTAMP_FILTER_NONE)
	ptpv1-l4-sync         (HWTSTAMP_FILTER_PTP_V1_L4_SYNC)
	ptpv1-l4-delay-req    (HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ)
	ptpv2-l4-sync         (HWTSTAMP_FILTER_PTP_V2_L4_SYNC)
	ptpv2-l4-delay-req    (HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ)
	ptpv2-l2-sync         (HWTSTAMP_FILTER_PTP_V2_L2_SYNC)
	ptpv2-l2-delay-req    (HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ)
	ptpv2-event           (HWTSTAMP_FILTER_PTP_V2_EVENT)
```

Otherwise if only software timestamp is support, you may see

```
Time stamping parameters for eth0:
Capabilities:
	software-transmit
	software-receive
	software-system-clock
PTP Hardware Clock: none
Hardware Transmit Timestamp Modes: none
Hardware Receive Filter Modes: none
```

* Check synchronization between PHC and system time

```bash
$ sudo phc_ctl eth0 cmp
```

## Useful Documentation



## Relevant Projects

* [TSN Documentation Project for Linux](https://tsn.readthedocs.io/index.html)

## Reference

* [Ouster Sensor Docs (wiki)](https://static.ouster.dev/sensor-docs/index.html)
* [Ouster Lidar Software User Manual](https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2p0.pdf)
* [PTP for Mobile Robots](https://discourse.ros.org/t/experience-with-ptp-precision-time-protocol-for-mobile-robots/24707)