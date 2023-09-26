# Time Synchronization using PTP/IEEE1588

For a complicated robotic system, there may be multiple onboard computers and sensors working together to have the whole system functional. To ensure the data from different devices are synchronized in time, you may need to consider time synchronization among the devices. PTP is a synchronization protocol designed for this purpose and it allows sub-microsecond accuracy if properly configured. One example usage of PTP time synchronization is setting up an Ouster OS1-64 Lidar on a mobile robot for 3D mapping and navigation. To get the right timestamp for the pointcloud to be used with other parts of ROS stack, such as mapping and localization, a PTP grandmaster can be configured on the navigation computer to synchronize the time between Lidar and the computer.

## How PTP works

Here are some good references:

* [AlliedTelesis Doc: Precision Time Protocol & Transparent Clock](https://www.alliedtelesis.com/sites/default/files/ptp_feature_overview_guide_rev_a.pdf)
* [NetTimeLogic Doc: PTP Basics](https://www.nettimelogic.com/resources/PTP%20Basics.pdf)

You can either use a dedicated PTP grand master hardware or set up a Linux computer to act as the master. In this note, we mainly consider the latter case. 

## PTP Profiles

The following table is taken from [4] by peci1 published on ROS Discourse:

|       Profile        | BMCA  | Delay mech. | Layer |   
| :------------------: | :---: | :---------: | :---: | 
|       Default        |  Yes  |   P2P/E2E   | L2/L3 | 
|  gPTP (802.1AS) 17   |  Yes  |     P2P     |  L2   |  
| Automotive        10 |  No   |     P2P     |  L2   |   
|      Autosar 9       |  No   |     P2P     |  L2   |  
|        LXI 1         |  Yes  |     P2P     |  L3   |   
|  IEC 62439-3 L2P2P   |  Yes  |     P2P     |  L2   |    
|  IEC 62439-3 L3E2E   |  Yes  |     E2E     |  L3   |   
|    Power Profile     |  Yes  |     P2P     |  L2   |   
|    GigE Vision 11    |  Yes  |   P2P/E2E   |   ?   |  

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

### Checking Clocks Synchronization [6]

* Check time synchronization between PHC and the Grandmaster clock

    Look at output of the **ptp4l**:

    ```
    ptp4l[5374018.735]: rms  787 max 1208 freq -38601 +/- 1071 delay  -14 +/-   0
    ptp4l[5374019.735]: rms 1314 max 1380 freq -36204 +/- 346 delay   -14 +/-   0
    ptp4l[5374020.735]: rms  836 max 1106 freq -35734 +/-  31 delay   -14 +/-   0
    ptp4l[5374021.736]: rms  273 max  450 freq -35984 +/-  97 delay   -14 +/-   0
    ptp4l[5374022.736]: rms   50 max   82 freq -36271 +/-  64 delay   -14 +/-   0
    ptp4l[5374023.736]: rms   81 max   86 freq -36413 +/-  17 delay   -14 +/-   0
    ```

    If ptp4l consistently reports rms lower than 100 ns, the PHC is synchronized.

* Check time synchronization between the PHC and the system clock

    Look at output of the **phc2sys**:

    ```
    phc2sys[5374168.545]: CLOCK_REALTIME phc offset   -372582 s0 freq    +246 delay   6649
    phc2sys[5374169.545]: CLOCK_REALTIME phc offset   -372832 s1 freq      -4 delay   6673
    phc2sys[5374170.547]: CLOCK_REALTIME phc offset        68 s2 freq     +64 delay   6640
    phc2sys[5374171.547]: CLOCK_REALTIME phc offset       -20 s2 freq      -3 delay   6687
    phc2sys[5374172.547]: CLOCK_REALTIME phc offset        47 s2 freq     +58 delay   6619
    phc2sys[5374173.548]: CLOCK_REALTIME phc offset       -40 s2 freq     -15 delay   6680
    ```

    If phc2sys consistently reports offset lower than 100 ns, the System clock is synchronized.

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

## Relevant Projects

* [TSN Documentation Project for Linux](https://tsn.readthedocs.io/index.html)

## Reference

* [1] [Ouster Sensor Docs (wiki)](https://static.ouster.dev/sensor-docs/index.html)
* [2] [Ouster Lidar Software User Manual](https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2p0.pdf)
* [3] [PTP for Mobile Robots](https://discourse.ros.org/t/experience-with-ptp-precision-time-protocol-for-mobile-robots/24707)
* [4] [Combining PTP with NTP to Get the Best of Both Worlds](https://www.redhat.com/en/blog/combining-ptp-ntp-get-best-both-worlds) 
* [5] [Synchronize to PTP or NTP Time Using timemaster](https://access.redhat.com/documentation/en-us/red_hat_enterprise_linux/6/html/deployment_guide/sec-synchronize_to_ptp_or_ntp_time_using_timemaster)
* [6] [Synchronizing Time with Linux PTP](https://tsn.readthedocs.io/timesync.html#checking-clocks-synchronization)