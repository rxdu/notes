# Time Synchronization using PTP/IEEE1588

For a complicated robotic system, there may be multiple onboard computers and sensors working together to have the whole system functional. To ensure the data from different devices are synchronized in time, you may need to consider time synchronization among the devices. PTP is a synchronization protocol designed for this purpose and it allows sub-microsecond accuracy if properly configured. One example usage of PTP time synchronization is setting up an Ouster OS1-64 Lidar on a mobile robot for 3D mapping and navigation. To get the right timestamp for the pointcloud to be used with other parts of ROS stack, such as mapping and localization, a PTP grandmaster can be configured on the navigation computer to synchronize the time between Lidar and the computer.

## 1. Background Information

### 1.1 How PTP works

Here are some good references:

* [AlliedTelesis Doc: Precision Time Protocol & Transparent Clock](https://www.alliedtelesis.com/sites/default/files/ptp_feature_overview_guide_rev_a.pdf)
* [NetTimeLogic Doc: PTP Basics](https://www.nettimelogic.com/resources/PTP%20Basics.pdf)

You can either use a dedicated PTP grand master hardware or set up a Linux computer to act as the master. In this note, we mainly consider the latter case. 

### 1.2 PTP profiles

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

* The `default` profile is most commonly supported and used by computers and sensors on the robot
* The `gPTP` profile has more strict requirements in the setup (for both the devices and network switches)

### 1.3 Additional tutorials & references

* [Redhat Doc: Configuring PTP Using ptp4l](https://access.redhat.com/documentation/en-us/red_hat_enterprise_linux/6/html/deployment_guide/ch-configuring_ptp_using_ptp4l)
* [ptp4l (manual)](https://manpages.ubuntu.com/manpages/focal/en/man8/ptp4l.8.html)
* [phc2sys (manual)](https://manpages.ubuntu.com/manpages/focal/en/man8/phc2sys.8.html)
* [Ouster PTP Reference](https://static.ouster.dev/sensor-docs/image_route1/image_route3/appendix/ptp-quickstart.html)
* [PTP support in the Raspberry Pi CM4 and CM5](https://github.com/jclark/rpi-cm4-ptp-guide)
* [TSN Documentation Project for Linux](https://tsn.readthedocs.io/index.html)
* [Raspberry Pi CM4 PTP Guide](https://github.com/jclark/rpi-cm4-ptp-guide)

## 2. Configure PTP in Linux 

### 2.1 Sofware packages

You may use `ethtool`, `linuxptp` and `chrony` in the setup. They can be installed from apt-get directly:

```bash
## install the packages
$ sudo apt install ethtool linuxptp 

## only install if needed
sudo apt install chrony
```

The following list is taken from the Ouster documentation to give you a brief idea of the functions of the packages:

* **ethtool** - A tool to query the hardware and driver capabilities of a given Ethernet interface.
* **linuxptp** - Linux PTP package with the following components:
    - **ptp4l** daemon to manage hardware and participate as a PTP node
    - **phc2sys** to synchronize the Ethernet controller’s hardware clock to the Linux system clock or shared memory region
    - **pmc** to query the PTP nodes on the network.
    - **ts2phc** to synchronize PTP Hardware Clocks (PHC) to external time stamp signals.
* **chrony** - A NTP and PTP time synchronization daemon. It can be configured to listen to both NTP time sources via the Internet and a PTP master clock such as one provided by a GPS with PTP support. This will validate the time configuration makes sense given multiple time sources.

### 2.2 Check hardware and driver support

You can use the following command to check the hardware and driver support of the network interface:

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

### 2.3 General workflow

In general, what we try to achieve in time synchronization for devices within a robot include:

* all these devices share the same time base after synchronization (with bounded time offset)
* the system time on all devices should not jump backwards (monotonically increasing)

The general synchonization setup is illustrated as follows:

```
              [ Internet NTP Servers (optional) ]
                             |
                             v
        +-----------------------------------------------+
        |          🧠 PTP Grandmaster Node              |
        |   (Main onboard PC with PHC-capable NIC)      |
        |-----------------------------------------------|
        |                                               |
        |  1. Chrony disciplines CLOCK_REALTIME         |
        |     - One-time step (makestep 1.0 1)          |
        |     - Then slews slowly to avoid jumps        |
        |                                               |
        |  2. phc2sys pushes CLOCK_REALTIME to PHC:     |
        |     phc2sys -w -s CLOCK_REALTIME -c eth0      |
        |     -> Ensures PHC stays aligned to system    |
        |                                               |
        |  3. ptp4l advertises PHC to PTP network:      |
        |     ptp4l -i eth0 -f /etc/ptp4l.conf          |
        |     -> Acts as Grandmaster                    |
        +-----------------------------------------------+
                             |
                             |   (PTP over Ethernet)
                             v
        +-----------------------------------------------+
        |       🧩 PTP Slave Nodes (Aux PCs, Jetsons)   |
        |-----------------------------------------------|
        |                                               |
        |  1. ptp4l syncs PHC from Grandmaster:         |
        |     ptp4l -s -i eth0                          |
        |                                               |
        |  2. phc2sys updates system clock:             |
        |     phc2sys -w -s eth0 -c CLOCK_REALTIME      |
        |                                               |
        |  -> CLOCK_REALTIME is now aligned with GM     |
        +-----------------------------------------------+
                             |
                             v
        +-----------------------------------------------+
        |        🎥 Sensors with PTP support (optional) |
        |      (e.g., Ouster LiDAR, FLIR cameras)       |
        |-----------------------------------------------|
        |  Internal PTP slave logic or hardware         |
        |  syncs device clock from network              |
        |  -> Ensures timestamps align with robot clocks|
        +-----------------------------------------------+
```

### 2.4 PTP grandmaster setup

#### 2.4.1 Configure Chrony to synchonize system time from the NTP servers

This step is optional as all devices can still work with a local time base. But in most cases, we may want the robot system to use the correct global time for convenience (e.g. when checking and retrieving logs from the robot). This can be achived with Chrony. 

Edit `/etc/chrony/chrony.conf`:

```
# Step the system clock instead of slewing it if the adjustment is larger than
# one second, but only in the first clock update (default 3 => 1).
makestep 1 1
```    

The main change we make is to make sure chony only steps once at the start (before the robot starts working) to avoid the system time to jump backwards.

You can use the following commands to check the chrony synchonization status:

```bash
$ chronyc tracking
$ chronyc sources -v
```

#### 2.4.2 Configure phc2sys to synchronize the system time to the PTP clock

This step synchonizes time from the system clock to the PHC. Otherwise, the initial value of the PHC time is undefined that you may see nonsense values from it. You can check the system time and PHC time with the following commands: 

```bash
# system time
$ date

# PHC time on eth0
$ sudo phc_ctl eth0 get
```

1. Create a systemd service for phc2sys: /etc/systemd/system/phc2sys.service

    ```
    [Unit]
    Description=Synchronize system clock or PTP hardware clock (PHC)
    Documentation=man:phc2sys
    After=chrony.service

    [Service]
    # -s: master clock, -c: slave clock
    ExecStart=/usr/sbin/phc2sys -s CLOCK_REALTIME -c eth0

    [Install]
    WantedBy=multi-user.target
    ```

2. Start and enable the phc2sys service

    ```
    $ sudo systemctl daemon-reload
    $ sudo systemctl start phc2sys.service
    # if you get no errors starting the service
    $ sudo systemctl enable phc2sys.service
    ```

#### 2.4.3 Set up ptp clock master

1. Update the configuration file /etc/linuxptp/ptp4l.conf

    If you're setting  up a master clock, change the following lines:

    ```
    #clockClass     248
    clockClass      128
    ```
    clockClass with a value of 128 indicates that the clock you're setting up is free-running and not synchonized to an external reference source (e.g. GPS time)
 

    If you have multiple network intefaces on the computer and you want the computer to act as a boundary clock:
    ```
      # at very bottom of the file
    boundary_clock_jbod 1
    [eth0]
    [eth1]
    ```
    Typically you won't need this configuration as it only gives you multiple isolated PTP networks (where the name "boundary" is from). 
    
    If what you really want is to synchonize time bewteen the interfaces, you still need to further configure the time using `phc2sys`.

    ```bash
    # e.g. set eth0 time to eth1
    phc2sys -w -s eth0 -c eth1
    ```    

2. Create a systemd service for ptp4l: /etc/systemd/system/ptp4l.service

    Make sure you update the interface name in the `-i eth0` part

    ```
    [Unit]
    Description=ptp4l service
    Requires=network-online.target
    After=network-online.target

    [Service]
    ExecStart=/usr/sbin/ptp4l -i eth0 -f /etc/linuxptp/ptp4l.conf
    Restart=on-failure

    [Install]
    WantedBy=multi-user.target
    ```

3. Start and enable the ptp4l service

    ```
    $ sudo systemctl daemon-reload
    $ sudo systemctl start ptp4l.service
    # if you get no errors starting the service
    $ sudo systemctl enable ptp4l.service
    ```

### 2.5 PTP slave setup

#### 2.5.1 Set up PTP slave

1. Update the configuration file /etc/linuxptp/ptp4l.conf

    If you're setting up a slave device, change the following lines:
    ```
    #slaveOnly              0
    slaveOnly               1
    ```

2. Create a systemd service for ptp4l: /etc/systemd/system/ptp4l.service

    Make sure you update the interface name in the `-i eth0` part

    ```
    [Unit]
    Description=ptp4l service
    Requires=network-online.target
    After=network-online.target

    [Service]
    ExecStart=/usr/sbin/ptp4l -i eth0 -f /etc/linuxptp/ptp4l.conf
    Restart=on-failure

    [Install]
    WantedBy=multi-user.target
    ```

3. Start and enable the ptp4l service

    ```
    $ sudo systemctl daemon-reload
    $ sudo systemctl start ptp4l.service
    # if you get no errors starting the service
    $ sudo systemctl enable ptp4l.service
    ```

#### 2.5.2 Configure phc2sys to synchronize the PTP clock to the system time

1. Create a systemd service for phc2sys: /etc/systemd/system/phc2sys.service

    ```
    [Unit]
    Description=Synchronize system clock or PTP hardware clock (PHC)
    Documentation=man:phc2sys
    Requires=ptp4l.service
    After=ptp4l.service

    [Service]
    ExecStart=/usr/sbin/phc2sys -w -s eth0 -c CLOCK_REALTIME

    [Install]
    WantedBy=multi-user.target
    ```

2. Start and enable the phc2sys service

    ```
    $ sudo systemctl daemon-reload
    $ sudo systemctl start phc2sys.service
    # if you get no errors starting the service
    $ sudo systemctl enable phc2sys.service
    ```

3. Make sure you have disabled all NTP clients that may change the system clock
 
    ```bash
    sudo systemctl disable --now chrony
    sudo systemctl disable --now systemd-timesyncd
    sudo systemctl disable --now ntp
    ```

### 2.6 Check clock synchronization 

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

More information about clock synchronization check can be found from [6].

## 3. Additional Information

* https://botblox.io/collections/frontpage

## Reference

* [1] [Ouster Sensor Docs (wiki)](https://static.ouster.dev/sensor-docs/index.html)
* [2] [Ouster Lidar Software User Manual](https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2p0.pdf)
* [3] [PTP for Mobile Robots](https://discourse.ros.org/t/experience-with-ptp-precision-time-protocol-for-mobile-robots/24707)
* [4] [Combining PTP with NTP to Get the Best of Both Worlds](https://www.redhat.com/en/blog/combining-ptp-ntp-get-best-both-worlds) 
* [5] [Synchronize to PTP or NTP Time Using timemaster](https://access.redhat.com/documentation/en-us/red_hat_enterprise_linux/6/html/deployment_guide/sec-synchronize_to_ptp_or_ntp_time_using_timemaster)
* [6] [Synchronizing Time with Linux PTP](https://tsn.readthedocs.io/timesync.html#checking-clocks-synchronization)
* [7] [Multi-robot coordination: Distributed synchronization of industrial robots through ROS 2](https://discourse.ros.org/t/multi-robot-coordination-distributed-synchronization-of-industrial-robots-through-ros-2/9288)