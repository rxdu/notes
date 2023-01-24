# CAN Bus in Linux

## Bring up CAN Interface

Note that the maximum bitrate depends on your specific hardware, for example PiCAN2 supports up to 1Mbps.

```bash
$ sudo ip link set can0 up type can bitrate 1000000
```

To automatically bring up the interface during boot, you can modify "/etc/network/interfaces"

```bash
$ sudo nano /etc/network/interfaces
```

Add the following lines

```bash
auto can0
    iface can0 inet manual
    pre-up /sbin/ip link set can0 type can bitrate 1000000 
    up /sbin/ifconfig can0 up
    post-up /sbin/ip link set can0 txqueuelen 10000
    down /sbin/ifconfig can0 down
```

You can check if CAN is brought up by using command "ifconfig". You should see something similar to this

```bash
$ can0: flags=193<UP,RUNNING,NOARP>  mtu 16
$     unspec 00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00  txqueuelen 10  (UNSPEC)
$     RX packets 4  bytes 32 (32.0 B)
$     RX errors 0  dropped 0  overruns 0  frame 0
$     TX packets 4  bytes 32 (32.0 B)
$     TX errors 1  dropped 1 overruns 0  carrier 1  collisions 0
```

## Loopback Test

You can use the loopback mode to test if PiCAN2 is working properly. You need to enable the loopback mode first.

```bash
$ sudo ip link set can0 down
$ sudo ip link set can0 type can loopback on
$ sudo ip link set can0 up type can bitrate 1000000
```

Now open two terminals, one as sender and one as receiver.

In sender terminal:

```bash
$ cansend can0 001#1122334455667788
```

In receiver terminal:

```bash
$ candump can0
```

If successful, you should expect

```bash
$ candump can0
$ can0  001   [8]  11 22 33 44 55 66 77 88
$ can0  001   [8]  11 22 33 44 55 66 77 88
```

Turn off the loopback mode:

```bash
$ sudo ip link set can0 down
$ sudo ip link set can0 type can loopback off
$ sudo ip link set can0 up type can bitrate 1000000
```

## Log Frames and Playback

You can log can frames from a CAN interface to a file for debugging

```bash
$ candump -l can0
```

Then you can playback the log file

```bash
$ canplayer -I <candump-log-file-name>.log
```

## Dump Frames with Filter Mask

Dump can frames with filter

```bash
$ candump can0,<interested-can-id(s)>:<filter-mask>
```

Example:

```bash
$ candump can0,602:1fffffff
```

The example command dumps CAN message with ID 0x602 from can0. If you wang to dump all messages with ID 0x60x, you can adjust the filter to be 1ffffff0.

## More Commands

Use this command to find more information of CAN related commands

```bash
$ ip link set can0 up type can help
```

## Reference

* [1] http://copperhilltech.com/pican2-controller-area-network-can-interface-for-raspberry-pi/
* [2] https://raspberrypi.stackexchange.com/questions/51829/unable-to-bring-can-interface-up-on-raspberry-pi-3
* [3] https://www.raspberrypi.org/forums/viewtopic.php?t=141052
* [4] https://harrisonsand.com/can-on-the-raspberry-pi/
* [5] https://dayba.wordpress.com/2017/05/25/playing-with-socketcan-using-can-utils/
* [6] http://www.cse.dmu.ac.uk/~eg/tele/CanbusIDandMask.html