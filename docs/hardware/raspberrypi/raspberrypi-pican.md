# PiCAN2 on Raspberry Pi 3

(Tested on Raspberry Pi3 with Raspbian Stretch 2017-09-07)

## Setup driver

```bash
$ sudo nano /boot/config.txt
```

Add the following lines

```bash
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
dtoverlay=spi-bcm2835
```

Reboot the board.

## Bring up CAN interface

Note that the maximum bitrate is 1000000 for PiCAN2.

```bash
$ sudo ip link set can0 up type can bitrate 1000000
```

To bring up the interface when boot, you can modify "/etc/network/interfaces"

```bash
$ sudo nano /etc/network/interfaces
```

Add the following lines

```bash
auto can0
     iface can0 inet manual
     pre-up /sbin/ip link set can0 type can bitrate 1000000 
     up /sbin/ifconfig can0 up
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

## Loopback test

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

## More commands

Use this command to find more information of CAN related commands

```bash
$ ip link set can0 up type can help
```

## Reference

* [1] http://copperhilltech.com/pican2-controller-area-network-can-interface-for-raspberry-pi/
* [2] https://raspberrypi.stackexchange.com/questions/51829/unable-to-bring-can-interface-up-on-raspberry-pi-3
* [3] https://www.raspberrypi.org/forums/viewtopic.php?t=141052
* [4] https://harrisonsand.com/can-on-the-raspberry-pi/
