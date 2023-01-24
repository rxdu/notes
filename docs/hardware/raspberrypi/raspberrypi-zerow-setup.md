# Basic setup for Raspberry Pi Zero W

## Setup Raspberry Pi Zero W

### SSH via USB and serial console

1. Download the raspbian image and flash to the SD card. Mount the SD card on the host computer. You should see two partitions "boot" and "rootfs".

2. Edit the file "/boot/config.txt" and add the following line to the end of the text:

```
dtoverlay=dwc2
# add the following line if you also want serial console
enable_uart=1
```

3. Edit the file "/boot/cmdline.txt", add "modules-load=dwc2,g_ether" after rootwait. It's a one-line text file and be careful about the format. After the change, it should look something like:

```
dwc_otg.lpm_enable=0 console=serial0,115200 console=tty1 root=/dev/mmcblk0p2 rootfstype=ext4 elevator=deadline fsck.repair=yes rootwait modules-load=dwc2,g_ether
```

4. Create a new file named "ssh" without any suffix to enable ssh.

```
$ touch ssh
```

5. USB network configuration. Edit "rootfs/etc/network/interfaces" and add the following lines:

```
allow-hotplug usb0
iface usb0 inet static
    address 192.168.7.2
    netmask 255.255.255.0
    network 192.168.7.0
    broadcast 192.168.7.255
```

6. Now you can plug the SD card back to rpi0W and boot the board. If the host computer fails to get an IP address for the USB connection with rpi, you need to manually assign one:

```
# use ifconfig to check the name of the USB connection with rpi first
$ sudo ifconfig enp0s20f0u2 192.168.7.3 netmask 255.255.255.0
```

### Patch kernel with Xenomai 3

Before pactching Xenomai 3, ther kernel shipped with "2018-06-27-raspbian-stretch-lite.img" is "4.14.50+". Afterwards the kernel is "4.14.37-ipipe+".

### Additional commands
  
```
# disable dhcpcd service
$ sudo systemctl disable dhcpcd.service 
$ sudo systemctl enable wpa_supplicant.service
```

Reference:

* https://raspberrypi.stackexchange.com/questions/66431/headless-pi-zero-ssh-access-over-usb
* https://learn.adafruit.com/turning-your-raspberry-pi-zero-into-a-usb-gadget/ethernet-gadget
* https://learn.adafruit.com/raspberry-pi-zero-creation/give-it-life
* https://www.thepolyglotdeveloper.com/2017/02/connect-raspberry-pi-pi-zero-usb-ttl-serial-cable/
* https://www.raspberrypi.org/documentation/configuration/uart.md