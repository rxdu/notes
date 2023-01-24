---
title: Beaglebone Setup
description: Basic setup for Beaglebone
---

## Flash Image to eMMC

Follow reference [1] to flash the OS to the eMMC storage. A brief summary:

* Download image and extract the .img file
* Create a bootable SD card with the image (using tools such as etcher)
* Boot into the OS on the SD card (holding the SD button and then power up the board)
* Modify "/boot/uEnv.txt" and uncomment the line that enables the flasher
* Shutdown the board and boot from the SD again, wait until the eMMC is flashed with the system

You can login to the system after the board boots up

```bash
$ ssh debian@192.168.7.2 # default password: temppwd
```

## Setup Ethernet over USB for Pocketbeagle

Add the following lines to /etc/network/interfaces by modifying the files directly after you mount the SD card to your computer

```
iface usb0 inet static
    address 192.168.7.2
    netmask 255.255.255.0
    gateway 192.168.7.1
    dns-nameservers 8.8.8.8
```

## Add New User

Now you can add your own account and delete the default one

```bash
$ sudo adduser --ingroup users <USERNAME>
$ sudo adduser <YOUR_USERNAME> sudo
$ logout
$ sudo deluser --remove-home user
```

## Setup Wifi

You can use "connman" to manage the wireless network. But I personally prefer using the old way.

* Disable connman

```bash
$ systemctl disable connman.service
```

* Generate WPA passphrase for your WiFi

```bash
$ wpa_passphrase <ssid> <password>
```

* Update iface usb0 inet static
    address 192.168.7.2
    netmask 255.255.255.0
    gateway 192.168.7.1
    dns-nameservers 8.8.8.8

```bash
$ sudo nano /etc/network/interfaces
```
```bash
# interfaces(5) file used by ifup(8) and ifdown(8)
auto lo
iface lo inet loopback

auto wlan0
iface wlan0 inet dhcp
    wpa-ssid ExampleWifi
    wpa-psk <wpa-psk-generated-by-wpa-passphrase-command>   
```

## Mount "/usr" and "/home" on SD card

The BeagleBone Blue has 4G on-board eMMC flash storage. It could be enough for many applications. But for experimental projects, it's more convenient to have more space so that you don't need to worry too much about filling the eMMC flash up when installing new packages.

Check reference [2] to get more detailed instructions. Here is a brief summary of what you need to do:

```bash
# 1. format your SD card into two partitions
# 2. insert the SD card and power up the board
# 3. check if you have the two partitions on the SD card recognized by the OS correctly
$ fdisk -l
# (the two partitions are labeled as "/dev/mmcblk0p1" and "/dev/mmcblk0p2" on my board)
# 4. format the two partions as ext4
$ mkfs.ext4 /dev/mmcblk0p1
$ mkfs.ext4 /dev/mmcblk0p2 
# 5. create temporary folders in /tmp and mount the two partitions to the temporary folders, copy files from /usr and /home accordingly
$ sudo mkdir /tmp/usr 
$ sudo mkdir /tmp/home
$ sudo mount /dev/mmcblk0p1 /tmp/usr 
$ sudo mount /dev/mmcblk0p2 /tmp/home
$ sudo rsync -ahv --progress /usr/ /tmp/usr/
$ sudo rsync -ahv --progress /home/ /tmp/home/ 
# 6. update /etc/fstab to reflect the change
$ sudo lsblk -no UUID /dev/mmcblk0p1 # get UUID of /dev/mmcblk0p1
# add two lines in /etc/fstab
$ UUID=<ID-YOU-GOT-FROM-ABOVE-CMD> /usr ext4    defaults 0 1
$ UUID=<ID-YOU-GOT-FROM-ABOVE-CMD> /home ext4    defaults 0 1
or
$ /dev/mmcblk0p1 /usr ext4 defaults 0 1
$ /dev/mmcblk0p2 /home ext4 defaults 0 1
# 7. reboot and check the files
$ df -h
```

## Add a Swap File

Sometimes a swap area may be needed to compile a large project on beaglebone black. Instead of setting up a swap partition, you can also use a swap file.

In the following steps, a swap file "/usr/swapfile1" will be created and used.

```bash
# 1. Create Storage File
$ sudo dd if=/dev/zero of=/usr/swapfile1 bs=1024 count=524288
# 2. Secure swap file
$ sudo chown root:root /usr/swapfile1
$ sudo chmod 0600 /usr/swapfile1
# 3. Set up a Linux swap area and enable the swap file
$ sudo mkswap /usr/swapfile1
$ sudo swapon /usr/swapfile1
# 4. Update /etc/fstab file
$ sudo nano /etc/fstab 
# Add the following line
/usr/swapfile1 none swap sw 0 0
```

Reboot the board and use htop to check if the swap file is set up correctly.

## Update Kernel

You can upgrade or change to a Preempt-RT patched kernel using the script provided in the Debian OS from Beaglebone

```bash
$ cd /opt/scripts/tools
$ git pull   # you may need to use sudo here
$ sudo /opt/scripts/tools/update_kernel.sh --bone-rt-channel --lts-4_4   # --lts-stable or other available options
```

Refer to [5] for additiona options.

Note that if you use the kernel from TI channel (for example, use "--ti-rt-channel" option), you might experience errors related to the PINMUX helper driver. 

## Robotics Cape library

TI channel kernels have more complete support to PRU drivers so you may want to use a kernel from the TI channel if you're using the Robotics Cape or Beaglebone Blue.[7]

When using the TI channel RT kernel 4.4 and 4.9 on Beaglebone Blue with the Robotics Cape library, I got the following error message from the library:

```bash
...
can't open: /sys/devices/platform/ocp/ocp:P9_24_pinmux/state
Pinmux: No such file or directory
WARNING: missing PINMUX driver
You probbaly just need a newer kernel
```

I'm not sure if this will happen to other boards or how to install this pinmux helper driver manually for the TI channel kernels. I resolved this issue by switching to a bone channel kernel.

The other issue related to the Robotics Cape library that occured even after installing the bone channel RT kernel is on the pru-rproc driver, which gives the error message

```bash
ERROR: pru-rproc driver missing
```

The solution is to add a symbolic link of clpru executable to /usr/share/ti/cgt-pru/bin folder:

```bash
$ cd /usr/share/ti/cgt-pru
$ mkdir bin
$ cd bin
$ ln -s /usr/bin/clpru clpru
```

See [6] for more discussions on this issue and the above solution is from this page.

**Reference**

* [1] https://github.com/beagleboard/beaglebone-blue/wiki/Flashing-firmware
* [2] https://gist.github.com/rxdu/35fb70c71cde5b44815e52bd8c338ff3
* [3] http://xeikonmirkwood.blogspot.com/2014/05/connman-is-annoying.html
* [4] http://pdkb.azurewebsites.net/Home/Detail/installing-can-bus-drivers-for-osd3358-systems-running-debian
* [5] https://github.com/beagleboard/bb.org-overlays
* [6] https://groups.google.com/forum/#!topic/beagleboard/od6h9yTKUD4
* [7] https://github.com/StrawsonDesign/librobotcontrol/issues/130
* [8] https://www.cyberciti.biz/faq/linux-add-a-swap-file-howto/