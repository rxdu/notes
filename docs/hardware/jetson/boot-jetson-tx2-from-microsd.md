---
title: Boot Jetson TX2/Nano from MicroSD Card
description: boot l4t system from SD card
---

This note may be useful if you're using a Jetson Nano/TX2 production module with 16G eMMC but want to boot from an SD card for more space.

## Install the bootloader

You first need to flash the eMMC normally with SDK manager. This process will install bootloader and firmware to the module.

## Copy Root Filesystem to SD Card

After the normal flashing process, you should be able to boot into the l4t system on eMMC. Insert the SD card to the Jetson board and format the card as "ext4" filesystem. Mount the SD card to the system. Here it's assumed that you mount the card to "/media/user/sdcard". Now you can copy the root filesystem to the SD card.

```bash
$ sudo rsync -axHAWX --numeric-ids --info=progress2 --exclude=/proc / /media/user/sdcard
```

## Modify Boot Sequence

For Jetpack 4.x (tested with Jetpack 4.6.2), the bootloader searches for the filesystem following the following order: SD card > eMMC > USB > NFS. This means if you want to boot from SD card, the boot configuration file "/boot/extlinux/extlinux.conf" on the SD card must be modified properly. What you need to do is to add a new boot entry, change root to the desired device (e.g. "/dev/mmcblk2p1") and update the default entry to be "microsd". The following is an example of the boot configuration file:

```
TIMEOUT 30
DEFAULT microsd

MENU TITLE L4T boot options

LABEL microsd
      MENU LABEL microsd kernel
      LINUX /boot/Image
      INITRD /boot/initrd
      APPEND ${cbootargs} quiet root=/dev/mmcblk2p1 rw rootwait rootfstype=ext4 console=ttyS0,115200n8 console=tty0 OS=l4t fbcon=map:0 net.ifnames=0

LABEL primary
      MENU LABEL primary kernel
      LINUX /boot/Image
      INITRD /boot/initrd
      APPEND ${cbootargs} quiet root=/dev/mmcblk0p1 rw rootwait rootfstype=ext4 console=ttyS0,115200n8 console=tty0 OS=l4t fbcon=map:0 net.ifnames=0 

# When testing a custom kernel, it is recommended that you create a backup of
# ...
# ...
```

Now reboot and verify your rootfilesystem is on the SD card.

```bash
$ df -h
```

## Reference

* [1] https://ttyusb0978.medium.com/nvidia-jetson-tx2-boot-menu-ad929204a70e
* [2] https://github.com/jetsonhacks/bootFromUSB/blob/main/copyRootToUSB.sh
* [3] https://elinux.org/Boot_from_sd