---
title: Flash Jetson Device with Virtualbox
description: How to flash Jetson Device with Virtualbox
---

**NOTE** The method described here is not reliable. In my case, it worked with Xavier NX dev kit but didn't work with the TX2 + Orbitty carrier board.

The main issue that may stop you from using an OS in virtualbox to flash a Jetson device using SDKManager is the USB setup. If the Jetson can be recognized properly in virtualbox, then the whole flashing process is basically the same with that in a native system.

General steps to follow:

* Install Virtualbox (tested with virtualbox v6.1)
* Install additional virtualbox supporting packages

```bash
$ sudo apt install virtualbox-ext-pack virtualbox-guest-additions-iso
```

* Add user in host system to "vboxusers" group

```bash
# you can check if your user is in vboxusers group
groups $USER
# if not, add to the group
$ sudo usermod -a -G vboxusers $USER
```

* Restart your computer and put the Jetson device into recovery mode. 

Now you should be able to add the Nvidia USB device to the virtualbox and flash the OS inside with SDKManager.
