---
title: Xenomai on Beaglebone
description: Build xenomai for Beaglebone
---

Build Xenomai-patched Linux for Beaglebone Black[1]. The general building steps for other ARM-based boards should be similar but specific details related to hardware can be drastically different.

#### Install toolchain

If you use a pre-built root file system, make sure you're using a compatible toolchain to compile bootloader and kernel.

```bash
$ wget -c https://releases.linaro.org/components/toolchain/binaries/6.4-2018.05/arm-linux-gnueabihf/gcc-linaro-6.4.1-2018.05-x86_64_arm-linux-gnueabihf.tar.xz
$ tar xf gcc-linaro-6.4.1-2018.05-x86_64_arm-linux-gnueabihf.tar.xz
```

#### Install dependencies

```bash
$ sudo apt install bison flex lzop u-boot-tools
```

#### Build bootloader

```bash
$ git clone https://github.com/u-boot/u-boot
$ cd u-boot/
$ git checkout v2018.09 -b tmp-v2018.09

$ wget -c https://rcn-ee.com/repos/git/u-boot-patches/v2018.09/0001-am335x_evm-uEnv.txt-bootz-n-fixes.patch
$ wget -c https://rcn-ee.com/repos/git/u-boot-patches/v2018.09/0002-U-Boot-BeagleBone-Cape-Manager.patch

$ patch -p1 < 0001-am335x_evm-uEnv.txt-bootz-n-fixes.patch
$ patch -p1 < 0002-U-Boot-BeagleBone-Cape-Manager.patch

$ make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- distclean
$ make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- am335x_evm_defconfig
$ make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- 
```

You should get "MLO", "u-boot.img" in the root folder of u-boot after successful compilation.

#### Buid Kernel

Download kernel source file

```bash
$ git clone https://github.com/beagleboard/linux.git

# look for the desired branch, for example
$ git tag -l | grep 4.9

# checkout the desired version into a new branch, use "4.9.88-ti-xenomai-r107" as an example
$ git checkout 4.9.88-ti-xenomai-r107 -b tmp-4.9.88-ti-xenomai-r107
```

Start the configuration menu and modify kernel options

```bash
$ make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- bb.org_defconfig 
$ make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- menuconfig
```

Check the following configurations for Xenomai

```bash
* General setup ---> 
- Local version - append to kernel release : "-ti-xenomai-r107"
- Stack Protector buffer overflow detection: Disable 
* Kernel Features  --->
- Preemption Model : select "Preemptible Kernel (Low-Latency Desktop)"                           
- Timer frequency : change to 1000Hz
- [] Allow for memory compaction : Disable
- [] Contiguous Memory Allocator : Disable
* CPU Power Management  --->
- [] CPU Frequency scaling : Disable
    
* Kernel hacking  --->
- [] KGDB: kernel debugger : Disable
```

You could also enable Xenomai supported device drivers in the config according to your needs.

Build kernel image:

```bash
$ make -j8 ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- zImage dtbs
$ make -j8 ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- modules
```

If you need to build uImage

```bash
$ make -j8 ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- uImage LOADADDR=0x80008000
```

You can also build debian packages for easier installation (for example, update kernel on the eMMC)

```bash
$ make -j8 ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- deb-pkg
```

If you want to add a custom string identifier to your kernel name, you can add the "LOCALVERSION" parameter

```bash
$ make -j8 ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- deb-pkg LOCALVERSION=-rt
```

Alternatively, you could deploy to the SD card manually.

#### Prepare SD card

Create 2 partitions

* /boot formated in FAT32 with "boot" flag 
* /rootfs formated in ext4

Set SD card location and kernel version for installation

```bash
$ export SD_ROOTFS=/media/rdu/rootfs
$ export kernel_version=4.9.88-ti-xenomai-r107 
```

#### Deploy u-boot

You need to copy "MLO", "u-boot.img", and the "uEnv.txt" into the /boot partitiona on the SD card.

```bash
##Rename as: uEnv.txt to override old bootloader in eMMC
##These are needed to be compliant with Angstrom's 2013.06.20 u-boot.

loadaddr=0x82000000
fdtaddr=0x88000000
rdaddr=0x88080000

initrd_high=0xffffffff
fdt_high=0xffffffff

##These are needed to be compliant with Debian 2014-05-14 u-boot.

loadximage=echo debug: [/boot/vmlinuz-${uname_r}] ... ; load mmc 0:2 ${loadaddr} /boot/vmlinuz-${uname_r}
loadxfdt=echo debug: [/boot/dtbs/${uname_r}/${fdtfile}] ... ;load mmc 0:2 ${fdtaddr} /boot/dtbs/${uname_r}/${fdtfile}
loadxrd=echo debug: [/boot/initrd.img-${uname_r}] ... ; load mmc 0:2 ${rdaddr} /boot/initrd.img-${uname_r}; setenv rdsize ${filesize}
loaduEnvtxt=load mmc 0:2 ${loadaddr} /boot/uEnv.txt ; env import -t ${loadaddr} ${filesize};
check_dtb=if test -n ${dtb}; then setenv fdtfile ${dtb};fi;
check_uboot_overlays=if test -n ${enable_uboot_overlays}; then setenv enable_uboot_overlays ;fi;
loadall=run loaduEnvtxt; run check_dtb; run check_uboot_overlays; run loadximage; run loadxrd; run loadxfdt;

mmcargs=setenv bootargs console=tty0 console=${console} ${optargs} ${cape_disable} ${cape_enable} root=/dev/mmcblk0p2 rootfstype=${mmcrootfstype} ${cmdline}

uenvcmd=run loadall; run mmcargs; echo debug: [${bootargs}] ... ; echo debug: [bootz ${loadaddr} ${rdaddr}:${rdsize} ${fdtaddr}] ... ; bootz ${loadaddr} ${rdaddr}:${rdsize} ${fdtaddr};
```
Make sure you specify the right path to the rootfs in "/boot/uEnv.txt". In the uEnv.txt shown above, it is "root=/dev/mmcblk0p2".

#### Root file system

You can use tools like buildroot to make your customized root file system or you can use debootstrap or multistrap to build your Debian/Ubuntu distribution. Here I use the pre-built debian for Beaglebone.

Copy pre-built root file system to the SD card

```bash
$ wget -c https://rcn-ee.com/rootfs/eewiki/minfs/debian-9.5-minimal-armhf-2018-07-30.tar.xz
$ tar xf debian-9.5-minimal-armhf-2018-07-30.tar.xz
$ sudo tar xfvp ./*-*-*-armhf-*/armhf-rootfs-*.tar -C /media/rootfs/
$ sync
$ sudo chown root:root /media/rootfs/
$ sudo chmod 755 /media/rootfs/
```

#### Deploy kernel

Install kernel and device tree:

```bash
# kernel
$ sudo cp -v ./arch/arm/boot/zImage ${SD_ROOTFS}/boot/vmlinuz-${kernel_version}

# device tree
$ sudo mkdir -p ${SD_ROOTFS}/boot/dtbs/${kernel_version}
$ sudo cp -v ./arch/arm/boot/dts/am335x* ${SD_ROOTFS}/boot/dtbs/${kernel_version}
```

Install headers:

```bash
$ make -j8 ARCH=arm headers_check
$ sudo make -j8 ARCH=arm INSTALL_HDR_PATH=${SD_ROOTFS} headers_install
```

Install modules and firmware

```bash
$ sudo make -j8 ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- modules_install INSTALL_MOD_PATH=${SD_ROOTFS}
$ sudo make -j8 ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- firmware_install INSTALL_MOD_PATH=${SD_ROOTFS}
```

Update "/rootfs/boot/uEnv.txt":

```bash
uname_r=4.9.88-ti-xenomai-r107
```

#### Additional configurations

Update fstab (Note you may need to change "/dev/mmcblk0p2" according to your SD configuration)

```bash
$ sudo sh -c "echo '/dev/mmcblk0p2  /  auto  errors=remount-ro  0  1' >> /media/rootfs/etc/fstab"
```

Update hostname (default is "arm")

```bash
$ sudo nano /etc/hostname
$ sudo nano /etc/hosts
```

Update network interface 

```bash
$ sudo nano /etc/network/interfaces
```

```bash
auto lo
iface lo inet loopback

auto eth0
iface eth0 inet dhcp
```

At this point, you should have a bootable Linux running on the Beaglebone.

#### Install Xenomai tools and libraries

First you need to check which version of Xenomai is install with the kernel. 

```bash
$ cat /proc/xenomai/version
```

In this case, the version of Xenomai is 3.0.6. Download code from the Xenomai website. 

Install dependencies

```bash
$ sudo apt-get install build-essential libtool libltdl-dev
```

Finally build the code

```bash
$ ./scripts/bootstrap
$ ./configure --enable-smp --with-core=cobalt
$ make
$ sudo make install
```

Once finished, you can test if the installation is successful

```bash
$ cd /usr/xenomai/bin
$ sudo ./latency
```
You should get something like

```bash
== Sampling period: 1000 us
== Test mode: periodic user-mode task
== All results in microseconds
warming up...
RTT|  00:00:01  (periodic user-mode task, 1000 us period, priority 99)
RTH|----lat min|----lat avg|----lat max|-overrun|---msw|---lat best|--lat worst
RTD|      4.041|      6.969|     28.750|       0|     0|      4.041|     28.750
RTD|      4.083|      8.077|     39.708|       0|     0|      4.041|     39.708
RTD|      4.041|      7.884|     40.541|       0|     0|      4.041|     40.541
RTD|      4.082|      7.033|     37.291|       0|     0|      4.041|     40.541
RTD|      4.082|      8.122|     42.165|       0|     0|      4.041|     42.165
RTD|      4.082|      7.020|     37.249|       0|     0|      4.041|     42.165
RTD|      4.082|      8.145|     42.457|       0|     0|      4.041|     42.457
RTD|      3.831|      7.878|     41.415|       0|     0|      3.831|     42.457
RTD|      4.040|      6.717|     32.665|       0|     0|      3.831|     42.457
```

**Reference**

* [1] https://www.digikey.com/eewiki/display/linuxonarm/BeagleBone+Black
* [2] http://www.simplerobot.net/2018/06/build-realtime-xenomai-3-kernel-for.html
* [3] https://lemariva.com/blog/2018/07/raspberry-pi-xenomai-patching-tutorial-for-kernel-4-14-y
* [4] https://stackoverflow.com/questions/21740619/how-can-i-generate-kernel-headers-for-an-unknown-embedded-arm-system
* [5] https://www.vultr.com/docs/how-to-change-your-hostname-on-debian