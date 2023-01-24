# Xenomai on Raspberry Pi

## Setup Xenomai on Raspberry Pi ZeroW

* Install toolchain

```
$ sudo apt-get install gcc-arm-linux-gnueabihf
$ sudo apt-get install --no-install-recommends ncurses-dev bc
```

* Create a workspace folder to keep everything at one place

```
$ cd <Your-Workspace-Directory>
$ mkdir linux_xenomai
$ cd linux_xenomai
```

* Download Linux kernel for Raspberry Pi 
  
```
$ git clone https://github.com/raspberrypi/linux.git --depth 3
```

* Download Xenomai source code 

```
$ wget https://xenomai.org/downloads/xenomai/stable/xenomai-3.0.7.tar.bz2
$ ln -s xenomai-3.0.7 xenomai
```

Extract files and modify "xenomai/scripts/prepare-kernel.sh": replace "ln -sf" with "cp" so that it will copy all neccessary xenomai files to linux source

* Download Xenomai kernel patch

Download the kernel patch from https://xenomai.org/downloads/ipipe/ . For best compatibility, choose the kernel and patch with similar version number. Otherwise, you may have to fix issues by yourself.

```
$ wget https://xenomai.org/downloads/ipipe/v4.x/arm/ipipe-core-4.14.36-arm-1.patch
```

At the time of writing, patch for kernel 4.14.36 is the latest one for downloading. Accordingly we need to switch to a different commit of the linux kernel for RPi for least compatibility issues. Choose 4.14.37 in our case:

```
$ cd linux
$ git reset --hard 29653ef5475124316b9284adb6cbfc97e9cae48f
```

You need to replace the following two files in "linux/drivers/irqchip/" to patch the kernel successfully:

```
linux/drivers/irqchip/irq-bcm2835.c
linux/drivers/irqchip/irq-bcm2836.c
```

Find the modified files from: https://github.com/lemariva/RT-Tools-RPi

* Patch the kernel

```
$ cd linux_xenomai
$ ./xenomai/scripts/prepare-kernel.sh --linux=./linux --arch=arm --ipipe=./ipipe-core-4.14.36-arm-1.patch
```

If the patching is successful, you should get something like:

```
...
I-pipe core/arm #1 installed.
Links installed.
Build system ready.
```

* Configure and build patched kernel (for Raspberry Pi ZeroW)

```
$ cd linux
$ make -j8 O=build ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- bcmrpi_defconfig
make O=build ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -j8 menuconfig
```

Make changes according to the instructions in [1].

Build kernel image, modules and device overlay

```
$ make O=build ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -j8 bzImage modules dtbs
$ make O=build ARCH=arm INSTALL_MOD_PATH=dist -j8 modules_install
```

* Deploy the kernel

You can either configure the new kernel manually like [2] or build the kernel/headers to be ".deb" packages like [1].

* Compiling Xenomai tools

```
$ cd xenomai
$ ./scripts/bootstrap
$ ./configure --host=arm-linux-gnueabihf --disable-smp --with-core=cobalt
$ make
$ sudo make install
```

## Reference:

* [1] https://github.com/thanhtam-h/rpi01-4.1.21-xeno3/tree/master/scripts
* [2] https://lemariva.com/blog/2018/07/raspberry-pi-xenomai-patching-tutorial-for-kernel-4-14-y
* [3] https://gitlab.denx.de/Xenomai/xenomai/wikis/Setting_Up
  