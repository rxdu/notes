# Build IMU IIO driver for Pocketbeagle 

The workspace is assumed to be at "~/beaglebone" and everything will be inside this folder by default in the following steps.

## Install tools

Install tools:

```bash
$ sudo apt install git bc bison flex libssl-dev make lzop
```

Install the same version of gcc compiler with the one in your debian image on Beaglebone. 

## Fetch Linux kernel source

* Beaglebone Linux Kernel ()

```bash
$ git clone -b 4.19-rt https://github.com/beagleboard/linux.git
```

The folder "linux" will be refered as the "target directory" in the following context.

* ST Linux Kernel with IIO Drivers

```bash
$ git clone -b linux-4.19.y-gh https://github.com/STMicroelectronics/STMems_Linux_IIO_drivers.git
```

The folder "STMems_Linux_IIO_drivers" will be refered as the "driver source directory" in the following context.

**Note**: you will need to switch to a different branch accordingly if you want to use a different Linux kernel version.

## Integrate IIO drivers 

Follow the instructions in README.md at https://github.com/STMicroelectronics/STMems_Linux_IIO_drivers/tree/linux-4.19.y-gh 

Here is a brief summary of the steps:

1. Copy driver source code (STMems_Linux_IIO_drivers/drivers/iio/imu/<your-imu-ic-name\>) into the target directory (linux/drivers/iio/imu/<your-imu-ic-name\>)
2. Update Kconfig and makefile in the target directory to include the driver to be integrated
3. Update custom event and channel types in "include/uapi/linux/iio/types.h" and "include/uapi/linux/iio/types.h" in the target directory

Now you have the beaglebone kernel source "patched" with the ST IIO sensor driver. You can build and deploy the kernel as usual.

```bash
$ make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- bb.org_defconfig 
$ make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- menuconfig
```

You would need to update the following configurations. Additional options might be needed according to your application:

```
* General setup ---> 
    - Local version - append to kernel release : "-pb-imu-rt"
* Kernel Features  --->
    - Timer frequency : change to 1000Hz
* Device Drivers --->
    - Industrial I/O support --->
        - Mark your driver module as "M"
```

After configuration, you can build the kernel

```bash
$ make -j12 ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- deb-pkg
```

You will get 3 .deb files to be installed on the pocketbeagle, e.g.:

* linux-headers-4.19.94-pb-imu-rt+_4.19.94-pb-imu-rt+-5_armhf.deb
* linux-libc-dev_4.19.94-pb-imu-rt+-5_armhf.deb
* linux-image-4.19.94-pb-imu-rt+_4.19.94-pb-imu-rt+-5_armhf.deb

Copy and install on pocketbeagle with "dpkg" command.

## Update device tree overlay

* Build the device tree overlay

Now you've got the drivers compiled with the kernel and you will need to add a device tree overlay to load the driver.

```bash
$ git clone https://github.com/beagleboard/bb.org-overlays.git
```

It's easiest to use overlay files inside this repository as the base and modify from one that is cloest to your setup. For example, my LSM6DSM is connected to the SPI0 on pocketbeagle and I use "bb.org-overlays/src/arm/PB-SPI0-ETH-WIZ-CLICK.dts" as a reference to make my "bb.org-overlays/src/arm/PB-SPI0-LSM6DSMTR.dts"

```
/*
 * Copyright (C) 2017 Robert Nelson <robertcnelson@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

/dts-v1/;
/plugin/;

#include <dt-bindings/gpio/gpio.h>
#include <dt-bindings/pinctrl/am33xx.h>
#include <dt-bindings/interrupt-controller/irq.h>

/ {
	/*
	 * Helper to show loaded overlays under: /proc/device-tree/chosen/overlays/
	 */
	fragment@0 {
		target-path="/";
		__overlay__ {

			chosen {
				overlays {
					PB-SPI0-LSM6DSMTR = __TIMESTAMP__;
				};
			};
		};
	};

	/*
	 * Free up the pins used by the cape from the pinmux helpers.
	 */
	fragment@1 {
		target = <&ocp>;
		__overlay__ {
			P1_04_pinmux { status = "disabled"; }; /* LSM6DSMTR INT1 */
			P1_06_pinmux { status = "disabled"; }; /* SPI0 CS0 */
			P1_08_pinmux { status = "disabled"; }; /* SPI0 CLK */
			P1_10_pinmux { status = "disabled"; }; /* SPI0 MISO */
			P1_12_pinmux { status = "disabled"; }; /* SPI0 MOSI */
			P2_22_pinmux { status = "disabled"; }; /* LSM6DSMTR INT2 (not used) */
		};
	};

	fragment@2 {
		target = <&am33xx_pinmux>;
		__overlay__ {
			lsm6dsmtr_pins: pinmux_lsm6dsmtr_pins {
				pinctrl-single,pins = <
					AM33XX_IOPAD(0x0838, PIN_INPUT | MUX_MODE7 ) /* (T6) P2_22, gpmc_ad14.gpio1[14] INT2 */
					AM33XX_IOPAD(0x08ec, PIN_INPUT | MUX_MODE7 ) /* (R6) P1_04, gpmc_a11.gpio2[25] INT1 */
				>;
			};

			pb_spi0_pins: pinmux_pb_spi0_pins {
				pinctrl-single,pins = <
					AM33XX_IOPAD(0x0950, PIN_INPUT | MUX_MODE0 ) /* (A17) spi0_sclk.spi0_sclk */
					AM33XX_IOPAD(0x0954, PIN_INPUT | MUX_MODE0 ) /* (B17) spi0_d0.spi0_d0 */
					AM33XX_IOPAD(0x0958, PIN_INPUT | MUX_MODE0 ) /* (B16) spi0_d1.spi0_d1 */
					AM33XX_IOPAD(0x095c, PIN_INPUT | MUX_MODE0 ) /* (A16) spi0_cs0.spi0_cs0 */
				>;
			};
		};
	};

	fragment@3 {
		target = <&spi0>;
		__overlay__ {
			status = "okay";
			pinctrl-names = "default";
			pinctrl-0 = <&pb_spi0_pins>;

			channel@0{ status = "disabled"; };
			channel@1{ status = "disabled"; };
		};
	};

	fragment@4 {
		target = <&spi0>;
		__overlay__ {
			#address-cells = <1>;
			#size-cells = <0>;

			lsm6dsmtr: lsm6dsm@0 {
                spi-max-frequency = <5000000>;
				compatible = "st,lsm6dsm";
				reg = <0x0>;
                interrupt-parent = <&gpio2>;
				interrupts = <25 IRQ_TYPE_LEVEL_HIGH>;
			};
		};
	};
};
```

If everything in the device tree is set up correctly, you will get "bb.org-overlays/src/arm/PB-SPI0-LSM6DSMTR.dtbo" after executing "make". Copy this dtbo file to "/lib/firmware" folder on the pocketbeagle.

* Use the compiled device tree overlay file

Open "/boot/uEnv.txt" with a text editor on your pocketbeagle, add the following line at the "Additional custom capes" section:

```
###Additional custom capes
uboot_overlay_addr4=/lib/firmware/PB-SPI0-LSM6DSMTR-INT1.dtbo
```

Now you can reboot and the driver should be loaded correctly. You can check the IMU devices at "/sys/bus/iio/devices" and you should see devices like "iio:device0", "iio:device1" etc. In my case, "iio:device1" cooresponds to the accelerometer and you can see attributes like:

* current_timestamp_clock
* in_accel_x_raw
* in_accel_x_scale
* in_accel_y_raw
* in_accel_y_scale
* in_accel_z_raw
* in_accel_z_scale

You should be able to read values from the attributes, for example: 

```bash
debian@beaglebone:/sys/bus/iio/devices/iio:device1$ cat in_accel_z_raw
17111
```

## Reference

* [1] https://wiki.analog.com/resources/tools-software/linux-drivers/iio-inertial-measurement-units/adis16475#trigger_management
* [2] https://events19.linuxfoundation.org/wp-content/uploads/2017/12/Bandan-Das_Drone_SITL_bringup_with_the_IIO_framework.pdf
* [3] https://github.com/STMicroelectronics/st-mems-android-linux-drivers-iio
* [4] https://www.kernel.org/doc/html/v5.10/driver-api/iio/index.html