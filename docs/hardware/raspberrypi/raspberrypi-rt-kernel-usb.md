# Fix USB Issue in Kernel 4.19.y-rt for Raspberry Pi

After building and replacing the Linux kernel 4.19.y-rt on Raspberry Pi 4B (8G), all the USB ports stopped working. The following shows the fix to this issue.

## Update Device Tree

The device tree file "/arch/arm/boot/dts/bcm2711-rpi-4-b.dts" doesn't include the USB related snippets.

```git
--- a/arch/arm/boot/dts/bcm2711-rpi-4-b.dts     2020-12-18 13:20:23.973176269 +0000
+++ b/arch/arm/boot/dts/bcm2711-rpi-4-b.dts     2020-12-18 13:20:51.072542212 +0000
@@ -2,6 +2,7 @@
 /dts-v1/;
 #include "bcm2711.dtsi"
 #include "bcm2835-rpi.dtsi"
+#include "bcm283x-rpi-usb-peripheral.dtsi"
 
 #include <dt-bindings/reset/raspberrypi,firmware-reset.h>
 
--- /dev/null   2020-12-15 00:20:37.047999998 +0000
+++ a/arch/arm/boot/dts/bcm283x-rpi-usb-peripheral.dtsi 2020-12-18 13:25:45.090219029 +0000
@@ -0,0 +1,7 @@
+// SPDX-License-Identifier: GPL-2.0
+&usb {
+       dr_mode = "peripheral";
+       g-rx-fifo-size = <256>;
+       g-np-tx-fifo-size = <32>;
+       g-tx-fifo-size = <256 256 512 512 512 768 768>;
+};
```

There are two changes. The first one simply add a line to include "bcm283x-rpi-usb-peripheral.dtsi" in the "/arch/arm/boot/dts/bcm2711-rpi-4-b.dts". The second one creates the included snippets which was missing in the Linux source tree.

## Change the Firmware

According to the discussion in [2], there seems to be some issues in the default firmware. As suggested, copy the firmware binaries from the Ubuntu distribution. Download the firmware package from [here](https://launchpad.net/ubuntu/+source/linux-firmware-raspi2/1.20200212-0ubuntu1): linux-firmware-raspi2_1.20200212.orig.tar.gz.

Extract the files and copy to "/boot" on the Raspberry Pi. You can backup the existing "*.elf" and "*.dat" in case you need to recover late.

After rebooting, the USB ports should work. But do note that it's unknown wether there are any other possible issues not covered by this fix and further investigations are required.

## Reference

* [1] https://github.com/raspberrypi/linux/issues/3976
* [2] https://archlinuxarm.org/forum/viewtopic.php?f=65&t=14734&p=65334#p65334