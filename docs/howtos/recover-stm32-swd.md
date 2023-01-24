---
title: Recover STM32 SWD
description: How to recover SWD pins using JLink
---

It's possible to disable the SWD pins by accident and the SWD interface will become unavailable after a successful flashing. This happened to me a few times when I used STM32CubeMX to generate driver code and forgot to enable the "Serial Wire" debug interface in the "System Core/SYS" section. The following solution is provided by Andre Belanger for solving the Apollo 2 EVB and it should also work for other similar MCUs. (Tested with STM32F405RG)

The following is a summary of the solution (modified from Andre's answer):

1. Power-down the board completely to get all registers reset
2. Hold the nRST pin low to prevent code from executing
3. Power-up the board while continuing to hold nRST low
4. Use Segger J-link commander console (JLinkExe) to connect to the MCU (should connect even though nRST is asserted, but cannot program/erase while nRST is asserted)
5. Type a long list of “halt” commands into a text editor, copy them into clipboard, then paste into the J-Link command window.  The halt commands that try to execute while nRST is held low will fail, and will execute quickly, so you need to paste a long list of halt commands (~ 100)
6. Immediately release nRST while the halt commands are being processed (note that even 100 halt commands will execute in ~ 1 second so small but manageable window in which to release nRST)
7. Debugger should reports that halt was successful, at which point the "erase" command can be used to remove the problem firmware
8. If the halt commands fail, then the full procedure should be repeated starting from step #1 (simply asserting reset will not be sufficient for many issues, need to start from power-down).

Sample halt commands, make sure you have enough of them to get you enough time

```
halt
halt
halt
halt
halt
halt
halt
halt
halt
halt
halt
halt
```

## Reference

- [1] https://support.ambiqmicro.com/hc/en-us/community/posts/360030663231-What-to-do-if-firmware-disables-SWD-pins-Help-