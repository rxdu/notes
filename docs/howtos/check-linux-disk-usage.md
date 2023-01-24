---
title: Check Linux Disk Usage
description: Find files that are taking up the disk space
---

You may have encountered the situation that your Linux system is running out of space. This happens more often on embedded devices running on a small eMMC or SD card. To cleanup the system, you first need to find out what is taking up the space. 

* Check partitions

```bash
$ df -h
```

With this command you will know which partition is full.

* Check file/folder within a partition

```bash
$ sudo apt-get install ncdu
$ sudo ncdu -x <the-folder-you-want-to-check>
```

With this command you will know which file/folder is taking up the space.

## Reference

- [1] https://askubuntu.com/questions/266825/what-do-i-do-when-my-root-filesystem-is-full