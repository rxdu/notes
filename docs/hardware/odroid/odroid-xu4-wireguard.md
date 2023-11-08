# Install Wireguard on Odroid XU4

In order to install the wireguard kernel module properly, you need to make sure the linux headers are install properly first. You can find the headers package that matches your kernel with the commands:

```bash
# find kernel version
$ uname -r
# search for linux-headers package
$ apt search linux-headers
```

For the case that the Odroid XU4 runs Armbian Ubuntu Jammy with kernel "5.4.253-current-odroidxu4", you can install

```bash
$ sudo apt install linux-headers-current-odroidxu4
```

Then you can try to install wireguard

```bash
$ sudo apt install wireguard wireguard-dkms
```

If the installation is successful, you should be able to bring up the interface. 

In the case that you encounter the following error during bring up

```
Warning: /etc/resolv.conf is not a symbolic link to /run/resolvconf/resolv.conf
```

You can try to resolve the issue by 

```bash
$ sudo dpkg-reconfigure resolvconf
```

## Reference

* [1] https://forum.armbian.com/topic/9282-installation-of-wireguard/
* [2] https://github.com/pop-os/pop/issues/773