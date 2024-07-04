# mDNS

## Install mDNS on Ubuntu

To get mDNS support, you need to install the following packages:

```bash
$ sudo apt-get install avahi-daemon libnss-mdns
```

Edit the hosts line in: /etc/nsswitch.conf as follows:

```
hosts:          files mdns4_minimal [NOTFOUND=return] dns 
```

This tells the computer to look first at the hosts file, then at mDNS.

With the above setup. your computer should be reachable at `hostname.local`, for example, `rdu-rpi4.local`.

## Change the Host Name

If you would like to have a different name, you can modify the configuration file `/etc/avahi/avahi-daemon.conf`. Look for the lines

```bash
#host-name=foo
#domain-name=local
```

Uncomment the lines and update the names accordingly.

## Reference

* https://support.bostondynamics.com/s/article/Configuring-multicast-DNS
* https://wiki.archlinux.org/title/avahi