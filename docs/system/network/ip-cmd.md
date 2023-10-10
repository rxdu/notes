# IP Command

On newer Ubuntu releases, you may not have "ifconfig" command by default. Instead, you have "ip" command out-of-box. "ip" is starting to replace "ifconfig" in newer Linux distributions.

You can still install ifconfig if it's not present in the system:

```bash
$ sudo apt-get install net-tools
```

Similarly, you can manually install the ip tool:

```bash
$ sudo apt-get install iproute2
```

* Display Current Network Settings

```bash
$ ifconfig
```

```bash
$ ip a
```

* Enable and Disable an Interface

```bash
$ sudo ifconfig eth0 up
$ sudo ifconfig eth0 down
```

```bash
$ sudo ip link set eth0 up
$ sudo ip link set eth0 down
```

* Assign a IP/Netmask to an Interface

```bash
$ sudo ifconfig eth0 192.168.0.2
$ sudo ifconfig eth0 netmask 255.255.255.0
$ sudo ifconfig eth0 del 192.168.1.10
$ sudo ifconfig eth0 mtu 1080
```

```bash
$ sudo ip addr add 192.168.0.2/24 dev eth0
$ sudo ip addr del 192.168.0.2/24 dev eth0
$ sudo ip link set dev eth0 mtu 1500
```

* Show Routing Table

```bash
$ route -n
$ sudo route add default gw 192.168.1.1
$ sudo route add -net 10.5.5.10 netmask 255.255.255.0 gw 192.168.0.1
```

```bash
$ ip route show
$ sudo ip route add default via 192.168.1.1
$ sudo ip route add 10.5.5.10/24 via 192.168.0.1 dev eth0
$ sudo ip route del 10.5.5.10/24
$ sudo ip route del default via 62.12.113.1 dev eth1
```
