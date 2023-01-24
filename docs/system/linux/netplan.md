# Netplan Reference

## Network Configuration Command: "ip"

On Ubuntu 22.04 server, you don't get "ifconfig" command by default (it's the case at least on the version for Raspberry Pi). Instead, you have "ip" command out-of-box. "ip" is starting to replace "ifconfig" in newer Linux distributions.

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

## Netplan Reference Configuration

Since Ubuntu 20.04, "netplan" is used to manage network interfaces, replacing the old "/etc/network/interfaces" configuration file. You can find configurations for netplan at "/etc/netplan". After modifying the "*.yaml" file, you can use the following command to apply the changes:

```bash
$ sudo netplan apply
```

Here are some snippets of the most commonly used netplan configurations from [1]. You can find more details in the [official documentation](https://netplan.io/reference).


### Create a Loopback Interface

```yaml
network:
    version: 2
    renderer: networkd
    ethernets:
        lo:
            addresses: [ "127.0.0.1/8", "::1/128", "7.7.7.7/32" ]
```

### Connect to Network with DHCP

```yaml
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: true
  wifis:
    wlan0:
      dhcp4: true
```

### Connect to Ethernet with Static IP

```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 10.10.10.2/24
      nameservers:
        search: [mydomain, otherdomain]
        addresses: [10.10.10.1, 1.1.1.1]
      routes:
        - to: default
          via: 10.10.10.1
```

### Connect to Wireless Network with Static IP

```yaml
network:
  version: 2
  renderer: networkd
  wifis:
    wlan0:
      dhcp4: no
      dhcp6: no
      addresses: [192.168.0.21/24]
      nameservers:
        addresses: [192.168.0.1, 8.8.8.8]
      access-points:
        "network_ssid_name":
            password: "**********"
      routes:
        - to: default
          via: 192.168.0.1
```

### Configure a Network Bridge

```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
      enp3s0:
          dhcp4: no
  bridges:
      br0:
          dhcp4: yes
          interfaces:
              - enp3s0
```

## Reference

* [1] https://netplan.io/examples
* [2] https://netplan.io/reference