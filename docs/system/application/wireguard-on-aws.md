---
title: Wireguard Setup on AWS
description: Instructions on setting up Wireguard on AWS
---

## Tested Environment

- Debian 10.5 on AWS
- Ubuntu 20.04 on AWS

## Install wireguard on server

```bash
$ sudo apt-get install wireguard-dkms wireguard-tools linux-headers-$(uname -r)
```

Note that the package "linux-headers-$(uname -r)" is necessary to run wireguard.

Next we need to enable IP Forwarding. Open "/etc/sysctl.conf" and search for the line "#net.ipv4.ip_forward". Uncomment this line by removing the # at the beginning. It should look like this: 

```
net.ipv4.ip_forward=1
```

## Generate server and client keys

You will need root privilege to manipuate the "/etc/wireguard" directory

```bash
$ sudo -i 
$ cd /etc/wireguard
$ umask 077
$ wg genkey | tee server_private_key | wg pubkey > server_public_key
```

You will need to create a key pair for each client  (more deails below, not needed for now)

```bash
$ wg genkey | tee <client_private_key_name> | wg pubkey > <client_public_key_name>
```

## Generate server and client keys

You will need root privilege to manipuate the "/etc/wireguard" directory

```bash
$ sudo -i 
$ cd /etc/wireguard
$ umask 077
$ wg genkey | tee server_private_key | wg pubkey > server_public_key
```

You will need to create a key pair for each client  (more deails below, not needed for now)

```bash
$ wg genkey | tee <client_private_key_name> | wg pubkey > <client_public_key_name>
```

## Create server configuration file

Create a file "**/etc/wireguard/wg0.conf"** with the following content

```bash
[Interface]
Address = 10.200.200.1/24
SaveConfig = true
PostUp = /etc/wireguard/iptable-set.sh
PostDown = /etc/wireguard/iptable-reset.sh
ListenPort = 51820
PrivateKey = <insert server_private_key>

[Peer]
PublicKey = <insert client_public_key>
AllowedIPs = 10.200.200.2/32

```

Note in the above example,  

* "10.200.200.0" range is used for the VPN network. You can change it to other preferred ranges
* The ListenPort has to be accesible and you will need to add a rule in your Firewall to allow "UDP:<ListenPort\>"
* Two scripts are used to set up or recover the routing table when you bring up or down the VPN network

The setup script: 

```bash
#!/bin/bash

# Reference:
#  [1] https://www.ckn.io/blog/2017/11/14/wireguard-vpn-typical-setup/
#  [2] https://www.cyberciti.biz/faq/how-to-set-up-wireguard-firewall-rules-in-linux/

IN_FACE="eth0"                   # NIC connected to the internet
WG_FACE="wg0"                    # WG NIC 
SUB_NET="10.200.200.0/24"            # WG IPv4 sub/net aka CIDR
WG_PORT="51820"                  # WG udp port
 
# track VPN connection
/sbin/iptables -A INPUT -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT
/sbin/iptables -A FORWARD -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT

# allow incoming VPN traffic on the listening port
/sbin/iptables -A INPUT -p udp -m udp --dport $WG_PORT -m conntrack --ctstate NEW -j ACCEPT
# allow both TCP and UDP recursive DNS traffic
/sbin/iptables -A INPUT -s $SUB_NET -p tcp -m tcp --dport 53 -m conntrack --ctstate NEW -j ACCEPT
/sbin/iptables -A INPUT -s $SUB_NET -p udp -m udp --dport 53 -m conntrack --ctstate NEW -j ACCEPT
# allow forwarding of packets between interfaces
/sbin/iptables -A FORWARD -i $WG_FACE -o $WG_FACE -m conntrack --ctstate NEW -j ACCEPT
/sbin/iptables -A FORWARD -i $IN_FACE -o $WG_FACE -m conntrack --ctstate NEW -j ACCEPT
/sbin/iptables -A FORWARD -i $WG_FACE -o $IN_FACE -m conntrack --ctstate NEW -j ACCEPT
# set up nat
/sbin/iptables -t nat -A POSTROUTING -s $SUB_NET -o $IN_FACE -j MASQUERADE

```

The recovery script:

```bash
#!/bin/bash
 
IN_FACE="eth0"                   # NIC connected to the internet
WG_FACE="wg0"                    # WG NIC 
SUB_NET="10.200.200.0/24"            # WG IPv4 sub/net aka CIDR
WG_PORT="51820"                  # WG udp port

# allow incoming VPN traffic on the listening port
/sbin/iptables -D INPUT -p udp -m udp --dport $WG_PORT -m conntrack --ctstate NEW -j ACCEPT
# allow both TCP and UDP recursive DNS traffic
/sbin/iptables -D INPUT -s $SUB_NET -p tcp -m tcp --dport 53 -m conntrack --ctstate NEW -j ACCEPT
/sbin/iptables -D INPUT -s $SUB_NET -p udp -m udp --dport 53 -m conntrack --ctstate NEW -j ACCEPT
# allow forwarding of packets between interfaces
/sbin/iptables -D FORWARD -i $WG_FACE -o $WG_FACE -m conntrack --ctstate NEW -j ACCEPT
/sbin/iptables -D FORWARD -i $IN_FACE -o $WG_FACE -m conntrack --ctstate NEW -j ACCEPT
/sbin/iptables -D FORWARD -i $WG_FACE -o $IN_FACE -m conntrack --ctstate NEW -j ACCEPT
# set up nat
/sbin/iptables -t nat -D POSTROUTING -s $SUB_NET -o $IN_FACE -j MASQUERADE

```


## Configure DNS

Here "ubound" is used to provide DNS 

```bash
$ apt-get install unbound unbound-host dnsutils
```

Download the list of root DNS servers

```bash
$ curl -o /var/lib/unbound/root.hints https://www.internic.net/domain/named.cache
$ chown -R unbound:unbound /var/lib/unbound
$ cd /etc/unbound/unbound.conf.d
$ nano unbound_srv.conf
```

Add the following content

```text
server:

  num-threads: 4

  #Enable logs
  verbosity: 1

  #list of Root DNS Server
  root-hints: "/var/lib/unbound/root.hints"

  #Use the root servers key for DNSSEC
  #auto-trust-anchor-file: "/var/lib/unbound/root.key"

  #Respond to DNS requests on all interfaces
  interface: 0.0.0.0
  max-udp-size: 3072

  #Authorized IPs to access the DNS Server
  access-control: 0.0.0.0/0                 refuse
  access-control: 127.0.0.1                 allow
  access-control: 10.200.200.0/24               allow

  #not allowed to be returned for public internet  names
  private-address: 10.200.200.0/24

  # Hide DNS Server info
  hide-identity: yes
  hide-version: yes

  #Limit DNS Fraud and use DNSSEC
  harden-glue: yes
  harden-dnssec-stripped: yes
  harden-referral-path: yes

  #Add an unwanted reply threshold to clean the cache and avoid when possible a DNS Poisoning
  unwanted-reply-threshold: 10000000

  #Have the validator print validation failures to the log.
  val-log-level: 1

  #Minimum lifetime of cache entries in seconds
  cache-min-ttl: 1800

  #Maximum lifetime of cached entries
  cache-max-ttl: 14400
  prefetch: yes
  prefetch-key: yes

```

Now restart unbound service and enable it to autostart

```bash
$ systemctl restart unbound
$ systemctl enable unbound
```

## Create server configuration file

Create a file "**/etc/wireguard/wg0.conf"** with the following content

```text
[Interface]
Address = 10.200.200.1/24
SaveConfig = true
PostUp = /etc/wireguard/iptable-set.sh
PostDown = /etc/wireguard/iptable-reset.sh
ListenPort = 51820
PrivateKey = <insert server_private_key\>

[Peer]
PublicKey = <insert client_public_key\>
AllowedIPs = 10.200.200.2/32

```

Note in the above example,  

- "10.200.200.0" range is used for the VPN network. You can change it to other preferred ranges

- The ListenPort has to be accesible and you will need to add a rule in your Firewall to allow "UDP:<ListenPort\>"

- Two scripts are used to set up or recover the routing table when you bring up or down the VPN network

The setup script: 

```bash
#!/bin/bash

# Reference:
#  [1] https://www.ckn.io/blog/2017/11/14/wireguard-vpn-typical-setup/
#  [2] https://www.cyberciti.biz/faq/how-to-set-up-wireguard-firewall-rules-in-linux/

IN_FACE="eth0"                   # NIC connected to the internet
WG_FACE="wg0"                    # WG NIC 
SUB_NET="10.200.200.0/24"            # WG IPv4 sub/net aka CIDR
WG_PORT="51820"                  # WG udp port
 
# track VPN connection
/sbin/iptables -A INPUT -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT
/sbin/iptables -A FORWARD -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT

# allow incoming VPN traffic on the listening port
/sbin/iptables -A INPUT -p udp -m udp --dport $WG_PORT -m conntrack --ctstate NEW -j ACCEPT
# allow both TCP and UDP recursive DNS traffic
/sbin/iptables -A INPUT -s $SUB_NET -p tcp -m tcp --dport 53 -m conntrack --ctstate NEW -j ACCEPT
/sbin/iptables -A INPUT -s $SUB_NET -p udp -m udp --dport 53 -m conntrack --ctstate NEW -j ACCEPT
# allow forwarding of packets between interfaces
/sbin/iptables -A FORWARD -i $WG_FACE -o $WG_FACE -m conntrack --ctstate NEW -j ACCEPT
/sbin/iptables -A FORWARD -i $IN_FACE -o $WG_FACE -m conntrack --ctstate NEW -j ACCEPT
/sbin/iptables -A FORWARD -i $WG_FACE -o $IN_FACE -m conntrack --ctstate NEW -j ACCEPT
# set up nat
/sbin/iptables -t nat -A POSTROUTING -s $SUB_NET -o $IN_FACE -j MASQUERADE

```

The recovery script:

```bash
#!/bin/bash
 
IN_FACE="eth0"                   # NIC connected to the internet
WG_FACE="wg0"                    # WG NIC 
SUB_NET="10.200.200.0/24"            # WG IPv4 sub/net aka CIDR
WG_PORT="51820"                  # WG udp port

# allow incoming VPN traffic on the listening port
/sbin/iptables -D INPUT -p udp -m udp --dport $WG_PORT -m conntrack --ctstate NEW -j ACCEPT
# allow both TCP and UDP recursive DNS traffic
/sbin/iptables -D INPUT -s $SUB_NET -p tcp -m tcp --dport 53 -m conntrack --ctstate NEW -j ACCEPT
/sbin/iptables -D INPUT -s $SUB_NET -p udp -m udp --dport 53 -m conntrack --ctstate NEW -j ACCEPT
# allow forwarding of packets between interfaces
/sbin/iptables -D FORWARD -i $WG_FACE -o $WG_FACE -m conntrack --ctstate NEW -j ACCEPT
/sbin/iptables -D FORWARD -i $IN_FACE -o $WG_FACE -m conntrack --ctstate NEW -j ACCEPT
/sbin/iptables -D FORWARD -i $WG_FACE -o $IN_FACE -m conntrack --ctstate NEW -j ACCEPT
# set up nat
/sbin/iptables -t nat -D POSTROUTING -s $SUB_NET -o $IN_FACE -j MASQUERADE

```

## Configure DNS

Here "ubound" is used to provide DNS 

```bash
$ apt-get install unbound unbound-host dnsutils
```

Download the list of root DNS servers

```bash
$ curl -o /var/lib/unbound/root.hints https://www.internic.net/domain/named.cache
$ chown -R unbound:unbound /var/lib/unbound
$ cd /etc/unbound/unbound.conf.d
$ nano unbound_srv.conf
```

Add the following content

```text
server:

  num-threads: 4

  #Enable logs
  verbosity: 1

  #list of Root DNS Server
  root-hints: "/var/lib/unbound/root.hints"

  #Use the root servers key for DNSSEC
  #auto-trust-anchor-file: "/var/lib/unbound/root.key"

  #Respond to DNS requests on all interfaces
  interface: 0.0.0.0
  max-udp-size: 3072

  #Authorized IPs to access the DNS Server
  access-control: 0.0.0.0/0                 refuse
  access-control: 127.0.0.1                 allow
  access-control: 10.200.200.0/24               allow

  #not allowed to be returned for public internet  names
  private-address: 10.200.200.0/24

  # Hide DNS Server info
  hide-identity: yes
  hide-version: yes

  #Limit DNS Fraud and use DNSSEC
  harden-glue: yes
  harden-dnssec-stripped: yes
  harden-referral-path: yes

  #Add an unwanted reply threshold to clean the cache and avoid when possible a DNS Poisoning
  unwanted-reply-threshold: 10000000

  #Have the validator print validation failures to the log.
  val-log-level: 1

  #Minimum lifetime of cache entries in seconds
  cache-min-ttl: 1800

  #Maximum lifetime of cached entries
  cache-max-ttl: 14400
  prefetch: yes
  prefetch-key: yes

```

Now restart unbound service and enable it to autostart

```bash
$ systemctl restart unbound
$ systemctl enable unbound
```

You may need to disable the default DNS resolver if unbound fails to start with an error message saying port 53 has been binded to another process [8][9][10]

```bash
% use netstat to check whether port 53 has been binded 
$ netstat -lutnp

% disable systemd-resolved
$ sudo systemctl stop systemd-resolved
$ sudo systemctl disable systemd-resolved
```

You can test your DNS setup with the following commands and you should expect to see similar results returned

```bash
$ nslookup www.google.com. 10.200.200.1
Server:		10.8.6.1
Address:	10.8.6.1#53

Non-authoritative answer:
Name:	www.google.com
Address: 74.125.200.99
Name:	www.google.com
Address: 74.125.200.106
Name:	www.google.com
Address: 74.125.200.103
Name:	www.google.com
Address: 74.125.200.105
Name:	www.google.com
Address: 74.125.200.104
Name:	www.google.com
Address: 74.125.200.147
Name:	www.google.com
Address: 2404:6800:4003:c00::68
Name:	www.google.com
Address: 2404:6800:4003:c00::67
Name:	www.google.com
Address: 2404:6800:4003:c00::63
Name:	www.google.com
Address: 2404:6800:4003:c00::93

$ unbound-host -C /etc/unbound/unbound.conf -v ietf.org
[1599212188] libunbound[2097:0] notice: init module 0: validator
[1599212188] libunbound[2097:0] notice: init module 1: iterator
ietf.org has address 4.31.198.44 (secure)
ietf.org has IPv6 address 2001:1900:3001:11::2c (secure)
ietf.org mail is handled by 0 mail.ietf.org. (secure)

```

Additionally you can use [http://dnsleak.com/](http://dnsleak.com/) to test DNS leakage.

## Start wireguard service on server

Now you're ready to start the wireguard service on your server

```bash
$ sudo wg-quick up wg0
$ sudo systemctl enable wg-quick@wg0.service 
```

Use the following command to bring down the service

```text
$ sudo wg-quick down wg0
```

### Setup clients

## Server side

As mentioned above, you will have create a key pair for each client, for example:

```text
$ wg genkey | tee rdu_acer_private.key | wg pubkey > rdu_acer_public.key
```

Register the client on the server

```bash
$ wg set wg0 peer <new_client_public_key> allowed-ips <new_client_vpn_IP>/32
```

In the following client setup part, we will assume you've assigned 10.200.200.2/32 to the client rdu-acer. 

**You need to restart the "wg0" interface to make this change into effect.**

## Client side

You can either set up the VPN profile on your device manually or from a QR code

**Manual setup on Linux**

```bash
$  sudo apt-get install wireguard-dkms wireguard-tools linux-headers-$(uname -r)
```

Create the configuration file

```bash
$ sudo -i
$ nano /etc/wireguard/wg0-acer.conf
```

```text
[Interface]
Address = 10.200.200.2/32
PrivateKey = <insert client_private_key>
DNS = 10.200.200.1

[Peer]
PublicKey = <insert server_public_key>
Endpoint = <VPN-Server-Public-IP>:<ListenPort>
AllowedIPs = 0.0.0.0/0
PersistentKeepalive = 25
```

Now you can bring up the connection

```bash
$ sudo wg-quick up wg0-acer
$ sudo systemctl enable wg-quick@wg0-acer.service
```

**QR code setup on Android**

Instead of manually typing in the VPN information on the phone, you can generate a QR code on the server and setup your phone with this QR code.

```bash
$ cd /etc/wireguard/
$ nano rdu_acer_qr.conf
```

```text
[Interface]
Address = 10.200.200.2/32
PrivateKey = client_private_key
DNS = 10.200.200.1

[Peer]
PublicKey = server_public_key
AllowedIPs = 0.0.0.0/0
Endpoint = <VPN-Server-Public-IP>:<ListenPort>
PersistentKeepalive = 25
```

Then generate the QR code:

```bash
$ apt install qrencode
$ qrencode -t ansiutf8 < rdu_acer_qr.conf
```

Now you can scan the QR code on your phone to setup the VPN connection.

## Reference

* [1] https://golb.hplar.ch/2018/10/wireguard-on-amazon-lightsail.html
* [2] https://www.ckn.io/blog/2017/11/14/wireguard-vpn-typical-setup/
* [3] https://www.cyberciti.biz/faq/how-to-set-up-wireguard-firewall-rules-in-linux/
* [4] https://stackoverflow.com/questions/37570910/rtnetlink-answers-operation-not-supported
* [5] https://emanuelduss.ch/2018/09/wireguard-vpn-road-warrior-setup/
* [6] https://engineerworkshop.com/blog/how-to-set-up-a-wireguard-vpn-server-on-ubuntu-linux/
* [7] https://www.zahradnik.io/wireguard-a-vpn-with-real-world-usage-in-mind
* [8] https://askubuntu.com/questions/907246/how-to-disable-systemd-resolved-in-ubuntu
* [9] https://blobfolio.com/2017/05/fix-linux-dns-issues-caused-by-systemd-resolved/
* [10] https://askubuntu.com/questions/59458/error-message-sudo-unable-to-resolve-host-none
* [11] https://www.linuxbabe.com/ubuntu/set-up-local-dns-resolver-ubuntu-20-04-bind9
* [12] https://nlnetlabs.nl/documentation/unbound/howto-anchor/