# Network Inspection

## Port

Both `netstat` and `ss` command can be used for port usage inspection. `ss` is regarded as a newer and faster alternative to the older `netstat`. 

You can find port information with the following arguments:

```bash
$ sudo netstat -tulpn
$ sudo ss -tulpn
```

"tulpn" is a combination of flags with the following meanings:

    -t : display only TCP sockets
    -u : display only UDP sockets
    -l : display listening sockets
    -p : show process using socket
    -n : don't resolve service names (e.g. using DNS)

If you want to see all ports instead of only listening ports, you can replace "-l" with "-a".

More filtering can be done with the result returned by the above command. 

For example, to inspect usage of port 22:

```bash
$ sudo ss -tulpn | grep :22
```

You should see something similiar to the following if you have sshd installed:

```bash
tcp   LISTEN 0      128               0.0.0.0:22         0.0.0.0:*    users:(("sshd",pid=1638,fd=3))            
tcp   LISTEN 0      128                  [::]:22            [::]:*    users:(("sshd",pid=1638,fd=4))
```

## DNS

Perform a DNS lookup

```bash
$ nslookup google.com    
```

## Routing

Track the route that packets take to reach a destination on a TCP/IP network

```bash
$ sudo traceroute -T www.google.com
```

## Bandwidth

Check bandwidth between two hosts

```bash
# on the first computer
$ iperf3 -s

# on the second computer
$ iperf3 -c <ip-of-1st-host>
```

## Traffic

Monitor network traffic on the host:

```bash
$ sudo iftop
```

## Reference

* https://www.cyberciti.biz/faq/how-do-i-check-if-a-port-is-in-use-on-linux/