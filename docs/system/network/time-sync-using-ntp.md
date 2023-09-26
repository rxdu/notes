# Time Synchronization using NTP and Chrony

## Local Time Sync

You can use chrony to get one computer to synchronize time with another computer in the same network (e.g. sync between an onboard computer and the host computer).

1. On computer 1 (which acts as time source), edit "/etc/chrony/chrony.conf" and add the network id

```
allow 192.168.1.0/24
```

2. On the client computer, edit "/etc/chrony/chrony.conf", add the ip of the reference computer

```
server 192.168.1.XX iburst
```

3. Make sure you restart the chronyd service after the changes on both of the computers

```
$ sudo systemctl restart chronyd
```

## Command Reference

* Chrony service management

```
$ sudo apt-get install chrony
$ sudo systemctl status chronyd
$ sudo systemctl restart chronyd
```

* Check the status of time tracking 

```
$ chronyc tracking
```

Example output:

```
$ chronyc tracking
Reference ID    : AC682C78 (ntp-singapore.gombadi.com)
Stratum         : 3
Ref time (UTC)  : Tue Sep 26 07:26:09 2023
System time     : 0.000219948 seconds slow of NTP time
Last offset     : -0.000393732 seconds
RMS offset      : 0.000415190 seconds
Frequency       : 14.277 ppm slow
Residual freq   : -0.011 ppm
Skew            : 0.149 ppm
Root delay      : 0.041356131 seconds
Root dispersion : 0.003122247 seconds
Update interval : 1024.8 seconds
Leap status     : Normal
```

* Check source of time references
  
```
$ chronyc sources
```

Example output:

```
$ chronyc sources
MS Name/IP address         Stratum Poll Reach LastRx Last sample               
===============================================================================
^- prod-ntp-5.ntp4.ps5.cano>     2  10   377   753  -1024us[-1349us] +/-   85ms
^- prod-ntp-3.ntp4.ps5.cano>     2  10   377  1052  -3399us[-3719us] +/-   82ms
^- prod-ntp-4.ntp1.ps5.cano>     2  10   377   51m   -435us[  -88us] +/-   80ms
^- alphyn.canonical.com          2  10   377   24m    +11ms[  +11ms] +/-  157ms
^+ time.cloudflare.com           3  10   377   987  -2820us[-3141us] +/-   47ms
^+ time.cloudflare.com           3  10   377   218  -3917us[-3917us] +/-   48ms
^* ntp-singapore.gombadi.com     2  10   377   675  +1893us[+1566us] +/-   22ms
^- sg.time.clearnet.pw           2  10   377   819  -9809us[  -10ms] +/-  118ms
```

## Reference

* https://chrony-project.org/
* https://robofoundry.medium.com/how-to-sync-time-between-robot-and-host-machine-for-ros2-ecbcff8aadc4