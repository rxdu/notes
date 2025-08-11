# Cyclone DDS Settings

## Cyclone DDS configuration

You can apply changes to the Cyclone DDS configuration by creating a file at `/etc/cyclonedds.xml` (or any other path you prefer).

```bash
sudo nano /etc/cyclonedds.xml
```

Then you can set the `CYCLONEDDS_URI` environment variable to the path of the configuration file.

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///etc/cyclonedds.xml/cyclonedds.xml
```

After the changes are applied, you need to restart the ROS daemon.

```bash
ros2 daemon stop
```

Alternatively, you can search the daemon process in `htop` and kill it.

```bash
$ htop

# press F3 to search, use keyword ros2cli.daemon
# trigger a kill signal with F9, then signal type 9
```

## Traffic on the loopback interface

According to this [issue](https://github.com/ros2/rmw_cyclonedds/issues/370), the `ROS_LOCALHOST_ONLY` environment variable is not supported in Cyclone DDS and it should not be set in your `~/.bashrc` file when you try to limit the traffic to the loopback interface.

You may achieve this by specifying the Cyclone DDS to only listen to traffic from `localhost`.

This can be done by specifying the `lo` interface in the `cyclonedds.xml` file.

```text
<Interfaces>
    <NetworkInterface name="lo" priority="default" multicast="default" />
</Interfaces>
```

You also need to enable multicast on the `lo` interface.

```bash
sudo ip link set lo multicast on
```

You can also persist the multicast setting on the `lo` interface by adding a systemd service.

```bash
sudo nano /etc/systemd/system/lo-multicast.service
```

```
[Unit]
Description=Enable Multicast on Loopback

[Service]
Type=oneshot
ExecStart=/usr/sbin/ip link set lo multicast on

[Install]
WantedBy=multi-user.target
```

## System-wide network settings

You need to increase the maximum receive buffer size for network packets and set the IP fragmentation settings before you can apply the suggested Cyclone DDS configuration.

```bash
sudo nano /etc/sysctl.d/10-cyclone-max.conf
```

Add the following content to the file.

```bash
# Increase the maximum receive buffer size for network packets
net.core.rmem_max=2147483647  # 2 GiB, default is 208 KiB

# IP fragmentation settings
net.ipv4.ipfrag_time=3  # in seconds, default is 30 s
net.ipv4.ipfrag_high_thresh=134217728  # 128 MiB, default is 256 KiB
```

Then you can apply the changes with the following command.

```bash
sudo sysctl -p --system
```

If you only want to apply the changes to the current session, you can use the following command

```bash
# Increase the maximum receive buffer size for network packets
sudo sysctl -w net.core.rmem_max=2147483647  # 2 GiB, default is 208 KiB

# IP fragmentation settings
sudo sysctl -w net.ipv4.ipfrag_time=3  # in seconds, default is 30 s
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728  # 128 MiB, default is 256 KiB
```

You may validate the changes with the following command.

```bash
user@pc$ sysctl net.core.rmem_max net.ipv4.ipfrag_time net.ipv4.ipfrag_high_thresh
net.core.rmem_max = 2147483647
net.ipv4.ipfrag_time = 3
net.ipv4.ipfrag_high_thresh = 134217728
```

## Suggested Cyclone DDS configuration

The following is a configuration file that is suggested by the [Autoare Network Settings](https://autowarefoundation.github.io/autoware-documentation/main/installation/additional-settings-for-developers/network-configuration/dds-settings/#cyclonedds-configuration) documentation.

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface autodetermine="false" name="lo" priority="default" multicast="default" />
      </Interfaces>
      <AllowMulticast>default</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Internal>
      <SocketReceiveBufferSize min="10MB"/>
      <Watermarks>
        <WhcHigh>500kB</WhcHigh>
      </Watermarks>
    </Internal>
  </Domain>
</CycloneDDS>
```

## Reference

* [1] [Autoare Network Settings](https://autowarefoundation.github.io/autoware-documentation/main/installation/additional-settings-for-developers/network-configuration/)
* [2] [ROS2 DDS Tuning](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html#)