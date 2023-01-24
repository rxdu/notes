---
title: ROS2 CLI Reference
description: 
---

## Environment Variables

The following environment variables should have been set properly after a successful installation:

```
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=humble
```

* To use a different domain ID

```bash
$ export ROS_DOMAIN_ID=<your_domain_id>
```
Due to platform-specific constrains, a safe option of a domain ID is between 0 and 101 inclusively. All ROS 2 nodes use domain ID 0 by default. [1]

* To limit the ROS2 communication to localhost

```bash
$ export ROS_LOCALHOST_ONLY=1
````

## CLI Tools

### ROS pkg

```bash
$ ros2 pkg create <package_name>
$ ros2 pkg list
```

### ROS node

```bash
$ ros2 run <package_name> <executable_name>
$ ros2 node list
$ ros2 node info <node_name>
```

### ROS lifecycle

```bash
# output a list of nodes with lifecycle
$ ros2 lifecycle nodes
# output a list of available transitions for a node
$ ros2 lifecycle list <node_name>
# get lifecycle state of a node
$ ros2 lifecycle get <node_name>
# trigger lifecycle state transition of a node
$ ros2 lifecycle set <node_name> <transition>
```

Possible transitions to invoke are [4]:

* configure
* activate
* deactivate
* cleanup
* shutdown

### ROS daemon

```bash
$ ros2 daemon start/status/stop
```

The ROS2 daemon is a node that helps to accelerate the node discovery process. It is not required to run ROS2 nodes but it makes the newly started node to discover other nodes faster, which is especially useful for CLI commands like "ros2 node list".

### ROS topic

```bash
$ ros2 topic list
$ ros2 topic echo <topic_name>
$ ros2 topic info/type/hz/bw/delay <topic_name>
$ ros2 topic pub <topic_name> <msg_type> [arguments]
```

### ROS service

```bash
$ ros2 service list -t
$ ros2 service type <service_name>
$ ros2 service call <service_name> <service_type> [arguments]
```

### ROS interface

This set of commands is to check the ROS messages and services (as the interface between nodes)

```bash
$ ros2 interface list
$ ros2 interface proto/show <msg_type>
```
An example:
```bash
$ ros2 interface show geometry_msgs/msg/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
	float64 x
	float64 y
	float64 z
Vector3  angular
	float64 x
	float64 y
	float64 z
```

### ROS param

```bash
$ ros2 param list
$ ros2 param get <node_name> <parameter_name>
$ ros2 param set <node_name> <parameter_name> <value>
```

To handle parameter files:

```bash 
$ ros2 param dump <node_name>
$ ros2 param load <node_name> <parameter_file>
```

```bash
# example: dump parameters of a node to file
$ ros2 param dump /turtlesim > turtlesim.yaml
$ ros2 param load /turtlesim turtlesim.yaml
```

To start the same node using your saved parameter values, use:

```bash
$ ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
```
```bash
# example: start turtlesim_node with turtlesim.yaml
$ ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim.yaml
```

### ROS action

```bash
$ ros2 action list [-t]
$ ros2 action info <action_name>
$ ros2 action send_goal <action_name> <action_type> <values>
```

### ROS bag

```bash
$ ros2 bag record <topic_name>
$ ros2 bag info <bag_file_name>
$ ros2 bag play <bag_file_name>
```

```bash
# commonly used arguments
$ ros2 bag record -a -o <bag_file_name>
$ ros2 bag play -l <bag_file_name>
```

### ROS multicast

```bash
$ ros2 multicast receive/send
```

This command pair is useful for testing if UDP multicast is working properly. You open one terminal and run "ros2 multicast receive" and then open another terminal and run "ros2 multicast send". If the send command is successful, you should see the message in the receive terminal.

On Linux, if the multicast communication is not successful, try to run the following command to enable multicast on the interface. 

```bash
$ sudo ip link set <interface_name> multicast on
```

And you may also need to update the firewall rules to allow multicast traffic. [3]

```bash
$ sudo ufw allow in proto udp to 224.0.0.0/4
$ sudo ufw allow in proto udp from 224.0.0.0/4
```

## Build Tools: colcon

### Build workspace

```bash
$ colcon build --symlink-install
# build the selected package only
$ colcon build --symlink-install --packages-select <package_name>
# build the selected package and its dependencies
$ colcon build --symlink-install --packages-up-to <package_name>
# build the selected package and packages that depends on it
$ colcon build --symlink-install --packages-above <package_name>
```

The "--symlink-install" option is to create a symlink to the build directory instead of copying the files. "This allows the installed files to be changed by changing the files in the source space (e.g. Python files or other not compiled resourced) for faster iteration."

### Run tests

```bash
$ colcon test
```

## Reference

* [1] https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html
* [2] https://github.com/ubuntu-robotics/ros2_cheats_sheet
* [3] https://docs.ros.org/en/rolling/How-To-Guides/Installation-Troubleshooting.html#enable-multicast
* [4] https://github.com/ros2/demos/blob/humble/lifecycle/README.rst