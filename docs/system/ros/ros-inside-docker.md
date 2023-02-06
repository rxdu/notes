# ROS inside Docker Containers

Tier-1 support of a specific ROS release is limited to one or two Ubuntu releases, for exmaple, ROS Melodic on Ubuntu 18.04, ROS Foxy on Ubuntu 20.04 and ROS Humble on Ubuntu 22.04. It's inconvenient to install multiple distributions and switch between each other when your projects rely on different ROS versions. 

In fact, you may also face similar issues when doing other kinds of development where setting up the development environment is non-trivial. In such case you can consider build and run your application in a container. You can follow normal Docker configuration steps to acheive this, but the [**VSCode with the Remote - Containers plugin**](https://code.visualstudio.com/docs/remote/containers) makes the process very handy. The following figure from the official documentation clearly illustrates how this works.

![architecture](./figures/architecture-containers.png)

