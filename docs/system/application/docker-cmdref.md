---
title: Docker Command Reference
description: Most commonly used docker commands
---

## Create/start/stop a Container

* Create and run a container

```bash
$ sudo docker run -it --rm <target-container> bash
```

* List all (running and stopped) containers

```bash
$ sudo docker ps -a
```

* Run a command in a running container

```bash
$ sudo docker exec -it <target-container> bash
```

* Stop/start/restart/remove a container

```bash
$ sudo docker stop <target-container>
$ sudo docker start <target-container>
$ sudo docker restart <target-container>
$ sudo docker rm <target-container>
```

* Check resource usage statistics of a container

```bash
$ sudo docker stats <target-container>
```

* Check log output of a running container

```bash
$ sudo docker logs -f <target-container>
```

## Advanced Configurations

* Map persistent storage volume

```bash
# host-src: an absolute path or a name value
# options: rw, ro
$ sudo docker run --rm \
    -v [host-src:]container-dest[:<options>] \
    <target-container>
```

## Manage Docker Images

* Create a docker image

```bash
$ sudo docker build . -t <image-name>:<tag> -f ./Dockerfile
# Do not use cache when building the image
$ sudo docker build . -t <image-name>:<tag> -f ./Dockerfile --no-cache
```

* List all images

```bash
$ sudo docker image list
```

* Remove a docker image

```bash
$ sudo docker image rm <image-name>:<tag>
$ sudo docker rmi <image-name>:<tag>
```

* Tag a docker image

```bash
$ sudo docker tag <image-id-or-name> <image-name>:<tag>
```

* Create a new image from a container’s changes

```bash
# first check changes
$ sudo docker diff <target-container>
# if all changes are okay, commit changes
$ sudo docker commit -a "author-name" -m "change msg" <target-container> <image-name>:<tag>
```

* Push/pull image from/to a docker registry

```bash
$ sudo docker pull <image-name>:<tag>
$ sudo docker push <image-name>:<tag>
```

## Docker Compose

Sample compose

```yml
version: '3'
services:
  service_one:
    privileged: true
    devices: 
      - /dev/video0:/dev/video101
    image: "sample-image:latest"
    logging:
        driver: "json-file"
        options:
            max-file: "5"
            max-size: "10m"
    network_mode: "host"
    volumes:
      - "/opt/config.yaml:/config.yaml"
    command: ["arg1", "/config.yaml"]
    restart: unless-stopped
  service_two:
    image: "sample-image:latest"
    user: root
    logging:
        driver: "json-file"
        options:
            max-file: "5"
            max-size: "10m"
    network_mode: "host"
    command: ["arg1", "arg2", "arg3"]
    restart: unless-stopped
```

## Nvidia Docker

You may need to pass in additional arguments to use the nvidia runtime:

```bash
$ sudo docker run --runtime=nvidia --network host -it <image-name>:<tag>
```

Allow containers to communicate with Xorg

```bash
$ sudo xhost +
$ sudo docker run -it --rm --net=host --runtime nvidia -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix nvcr.io/nvidia/l4t-base:r34.1
```

Option explained:

* -it means run in interactive mode
* --rm will delete the container when finished
* --runtime nvidia will use the NVIDIA container runtime while running the l4t-base container
* -v is the mounting directory, and used to mount hostÂs X11 display in the container filesystem to render output videos


## Reference

* https://docs.docker.com/engine/reference/commandline/run/
* https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-base