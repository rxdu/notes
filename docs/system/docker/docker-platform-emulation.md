# Docker Platform Emulation

In some cases, it's convenient to build and run non-x64 container images on a x64 computer. An example is to build docker images for [raspberry pi](../../hardware/raspberrypi/raspberrypi-cross-chroot-docker.md) or jetson on your host computer, which has a more powerful CPU.

## Install Tools

```bash
$ sudo apt install qemu qemu-user-static binfmt-support
$ docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

## Run the Container

For a multi-arch image hosted on container registry (such as Docker Hub), you can explicitly specify which platform of the image you want to pull or run:

```bash
# run arm64
$ docker pull --platform linux/arm64 alpine:latest
```

If you don't specify the platform, docker will try to pull the image for the same platform of your host computer. It will fail if the image is just for a different platform, giving you error messages similar to

```bash
no matching manifest for linux/amd64 in the manifest list entries
```


## Reference

* https://www.stereolabs.com/docs/docker/building-arm-container-on-x86