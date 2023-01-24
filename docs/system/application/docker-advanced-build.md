---
title: Advanced Docker Build
description: More advanced techniques for building docker images
---

## Conditional Build

Sometimes you may need to build the image with different sets of commands for a small part of the build process. You can choose which set to use by passing in user-defined arguments. There are two ways to handle this: 

* Use if/else condition from bash
* Use multi-stage build

If the number of different conditions is small and the command for each condition is short, using bash may be simpler. But generally, it's cleaner to use multi-stage build:

```docker
# control argument
ARG MY_ARG

# common tasks
FROM ubuntu:focal AS base
RUN echo "do common stuffs"

# condition 1
FROM base AS branch-arg1
RUN echo "this is the stage for MY_ARG=arg1"

# condition 2
FROM base AS branch-arg2
RUN echo "this is the stage for MY_ARG=arg2"

# more common tasks
FROM branch-${MY_ARG} AS final
RUN echo "more common stuffs"
```

Then you can build with command:

```bash
$ docker build --build-arg MY_ARG=arg1 .
```

## Build Multi-platform Images

You can use the buildx CLI plugin to create multi-platform images. It's a two-step process:

* Create a builder instance

Here you will create a builder with name "multi-platform" that supports "linux/amd64" and "linux/arm64":

```bash
$ sudo docker buildx create \
	--name multi-platform \
    --platform linux/amd64,linux/arm64 \
    --driver docker-container
```

* Use the builder instance to build the images

The following command will build the image for both "linux/amd64" and "linux/arm64" platforms and will push the image to the docker registry:

```bash
$ sudo docker buildx build \
		--build-arg MY_ARG=arg1 \
		--builder multi-platform \
		--output "type=image,push=true" \
		--platform linux/amd64,linux/arm64 \
		-t registry/image_name:tag \
		. -f ./Dockerfile
```


## Reference

* https://stackoverflow.com/questions/43654656/dockerfile-if-else-condition-with-external-arguments
* https://docs.docker.com.xy2401.com/buildx/working-with-buildx/
* https://blog.k4nz.com/5ec72c881c5ef14c740f876d7e26b9cd/
* https://www.docker.com/blog/faster-multi-platform-builds-dockerfile-cross-compilation-guide/