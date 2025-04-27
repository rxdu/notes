# Docker Volume Backup and Restore

!!! info 

    This note was written with the assistance of ChatGPT.

At the core, **Docker volumes** are just directories managed by Docker (often under `/var/lib/docker/volumes/<volume_name>/_data/` on Linux). To export and import a volume **properly and portably**, the most robust way is to **tar the contents**, then **copy and restore**.

## To Export (Backup) a Docker Volume

Suppose your volume is called `my_volume`.

```bash
docker run --rm -v my_volume:/volume -v $(pwd):/backup busybox tar czf /backup/my_volume.tar.gz -C /volume .
```

- `my_volume:/volume` mounts your volume into the container.
- `$(pwd):/backup` mounts your current directory for output.
- It creates a `my_volume.tar.gz` in your current directory containing all the files inside the volume.

Example:

```bash
# specify which volume to backup
VOLUME_NAME=my_volume

# create tarball of the volume
docker run --rm -v ${VOLUME_NAME}:/volume -v $(pwd):/backup busybox tar czf /backup/${VOLUME_NAME}.tar.gz -C /volume .
```

## To Import (Restore) the Volume on Another Computer

**First**, make sure the volume exists on the new machine:

```bash
docker volume create my_volume
```

**Then**, use a temporary container again to extract the tarball:

```bash
docker run --rm -v my_volume:/volume -v $(pwd):/backup busybox sh -c "cd /volume && tar xzf /backup/my_volume.tar.gz"
```

- Here, `/backup/my_volume.tar.gz` is the file you copied over from the first machine.

Example:

```bash
# specify which volume to restore
VOLUME_NAME=my_volume

# optional (if the volume doesn't exist yet)
docker volume create ${VOLUME_NAME}

# Overwrite the volume with the contents of the tarball
docker run --rm -v ${VOLUME_NAME}:/volume -v $(pwd):/backup busybox sh -c "cd /volume && tar xzf /backup/${VOLUME_NAME}.tar.gz"
```

## Some Extra Thoughts:

- If your volume is **large**, you can compress even further or split the file (`split` command).
- If you want a **full automated backup/restore**, you can script this process.
- For **highly dynamic data** (e.g., database volumes), consider **snapshotting while container is paused** to avoid inconsistency.
- For **critical applications**, it's better to have an application-level export (e.g., MySQL dump) rather than filesystem snapshotting.
