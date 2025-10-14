# Restore Ubuntu Kernel

If you accidentally removed or corrupted your Ubuntu kernel, you won't be able to boot into your system. This note documents the steps to restore the Ubuntu kernel.

* Boot from a Live USB
* Mount your root partition and chroot into it
* Reinstall the kernel

## Recovery Steps

First boot from a Live USB of Ubuntu. Then open a terminal and mount your root partition. If you have multiple Ubuntu installations, make sure to mount the correct one. You can identify your OS by checking the `/etc/os-release` file after mounting the partition.

```bash
cat /etc/os-release
```

Once confirmed, mount the root partition and bind the necessary filesystems, then chroot into it:

* Mount the root partition (replace `/dev/sdXY` with your actual root partition)

```bash
sudo mount /dev/sdXY /mnt
```

* Prepare resolv.conf for resolving DNS inside the chroot environment

```bash
sudo cp /etc/resolv.conf /mnt/etc/resolv.conf
```

* Bind necessary filesystems and chroot into the mounted partition

```bash
sudo mount --bind /dev /mnt/dev
sudo mount --bind /proc /mnt/proc
sudo mount --bind /sys /mnt/sys
sudo chroot /mnt
```

```bash
# inside the chroot environment, update the package list and reinstall the kernel
apt update
apt install --reinstall linux-image-$(uname -r)

# e.g. for ubuntu 22.04 in Oct 2025, you may install
apt install linux-image-6.8.0-85-generic

# exit the chroot environment
exit
```

* Unmount the partitions and reboot

```bash
sudo umount /mnt/dev
sudo umount /mnt/proc
sudo umount /mnt/sys
sudo umount /mnt
sudo reboot
```

Now you should be able to boot into your Ubuntu system with the restored kernel. You may update/fix any other packages as needed.

## Reference

* https://askubuntu.com/questions/28099/how-to-restore-a-system-after-accidentally-removing-all-kernels