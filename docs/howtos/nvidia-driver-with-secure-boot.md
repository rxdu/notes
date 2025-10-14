# NVIDIA Driver Installation with Secure Boot

When installing NVIDIA drivers on a system with Secure Boot enabled, you may encounter issues due to the kernel module not being signed. This guide will walk you through the steps to properly install NVIDIA drivers while ensuring compatibility with Secure Boot.

It's assumed that the secure boot is already enabled in the BIOS settings. And you can boot into the Ubuntu system but the NVIDIA graphics driver is not working properly (nvidia-smi doesn't show any NVIDIA devices).

## Install NVIDIA Driver

Since driver version 515+, NVIDIA open-sourced their kernel module. This is generally preferable as it allows for easier integration with the Linux kernel and better support for Secure Boot.

1. Add the NVIDIA PPA (Personal Package Archive) to your system:
```bash
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update
```

2. Install the NVIDIA driver:
```bash
# open-source driver
sudo apt install nvidia-driver-570-open

# if you prefer the proprietary driver, use:
sudo apt install nvidia-driver-570
```

3. Reboot your system:
```bash
sudo reboot
```

4. After rebooting, check if the NVIDIA driver is working:
```bash
nvidia-smi
```

The installation process should automatically handle the signing of the kernel module for Secure Boot. But from my experience, the signing process may fail sometimes. If you encounter issues, you may need to manually sign the kernel module.

## Manually Sign the Kernel Module

```bash
sudo mokutil --import /var/lib/shim-signed/mok/MOK.der
```

You will be prompted to create a password. This password will be used later in the MOK (Machine Owner Key) management during the next reboot.

Sign the NVIDIA kernel module:

```bash
sudo /usr/src/linux-headers-$(uname -r)/scripts/sign-file sha256 \
  /var/lib/shim-signed/mok/MOK.priv \
  /var/lib/shim-signed/mok/MOK.der \
  $(modinfo -n nvidia)
```

Reboot your system again and follow the prompts to enroll the MOK key you created earlier. You will need to enter the password you set during the `mokutil` step.

* Select Enroll MOK
* Choose Continue
* Enter the password you set above
* Finish and reboot

After rebooting, check if the NVIDIA driver is working:

## Reference

* [NVIDIA Driver PPA](https://launchpad.net/~graphics-drivers/+archive/ubuntu/ppa)