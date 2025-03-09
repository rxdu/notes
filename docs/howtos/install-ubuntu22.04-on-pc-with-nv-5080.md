# Install Ubuntu 22.04 on PC with Nvidia 50-series Graphic Card

Since Nvidia 50-series graphics cards are quite new and they require at least nvidia version 570 driver to work properly, the default Ubuntu 22.04.5 installation image (the newest version at the time of writing) won't work. It only gives you a black screen after the "Try or Install Ubuntu" step, making it impossible to continue with the installation. Before the next point release that bundles a newer nvidia driver comes out, an easy solution that you can use at the moment is to create a customized installation image.

* Download Ubuntu 22.04.5 iso image
* Install the Cubic Custom Ubuntu ISO Creator

```bash
sudo apt-add-repository universe
sudo apt-add-repository ppa:cubic-wizard/release
sudo apt update
sudo apt install --no-install-recommends cubic
```

* Start the customization process (you can refer to [2] for more detailed instructions) and install nvidia driver (refer to [3])

```bash
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update

sudo apt install nvidia-driver-570-server-open
```

* After all the Cubic customization steps are finished, you will get a new iso image that you can use to install Ubuntu on your computer 

## Reference

* [1] https://github.com/PJ-Singh-001/Cubic
* [2] https://www.makeuseof.com/create-custom-ubuntu-iso-cubic/
* [3] https://launchpad.net/~graphics-drivers/+archive/ubuntu/ppa