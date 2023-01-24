# Linux Commands

## Add New User

```bash
$ sudo adduser --ingroup users <USERNAME>
$ sudo adduser <YOUR_USERNAME> sudo
$ logout
$ sudo deluser --remove-home user
```

## Setup Wifi

* Generate WPA passphrase for your WiFi

```bash
$ wpa_passphrase <ssid> <password>
```

* Update /etc/network/interfaces

```bash
$ sudo nano /etc/network/interfaces
```

```bash
# interfaces(5) file used by ifup(8) and ifdown(8)
auto lo
iface lo inet loopback

auto wlan0
iface wlan0 inet dhcp
    wpa-ssid <ExampleWifi>
    wpa-psk <wpa-psk-generated-by-wpa-passphrase-command>
```

## Update System Time

```bash
$ sudo apt-get install ntp       
```


## Download File with URL

```bash
$ curl -O link-to-remote-file
```
   
## Get Root Privileges

if root user account is enabled.

```bash
$ su
```
You can logout using key: Ctrl+D 

Otherwise if system only allows using sudo

```bash
$ sudo -i
```

## Merge Multiple PDFs

```bash
# Install pdftk
$ sudo snap install pdftk

# Merge files
$ pdftk file1.pdf file2.pdf cat output result.pdf
```

## Extract Pages from PDFs

```bash
$ sudo apt-get install qpdf
# extract page 1 to 5 from input.pdf to output.pdf
$ qpdf input.pdf --pages . 1-5 -- output.pdf
```

## Convert PDF to JPG

```bash
$ sudo apt install libvips-tools
$ vips copy input.pdf[dpi=300] output.jpg
```

## Create Gif Animation

```bash
$ convert -delay 120 -loop 0 *.png animated.gif
```

## Screenshot and Screencast

```bash
$ sudo apt install kazam
```

## Get statistics of a code base

```bash
$ sudo apt install cloc
$ cd <code-folder>
$ cloc --exclude-dir=cmake,third_party .
```

## Reference

* [1] https://askubuntu.com/questions/2799/how-to-merge-several-pdf-files
