# Install Docker in Raspberry Pi OS

## Install Docker

```bash
$ curl -fsSL https://get.docker.com -o get-docker.sh
$ chmod +x ./get-docker.sh
$ sudo sh get-docker.sh
$ sudo systemctl to enable Docker
```

## Include a Non-Root Account to the Docker Group

```bash
$ sudo usermod -aG docker ${USER}
$ sudo usermod -aG docker ${USER}
```

Check whether it’s running:

```bash
$ groups ${USER}
```

## Install Docker-Compose

```bash
$ sudo apt-get install libffi-dev libssl-dev
$ sudo apt install python3-dev
$ sudo apt-get install -y python3 python3-pip
$ sudo pip3 install docker-compose
```

## Reference

* [1] https://jfrog.com/connect/post/install-docker-compose-on-raspberry-pi/