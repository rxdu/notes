# Systemd Reference

## Systemd Utils

* To list all services

```bash
$ sudo systemctl list-units --type=service
```

* To start/stop/restart/status/enable/disable a service

```bash
$ sudo systemctl [start/stop/restart/status/enable/disable] <service-name>
```

To enable a service means that the service will be started at boot time.

* To check the log of a service

```bash
$ sudo journalctl -u <service-name>
```

If you want to follow the log, you can use the `-f` option, e.g. `sudo journalctl -fu <service-name>`.

* To reload all the unit files after making changes

```bash
$ sudo systemctl daemon-reload
```

## Systemd Unit Files

Systemd unit files may be loaded from multiple locations. The main ones are (listed from lowest to highest precedence): 

* **/usr/lib/systemd/system/**: units provided by installed packages
* **/etc/systemd/system/**: units installed by the system administrator

A typical systemd unit file looks like this:

```
[Unit]
Description=sample service
Requires=udisks2.service
Requires=graphical.target
After=graphical.target

[Service]
User=root
Type=oneshot
ExecStartPre=/bin/sh -c 'echo "pre start"'
ExecStart=/usr/bin/notify-send "Hello World"
ExecStartPost=/bin/sh -c 'echo "post start"'
ExecStop=/usr/local/bin/handle_external_hdds.sh
Restart=on-failure
TimeoutSec=2s
RestartSec=5s

[Install]
WantedBy=multi-user.target
```

### [Unit] Section

In the [Unit] section, you can specify dependencies of the service:

* **Requires**: mandatory dependencies
* **Wants**: optional dependencies
* **After**: the service will be started after the specified services

"Note that Wants= and Requires= do not imply After=, meaning that if After= is not specified, the two units will be started in parallel."[1]

### [Service] Secion

In the [Service] secion, you can specify what the service does.

Service type can one of simple/forking/oneshot/dbus/simple/notify. 

* The default type is "simple" if "Type" is not set but "ExecStart" is set.
* The default type is "oneshot" if both "Type" and "ExecStart" are not set.

The "oneshot" type "indicates that the process will be short-lived and that systemd should wait for the process to exit before continuing on with other units."[2]

To configure the service to auto-restart, you can use the following options [2]:

* **Restart=**always/on-success/on-failure/on-abnormal/on-abort/on-watchdog
* **RestartSec=**: the amount of time to wait before attempting to restart the service
* **TimeoutSec=**: the amount of time that systemd will wait when stopping or stopping the service before marking it as failed or forcefully killing it

## Reference

* [1] https://wiki.archlinux.org/title/systemd
* [2] https://www.digitalocean.com/community/tutorials/understanding-systemd-units-and-unit-files
* [3] https://man.archlinux.org/man/systemd.service.5#EXAMPLES
