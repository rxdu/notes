# Journalctl Reference

## List Journal Entries from the Current Boot

* List logs from current boot

```bash
$ journalctl -b
```

* List logs from past boots

```bash
$ journalctl --list-boots
$ journalctl -b <boot-id>
# example
$ journalctl -b -1
```

## List Logs Based on Time

```bash
$ journalctl --since "2022-02-04 12:40:49”
$ journalctl --since "2015-06-26 23:15:00" --until "2015-06-26 23:20:00"
$ journalctl --since "yesterday"
$ journalctl --since 10:10 --until "1 hour ago"
```

## List Logs Based on Service Unit

* Check logs from a single service

```bash
$ journalctl -u <service-name.service>
# example
$ journalctl -u nginx.service --since today
```

* Check interleaved records from multiple units

```bash
$ journalctl -u nginx.service -u php-fpm.service --since today
```

* Following logs of services

```bash
$ journalctl -fu <service-name.service>
```

* List last n entries from the logs

```bash
$ journalctl -n <number-of-entries>
```

## Journalctl Storage

* Check existing disk usage

```bash
$ journalctl --disk-usage
```

* Delete old logs

```bash
$ sudo journalctl --vacuum-size=1G
$ sudo journalctl --vacuum-time=1years
```

* Keep logs persistent

```bash
$ sudo nano /etc/systemd/journald.conf
```

```
[Journal]
Storage=persistent
```

* Limit journal storage

You can set the storage limits in "/etc/systemd/journald.conf" by setting values to the following entries [1] 

* SystemMaxUse=: Specifies the maximum disk space that can be used by the journal in persistent storage.
* SystemKeepFree=: Specifies the amount of space that the journal should leave free when adding journal entries to persistent storage.
* SystemMaxFileSize=: Controls how large individual journal files can grow to in persistent storage before being rotated.
* RuntimeMaxUse=: Specifies the maximum disk space that can be used in volatile storage (within the /run filesystem).
* RuntimeKeepFree=: Specifies the amount of space to be set aside for other uses when writing data to volatile storage (within the /run filesystem).
RuntimeMaxFileSize=: Specifies the amount of space that an individual journal file can take up in volatile storage (within the /run filesystem) before being rotated.


## Reference

* [1] https://www.digitalocean.com/community/tutorials/how-to-use-journalctl-to-view-and-manipulate-systemd-logs
* [2] https://www.loggly.com/ultimate-guide/using-journalctl/
* [3] https://www.loggly.com/ultimate-guide/linux-logging-with-systemd/