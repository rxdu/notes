# Fix Raspberry Pi Locale Issue

To get rid of the locale warning on raspberry pi:

```bash
$ sudo sed -i "s/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/g" -i /etc/locale.gen
$ sudo locale-gen en_US.UTF-8
$ sudo update-locale en_US.UTF-8

$ export LANGUAGE=en_US.UTF-8
$ export LANG=en_US.UTF-8
$ export LC_ALL=en_US.UTF-8
```

## Reference

- [1] https://gist.github.com/tkhduracell/f54734c18ef1a2fd99e9