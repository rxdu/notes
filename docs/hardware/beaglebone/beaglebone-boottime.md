# Reduce Boot Time for Beaglebone

The following packages can be safely removed to reduce boot time:

```bash
$ sudo apt remove haveged bonescript c9-core-installer bb-node-red-installer nodejs bone101 nginx-full --purge
```

## Reference

* [1] https://github.com/RobertCNelson/omap-image-builder/issues/113