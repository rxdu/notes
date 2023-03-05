# Fix librt.so issue on Ubuntu 22.04

On a freshly installed Ubuntu 22.04, you may get compile error complaining that librt.so cannot be found. You can fix the issue by creating a symbolic link to the librt.so:

```bash
$ sudo ln -s /lib/x86_64-linux-gnu/librt.so.1 /usr/lib/x86_64-linux-gnu/librt.so
```
