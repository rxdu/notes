# Edit Grub Menu

Ubuntu uses Grub2 as its bootloader. If you have installed multiple operating systems on your computer, you may need to modify the boot order or remove certain boot entries.

There are two files you may need to modify:

* `/etc/default/grub`: contains the default settings and configurations for GRUB
* `/boot/grub/grub.cfg`: is the actual GRUB configuration file that the GRUB bootloader reads at boot time

If you modify `/etc/default/grub`, you need to run 

```bash
$ sudo update-grub
```

to update the changes to `/boot/grub/grub.cfg`.  

Typically, you may update `/etc/default/grub` to change:

* whether to show the boot menu (GRUB_DISABLE_OS_PROBER)
* timeout of the boot menu (GRUB_TIMEOUT)

You may need to update `/boot/grub/grub.cfg` if you want to change the boot menu entries. The following shows an example of the menu entry and submenu:

```bash
menuentry 'Ubuntu' --class ubuntu --class gnu-linux --class gnu --class os ...
        recordfail
        load_video
        ...
}
submenu 'Advanced options for Ubuntu' $menuentry_id_option ... {
        menuentry 'Ubuntu, with Linux 6.5.0-35-generic' --class ubuntu ...
                recordfail
                load_video
                ...
        }
```

Typically you may comment out or change the order of the menus and submenus.

## Reference

* https://www.ubuntubuzz.com/2022/06/how-to-edit-ubuntu-bootloader-menu-made-simple.html