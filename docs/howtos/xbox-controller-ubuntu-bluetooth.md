---
title: Connect Xbox One Controller to Ubuntu via Bluetooth
description: xbox controller bluetooth
---

If the required driver module doesn't load automatically, the xbox one controller won't connect to Ubuntu reliably. In this case, you can load the module manually

```bash
$ sudo modprobe xpad
```

Then the controller should pair and get connected after you turn it on (pressing the xbox logo button), hold the bluetooth pairing button (the one beside the USB type-C port) until the logo light flashes fast. 

## Reference

- [1] https://www.reddit.com/r/linuxquestions/comments/cvtfbq/getting_my_wired_xbox_one_controllers_to_work_on/