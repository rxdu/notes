# Pair Bluetooth Device from a Terminal

Pairing and connecting to a bluetooth device from a terminal is useful when you are working with a single-board computer and you don't have a monitor connected to it.

## Start the Bluetooth control utility

Install bluetooth tools if you haven't:

```bash
$ sudo apt -y install bluetooth bluez bluez-tools
```

Start the control utility:

```bash
$ bluetoothctl
```

You should get into the bluetoothctl prompt mode:

```bash
rdu@rpi4:~ $ bluetoothctl 
Agent registered
```

## Commands with the bluetoothctl prompt

* Turn on Bluetooth

```bash
power on
```

* Enable agent

```bash
agent on
```

* Scan for nearby devices

```bash
scan on
```

Wait for the device you want to pair with to appear in the list. Once it appears, note down its MAC address.

* Pair with the device

```bash
pair <MAC_ADDRESS>
```

* Connect to the device

```bash
connect <MAC_ADDRESS>
```

* Exit the Bluetooth control utility (or simply use Ctrl + D)

```bash
exit
```

Note you can also use the above command from the normal terminal prompt in the form: `bluetoothctl + [command]`, such as 

```bash
$ bluetoothctl power on
```

## Reference

* [1] https://www.geeksforgeeks.org/connecting-to-bluetooth-devices-via-cli/
* [2] https://www.baeldung.com/linux/bluetooth-via-terminal