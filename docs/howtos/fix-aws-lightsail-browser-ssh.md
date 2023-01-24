# Fix Lightsail SSH

When updating the Ubuntu system, I accidentally changed the SSH settings and the broswer SSH won't allow me to login. The error message was something like: 

```
Log in failed. If this instance has just started up, try again in a minute or two.

CLIENT_UNAUTHORIZED [769]
```

In order to fix this issue, you can ssh into the system using the SSH key and then add the following two lines to the SSH config file at "/etc/ssh/sshd_config"

```bash
TrustedUserCAKeys /etc/ssh/lightsail_instance_ca.pub
CASignatureAlgorithms +ssh-rsa
```

Restart the instance and the browser-based SSH should work again.