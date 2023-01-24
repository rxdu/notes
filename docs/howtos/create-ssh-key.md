---
title: Create SSH Key
description: How to create SSH key pair in Linux
---

Create key pair using different algorithms

```bash
$ ssh-keygen -t rsa -b 4096
$ ssh-keygen -t dsa
$ ssh-keygen -t ecdsa -b 521
$ ssh-keygen -t ed25519
```  

Specify a file name while creating the key pair

```bash
$ ssh-keygen -f ~/.ssh/rdu_github -t rsa -b 4096
```

Typically you want the permissions to be:

* .ssh directory: 700 (drwx------)
* public key (.pub file): 644 (-rw-r--r--)
* private key (id_rsa): 600 (-rw-------)

To access Github repositories with a SSH key, you may add the following config to ~/.ssh/config

```bash
Host *
  IdentitiesOnly=yes

Host github.com
HostName github.com
PreferredAuthentications publickey
IdentityFile ~/.ssh/<your-ssh-key-file-name>
```

If you have multiple Github accounts, you could add the second account like this

```bash
Host <2nd-name>.github.com
HostName github.com
PreferredAuthentications publickey
IdentityFile ~/.ssh/<your-2nd-ssh-key-file-name>
```

Then you can add "<2nd-name>" to the github ssh url in order to push or pull.

## Reference

- [1] https://www.ssh.com/ssh/keygen/
- [2] https://superuser.com/questions/215504/permissions-on-private-key-in-ssh-folder