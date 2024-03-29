# Create SSH Key

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
Host github.com
  HostName github.com
  PreferredAuthentications publickey
  IdentityFile ~/.ssh/<your-ssh-key-file-name>
  IdentitiesOnly=yes
```

## Reference

- [1] https://www.ssh.com/ssh/keygen/
- [2] https://superuser.com/questions/215504/permissions-on-private-key-in-ssh-folder