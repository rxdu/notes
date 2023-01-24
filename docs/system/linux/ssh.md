# SSH Reference

## Setup authorized_keys

If you don't want to always type in password to access a remote server, you can create a keypair on your local machine and add the public key to the remote servers' authorized_keys file.

On your local machine:

```bash
# create a ssh key pair
$ ssh-keygen -f ~/.ssh/my_sshkey -t rsa -b 4096
# transfer the public key to the server
$ scp ~/.ssh/my_sshkey.pub <username>@<server-address>:~/.ssh
```

**Note**: you're only supposed to share the "PUBLIC" key to the remote server.

On the remote server:

```bash
$ cat ~/.ssh/my_sshkey.pub >> ~/.ssh/authorized_keys
```

A simpler method to copy the public key to the remote server is to use the "ssh-copy-id" tool.

```
$ ssh-copy-id -i ~/.ssh/my_sshkey.pub <username>@<server-address>
```

Now you can try the SSH access on the local machine:

```bash
# $ ssh -i ~/.ssh/my_sshkey <username>@<server-address>
```

If you don't want to pass in the argument "-i ~/.ssh/my_sshkey" everytime, you can add the following to your "~/.ssh/config" file:

```
Host <server-address>
  Preferredauthentications publickey
  IdentityFile ~/.ssh/my_sshkey
```

## SSH Agent

The previous section has shown how to add your public key to the authroized keys list on a remote server. If you generated your SSH key pair with passphrase, you will need to type in the passphrase every time you connect to the server with "my_sshkey". In order to simplify this, you can use ssh-agent to store and retrieve the credential for you from the background:

```bash
# start the ssh-agent
$ eval `ssh-agent`
# add key to the ssh-agent
$ ssh-add ~/.ssh/my_sshkey
# to check all keys added to ssh-agent
$ ssh-add -l
# to delete a key from ssh-agent
$ ssh-add -D <key-to-be-deleted>
```

Now you only need to type in the passphrase once at the very first time you use the key.

## SSH Agent Forwarding

A typical use case of SSH agent forwarding is when you need SSH access to some resources (e.g. a github repository) from a remote host (e.g. a cloud server or a computer onboard a robot) but you don't want to set up SSH keys on the remote host for security reasons or just for convenience.

```bash
$ ssh -A <username>@<host-address>
```

With the "-A" option, you can use your SSH credentials on the server just like you're on the local computer.

**Note**: for security reasons, it's recommended that you only use the agent forwarding when necessary and exit in time when done with work, since someone with root access on the server can possibly gain access to your SSH credentials and impersonate you for unauthorized operations. Also remember not to use the forwarding to a server that you don't trust. For the same reason, it's not recommended to have the "ForwardAgent yes" configuration in your "~/.ssh/config" for your hosts.

## SSH ProxyJump

If you want to SSH into a second server (target server) through the first server (bridge server) from you local computer, you can use ProxyJump.

```bash
$ ssh -J <bridge-server> <target-server> 
```

You can also setup the ProxyJump from your ssh config file:

```
Host <bridge-server>
	Preferredauthentications publickey
    IdentityFile ~/.ssh/my_sshkey

Host <target-server>
	ProxyJump <bridge-server>
	User <username>
```

### SSH Port Forwarding/Tunneling

A few examples of port forwarding with SSH from [5]:

* Local forwarding

```bash
$ ssh -L 80:intra.example.com:80 gw.example.com
```

"This example opens a connection to the gw.example.com jump server, and forwards any connection to port 80 on the local machine to port 80 on intra.example.com."

* Remote forwarding

```bash
$ ssh -R 8080:localhost:80 public.example.com
```

"This allows anyone on the remote server to connect to TCP port 8080 on the remote server. The connection will then be tunneled back to the client host, and the client then makes a TCP connection to port 80 on localhost. Any other host name or IP address could be used instead of localhost to specify the host to connect to."

"This particular example would be useful for giving someone on the outside access to an internal web server. Or exposing an internal web application to the public Internet. This could be done by an employee working from home, or by an attacker."

* Reverse SSH tunnel

The reverse tunneling is essentially the use of remote fowarding. The following example is from [4]:

"For instance, to connect to your_domain on port 80 on our local computer, making the connection available on our remote host on port 8888, you could type:"

```
$ ssh -f -N -R 8888:your_domain:80 username@remote_host
```

"Now, on the remote host, opening a web browser to 127.0.0.1:8888 would allow you to see whatever content is at your_domain on port 80."

The extra "-fN" arguments tells SSH to go to background just before command execution and do not execute a remote command. [7]

In the above example, if you want to terminate the forwarding, you need to find the process id first and then kill the process:

```bash
$ ps aux | grep 8888
```

```
Output
1001      5965  0.0  0.0  48168  1136 ?        Ss   12:28   0:00 ssh -f -N -R 8888:your_domain:80 username@remote_host
1001      6113  0.0  0.0  13648   952 pts/2    S+   12:37   0:00 grep --colour=auto 8888
```

Now you can kill the process with id 5965

```bash
$ kill 5965
```

## Reference

* [1] https://dev.to/levivm/how-to-use-ssh-and-ssh-agent-forwarding-more-secure-ssh-2c32
* [2] https://smallstep.com/blog/ssh-agent-explained/
* [3] https://www.cnblogs.com/f-ck-need-u/p/10484531.html
* [4] https://www.digitalocean.com/community/tutorials/ssh-essentials-working-with-ssh-servers-clients-and-keys
* [5] https://www.ssh.com/academy/ssh/tunneling/example
* [6] https://jfrog.com/connect/post/reverse-ssh-tunneling-from-start-to-end/#:~:text=Reverse%20SSH%20Tunneling%20enables%20you,have%20a%20public%20IP%20address.
* [7] https://linuxcommand.org/lc3_man_pages/ssh1.html