# GitHub Multi-Account SSH Configuration

If you have multiple Github accounts and want to use all of them on one computer, you would need to change the ssh config so that the git client will know which ssh key to use to access each of the account.

```bash
Host <1st-name>.github.com
  HostName github.com
  PreferredAuthentications publickey
  IdentityFile ~/.ssh/<your-1st-ssh-key-file-name>
  IdentitiesOnly=yes

Host <2nd-name>.github.com
  HostName github.com
  PreferredAuthentications publickey
  IdentityFile ~/.ssh/<your-2nd-ssh-key-file-name>
  IdentitiesOnly=yes
```

Note in the above configuration:

* **Host**: you can give any name to differentiate the accounts
* **IdentitiesOnly**: you must set to yes. This argument specifies that ssh should only use the authentication identity files configured in the ssh_config files, not to use the default id_rsa key or the ones from the ssh-agent. 

## Reference

- [1] https://www.youtube.com/watch?v=6lA0oPoFCAE