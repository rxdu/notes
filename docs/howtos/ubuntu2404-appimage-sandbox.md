# Ubuntu 24.04 AppImage Sandbox

On Ubuntu 24.04, you may encounter the following error when running an AppImage:

```
$ ./Obsidian-1.8.7.AppImage 
[15810:0302/204155.842217:FATAL:setuid_sandbox_host.cc(163)] The SUID sandbox helper binary was found, but is not configured correctly. Rather than run without sandboxing I'm aborting now. You need to make sure that /tmp/.mount_ObsidinaDjMJ/chrome-sandbox is owned by root and has mode 4755.
Trace/breakpoint trap (core dumped)
```

You can create a profile for the AppImage to allow it to run without sandboxing by creating a file at `/etc/apparmor.d/obsidian.appimage` with the following content:

```bash
# This profile allows everything and only exists to give the
# application a name instead of having the label "unconfined"

abi <abi/4.0>,
include <tunables/global>

profile obsidian.appimage /path/to/Obsidian-1.6.7.AppImage flags=(default_allow) {
  userns,

  # Site-specific additions and overrides. See local/README for details.
  include if exists <local/obsidian.appimage>
}
```

## References

- https://askubuntu.com/questions/1512287/obsidian-appimage-the-suid-sandbox-helper-binary-was-found-but-is-not-configu
- https://ubuntu.com/blog/ubuntu-23-10-restricted-unprivileged-user-namespaces