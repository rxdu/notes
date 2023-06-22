# GPG Reference

GPG (GNU Privacy guard) is an open-source implementation of the OpenPGP protocol. It's often used to sign contents to ensure you're getting authentic information from the original publisher, not a modified version by someone else. It can also be used to encrypt and decrypt contents so that only people with the public key can get access to the contents that are encrypted with the paring private key.

Comparatively, GPG is mainly used with information "at rest", while TLS and SSH are more often used for data "in transit". 

## Install GPG

```
$ sudo apt install gpg
$ gpg --version
```

## Generate a Key

If you're generating the key in a graphical environment (where gpg will pop up a window to ask you to type in the password)

```
$ gpg --expert --full-gen-key
```

If you generate GPG key on the console or in a pure command-line environment, you should run the command with "--pinentry-mode=loopback" argument:

```
$ gpg --expert --pinentry-mode=loopback --full-gen-key
```

Example:

```
rdu@rdu-acer:~/Website/notes$ gpg --expert --full-gen-key
gpg (GnuPG) 2.2.27; Copyright (C) 2021 Free Software Foundation, Inc.
This is free software: you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.

Please select what kind of key you want:
   (1) RSA and RSA (default)
   (2) DSA and Elgamal
   (3) DSA (sign only)
   (4) RSA (sign only)
   (7) DSA (set your own capabilities)
   (8) RSA (set your own capabilities)
   (9) ECC and ECC
  (10) ECC (sign only)
  (11) ECC (set your own capabilities)
  (13) Existing key
  (14) Existing key from card
Your selection? 9
Please select which elliptic curve you want:
   (1) Curve 25519
   (3) NIST P-256
   (4) NIST P-384
   (5) NIST P-521
   (6) Brainpool P-256
   (7) Brainpool P-384
   (8) Brainpool P-512
   (9) secp256k1
Your selection? 1
Please specify how long the key should be valid.
         0 = key does not expire
      <n>  = key expires in n days
      <n>w = key expires in n weeks
      <n>m = key expires in n months
      <n>y = key expires in n years
Key is valid for? (0) 1y
Key expires at Fri 21 Jun 2024 01:57:56 PM +08
Is this correct? (y/N) y

GnuPG needs to construct a user ID to identify your key.

Real name: Ruixiang Du
Email address: ruixiang.du@westonrobot.com
Comment: test keygen
You selected this USER-ID:
    "Ruixiang Du (test keygen) <ruixiang.du@westonrobot.com>"

Change (N)ame, (C)omment, (E)mail or (O)kay/(Q)uit? O
We need to generate a lot of random bytes. It is a good idea to perform
some other action (type on the keyboard, move the mouse, utilize the
disks) during the prime generation; this gives the random number
generator a better chance to gain enough entropy.
We need to generate a lot of random bytes. It is a good idea to perform
some other action (type on the keyboard, move the mouse, utilize the
disks) during the prime generation; this gives the random number
generator a better chance to gain enough entropy.
gpg: key 713107D9E065DBCE marked as ultimately trusted
gpg: directory '/home/rdu/.gnupg/openpgp-revocs.d' created
gpg: revocation certificate stored as '/home/rdu/.gnupg/openpgp-revocs.d/D9304ED388718B0A014F262D713107D9E065DBCE.rev'
public and secret key created and signed.

pub   ed25519 2023-06-22 [SC] [expires: 2024-06-21]
      D9304ED388718B0A014F262D713107D9E065DBCE
uid                      Ruixiang Du (test keygen) <ruixiang.du@westonrobot.com>
sub   cv25519 2023-06-22 [E] [expires: 2024-06-21]
```

## Manage Keys

List all keys in your public keyring

```
$ gpg --list-keys
$ gpg --list-secret-key
```

List all keys with signature

```
$ gpg --list-sigs
```

Check your keys generated with your email

```
$ gpg --list-sigs <your-email>
```

Delete a key

```
$ gpg --delete-key <key-id>
```

Example:

```
$ gpg --list-sigs ruixiang.du@westonrobot.com
gpg: checking the trustdb
gpg: marginals needed: 3  completes needed: 1  trust model: pgp
gpg: depth: 0  valid:   1  signed:   0  trust: 0-, 0q, 0n, 0m, 0f, 1u
gpg: next trustdb check due at 2024-06-21
pub   ed25519 2023-06-22 [SC] [expires: 2024-06-21]
      D9304ED388718B0A014F262D713107D9E065DBCE
uid           [ultimate] Ruixiang Du (test keygen) <ruixiang.du@westonrobot.com>
sig 3        713107D9E065DBCE 2023-06-22  Ruixiang Du (test keygen) <ruixiang.du@westonrobot.com>
sub   cv25519 2023-06-22 [E] [expires: 2024-06-21]
sig          713107D9E065DBCE 2023-06-22  Ruixiang Du (test keygen) <ruixiang.du@westonrobot.com>
```

In the above example, 713107D9E065DBCE is the key ID and D9304ED388718B0A014F262D713107D9E065DBCE is the key fingerprint.

## Export Keys to Files

```
$ gpg --armor --export <key-id> > pubkey.asc
$ gpg --export-secret-keys --armor <key-id> > privkey.asc
```

You can name the file to anything you want. But "normally, .sig is used for detached signatures using the binary OpenPGP format, and .asc for when the contents are ASCII-armored. For everything else, .gpg is common for the binary format, .asc when armored." [2]

## Share Public Keys on Public Keyserver

Note you should only share public keys with others and never share your private keys.

```
$ gpg --send-key <key-id>
```

By default, it will send the key to "https://keys.openpgp.org/", you can specify a different keyserver, for example

```
$ gpg --keyserver hkps://keyserver.ubuntu.com --send-key <key-id>
```

## Import Keys from File/Keyserver

```
$ gpg --import <key-file>
$ gpg --recv-keys <key-id>
```

## Extend Key Expiration Date

You can modify the expiration date for both the primary key (0) and subkey (1). Read more about subkeys here [4].

```
$ gpg --pinentry-mode=loopback --edit-key <key-id>
gpg> key 0
gpg> expire
# enter new date and confirm
gpg> key 1
gpg> expire
# enter new date and confirm
gpg> save
```

Example:

```
$ gpg --pinentry-mode=loopback --edit-key 713107D9E065DBCE
gpg (GnuPG) 2.2.27; Copyright (C) 2021 Free Software Foundation, Inc.
This is free software: you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.

Secret key is available.

sec  ed25519/713107D9E065DBCE
     created: 2023-06-22  expires: 2024-06-21  usage: SC  
     trust: ultimate      validity: ultimate
ssb  cv25519/FCFBABCEE6D345D4
     created: 2023-06-22  expires: 2024-06-21  usage: E   
[ultimate] (1). Ruixiang Du (test keygen) <ruixiang.du@westonrobot.com>
gpg> key 0

sec  ed25519/713107D9E065DBCE
     created: 2023-06-22  expires: 2024-06-21  usage: SC  
     trust: ultimate      validity: ultimate
ssb  cv25519/FCFBABCEE6D345D4
     created: 2023-06-22  expires: 2023-06-23  usage: E   
[ultimate] (1). Ruixiang Du (test keygen) <ruixiang.du@westonrobot.com>

gpg> expire
Changing expiration time for the primary key.
Please specify how long the key should be valid.
         0 = key does not expire
      <n>  = key expires in n days
      <n>w = key expires in n weeks
      <n>m = key expires in n months
      <n>y = key expires in n years
Key is valid for? (0) 1
Key expires at Fri 23 Jun 2023 03:10:59 PM +08
Is this correct? (y/N) y

sec  ed25519/713107D9E065DBCE
     created: 2023-06-22  expires: 2023-06-23  usage: SC  
     trust: ultimate      validity: ultimate
ssb  cv25519/FCFBABCEE6D345D4
     created: 2023-06-22  expires: 2023-06-23  usage: E   
[ultimate] (1). Ruixiang Du (test keygen) <ruixiang.du@westonrobot.com>

gpg: WARNING: Your encryption subkey expires soon.
gpg: You may want to change its expiration date too.
gpg> save
```

## Reference

* [1] https://www.linuxbabe.com/security/a-practical-guide-to-gpg-part-1-generate-your-keypair
* [2] https://superuser.com/questions/814355/what-file-extensions-should-be-used-on-gpg-generated-output
* [3] https://www.linuxbabe.com/security/gpg-guide-public-key-management
* [4] https://wiki.debian.org/Subkeys