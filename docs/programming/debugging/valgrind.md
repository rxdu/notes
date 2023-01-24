---
title: Using Valgrind
description: A brief tutorial for using valgrind to check memory issues
---

Valgrind is a powerful toolset that can help to debug programs especially for memory-related issues.

A few references:

* [Valgrind Official Site](http://valgrind.org/)
* [Kcachegrind Project Site](http://kcachegrind.sourceforge.net/html/Usage.html)
* [How to profile C++ application with Callgrind / KCacheGrind](https://baptiste-wicht.com/posts/2011/09/profile-c-application-with-callgrind-kcachegrind.html)

## Frequently Used Commands

* Install the tool

```bash
# valgrind 
$ sudo apt-get install valgrind
# useful tools that can be used together with valgrind 
$ sudo apt-get install kcachegrind graphviz
```

* Memory error check

```bash
# quick check
$ valgrind <name-of-executable>
# full memory leak check
$ valgrind --leak-check=full <name-of-executable>
```

* Program profiling

```bash
# profile program and get an output file like callgrind.out.xxxxxx
$ valgrind --tool=callgrind <name-of-executable>
# then you can load the file with Kcachegrind
$ kcachegrind <name-of-profiling-output-file>
```
