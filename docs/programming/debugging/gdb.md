---
title: Using GDB Debugger
description: A brief tutorial for using GDB to debug your program
---

## Introduction

GDB debugger can be a very powerful tool for finding the cause of unexpected behaviors of your program. In a GDB debugging session, you can step your code by line or by instruction and you can also stop your code at a specific line, a function or an address. When your program stops, you can check the values of the variables at that moment. You can even change a variable as you want. Moreover, it allows you to check the status of the stack and inspect how the function are called. The use of GDB may seem not to be as easy as printf() but it can be much more helpful when you're trying to find a deeply hidden bug. Actually it's not as difficut to learn as it seems to be.

Here are a few good tutorials to get started:

* [GDB Tutorial: A Walkthrough with Examples](http://www.cs.umd.edu/~srhuang/teaching/cmsc212/gdb-tutorial-handout.pdf)
* [Using GNU's GDB Debugger](http://www.dirac.org/linux/gdb/index.php)
* [Guide to Faster, Less Frustrating Debugging](http://heather.cs.ucdavis.edu/~matloff/UnixAndC/CLanguage/Debug.html)
* [RMS's GDB Debugger Tutorial](http://www.unknownroad.com/rtfm/gdbtut/gdbtoc.html)
* [Debugging with GDB](http://www.delorie.com/gnu/docs/gdb/gdb_toc.html)

The official manual of GDB can be a good reference after you have already learned the basics and want to check more details of a feature:

* [GDB Online Manual](https://sourceware.org/gdb/current/onlinedocs/gdb/)

The following are my notes for quick reference of the most frequently used commands. Some of them are from the above materials.

## Background Knowledge

It can be very helpful to understand stack and stack frames when you use GDB for debugging. This [page](http://www.dirac.org/linux/gdb/02a-Memory_Layout_And_The_Stack.php#wherearewegoingtogo) from the tutorial "Using GNU's GDB Debugger" gives good explanation about them.

Briefly speaking, stack is a section of the memory and whenever there is a function call, a stack frame is created in the stack. This stack frame is used to store all related information about this call including the all the [automatic variables](http://en.wikipedia.org/wiki/Automatic_variable) of the newly called function, the return address and the arguments of the called function. In GDB, you can check how the functions are called from each other by inspecting the stack frames. By switching to a stack frame, you get access to variables in that scope, meaning you can check the values of local variables in the corresponding function. You cannot print or modify a variable if that variable doesn't exit in the current stack frame.

In addition to stack and stack frames, check topic "operating system" for more information about virtual memory and memory layout.

Another thing that you need to remember before starting a gdb debugging session is that you need to use "-g" flag when compiling your code to include debugging information in the generated executable file.

## Frequently Used Commands

* Starting a debugging session

```bash
$ gdb <name-of-executable>
(gdb) run arg1 arg2
```

Arg1 and arg2 will be remembered and used when you run again. If you want to reset or clear the arguments, use:

```bash
(gdb) set args <arg1/arg2/.../or none to clear saved args>
```

If your program is already running, you can still use gdb to debug it.

```bash
$ gdb <name-of-executable> <process-id>
```

Or you can use the attach command:

```bash
$ gdb
(gdb) attach <process-id>
```

* Using breakpoints and watchpoints

```bash
(gdb) break <function-name> # set break point by function name
(gdb) break <line-number>
(gdb) break <file-name:line-number>
(gdb) break <virtual-memory-address>
(gdb) break <place-to-break> if <condition>
(gdb) info breakpoints	# list all all breakpoints and watchpoints
(gdb) disable n		# n is the breakpoint/watchpoint identifier number from listed info
(gdb) enable n
(gdb) clear <function-name>/<line-number>/<file-name:line-number>/<virtual-memory-address>
(gdb) delete <breakpoint-identifier>
(gdb) delete		# delete all breakpoints
(gdb) continue		# continue execution from breakpoint
```
    
* Manipulating variables

```bash
(gdb) ptype <var-name>	# print data type
(gdb) print <var-name>	# print data value
(gdb) print /FMT <var-name>	# o:octal,x:hex,d:decimal,u:unsigned decimal,t:binary, 	f:float,a:address,c:char
(gdb) set <var-name> = <new-value>
(gdb) print <var-name> = <new-value>
(gdb) display <var-name>/<expression>   # automatically display specified var once program is stopped
(gdb) info display          # print the list of auto-display items
(gdb) enable/disable <display-item-num> # enable or disable the display a var/exp
(gdb) undisplay             # delete the item from display list
```

* Stepping and resuming program

```bash
(gdb) step			# step by line
(gdb) istep			# step by instruction
(gdb) next			# continue to next line in current stack frame (including function)
(gdb) continue
(gdb) where			# show current line number
(gdb) list <none>/<line-num>/<range>/<function-name>/<*address>
```

* Checking stack and stack frames

```bash
(gdb) backtrace		# display the current stack status
(gdb) frame			# show current stack frame
(gdb) frame <frame-num>	# switch to frame <frame-num>
```