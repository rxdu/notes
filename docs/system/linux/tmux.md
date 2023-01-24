# Tmux Reference

tmux provides two main functions: window management in the terminal and session management. Around the two functions, there are a few core concepts you need to understand:

* A session contains a group of windows
    - A window contains one or more panels
        - A panel contains a terminal where you can run programs inside

You can detatch from a session and re-attach to the same session again later so that you can resume your previous work. You can find the session/window/panel information from the status line, as shown in the following diagram[1].

![tmux status line](./figures/tmux_status_line_diagram.png)

Most tmux commands are carried out with the prefix key "C-b", which means you keep "Ctrl" and "B" key pressed together. Shortly after releasing the two keys, press another key for a specific command. For example, with **"C-b" + "%"**, you can split a panel into a left and right panels. 

## Setup tmux

* Install tmux

```bash
$ sudo apt install tmux
```

* Enable mouse support

```bash
$ tmux set-option mouse on
```

## Session Management

* Create a new named session

```bash
$ tmux new [-t] -s mysession
```

With the "-t" argument the new session is detached automatically after creation.

* Detach from the current session: **C-b d**

* List existing sessions

```bash
$ tmux ls
```

* Attach to a session

```bash
# attach to session with id
$ tmux attach -t 0
# attach to a named session
$ tmux attach -t mysession
```

* Rename an existing session

```bash
$ tmux rename-session -t 0 mysession2
```

## Window Management

* Create a new window: **C-b c**
* Rename the current window: **C-b ,**
* Change to window 0: **C-b 0**
* Change to next window: **C-b n**
* Change to previous window: **C-b p**
* Change to the last window: **C-b l**

* Create a new named window

```bash
$ tmux new-window [-d] -n mynewwindow
```

If the argument "-d" is specified, tmux will only create the new window but does not make the new window to be the current window.

## Panel Management

* Splits the current panel into two horizontally: **C-b %**
* Splits the current panel into two vertically: **C-b "**
* Switch between the panels: **C-b <arrow-key\>**
* Resize panel in direction of arrow key: **C-b C-<arrow-key\>**
* Toggle full-screen mode of the current panel: **C-b z**

## Session and Window in Tree View

* Show sessions: **C-b s**
* Show windows of current session: **C-b w**
* Exit tree view mode: **q**

## Reference

* [1] https://github.com/tmux/tmux/wiki/Getting-Started
* [2] https://man7.org/linux/man-pages/man1/tmux.1.html
* [3] https://www.hamvocke.com/blog/a-quick-and-easy-guide-to-tmux/