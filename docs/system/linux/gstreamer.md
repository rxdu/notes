# GStreamer

"GStreamer is a pipeline-based multimedia framework that links together a wide variety of media processing systems to complete complex workflows."[1]

## Installation

On Ubuntu or Debian

```bash
$ sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio
```

## GStreamer Tools

You can use "gst-launch-1.0" to quickly test pipelines before trying to implement your application using the gstreamer API calls.

### gst-launch-1.0

#### Command Pattern

A command with "gst-launch-1.0" generally has the following form:

```bash
$ gst-launch-1.0 element1 [property=value1] ! element2 [property=value2]
```

You could also name an element so that you can refer to it later

```bash
$ gst-launch-1.0 element1 [name=xyz] ! element2 xyz. ! element3
```

#### Sample Usages

* Display a test pattern
  
```bash
# video test src
$ gst-launch-1.0 videotestsrc ! videoconvert ! autovideosink
# audio test src
$ gst-launch-1.0 audiotestsrc ! autoaudiosink
# combine both video and audio 
$ gst-launch-1.0 audiotestsrc ! autoaudiosink videotestsrc ! autovideosink
```

* Play video from a file

```bash 
$ gst-launch-1.0 playbin uri=https://gstreamer.freedesktop.org/data/media/sintel_trailer-480p.webm
```

* Play video from a USB camera (v4l2)

```bash
$ gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! autovideosink
```

* Play video from a RTSP stream

```bash
$ gst-launch-1.0 rtspsrc location=rtsp://<rtsp-address> latency=0 buffer-mode=auto ! decodebin ! autovideosink
```

### gst-inspect-1.0

This tool is to inspect details of gstreamer plugins/elements

```bash
# list all available elements
$ gst-inspect-1.0
# list information regarding an element
$ gst-inspect-1.0 vp8dec
```

### gst-discoverer-1.0

This tool can be used to list information regarding the media that GStreamer can extract

```bash
$ gst-discoverer-1.0 https://gstreamer.freedesktop.org/data/media/sintel_trailer-480p.webm -v
```

## Reference

* [1] https://en.wikipedia.org/wiki/GStreamer
* [2] https://gstreamer.freedesktop.org/documentation/tutorials/basic/gstreamer-tools.html?gi-language=c
* [3] https://gstreamer.freedesktop.org/documentation/tools/gst-launch.html?gi-language=c
* [4] https://github.com/matthew1000/gstreamer-cheat-sheet/tree/master
* [5] https://www.wowza.com/blog/streaming-protocols