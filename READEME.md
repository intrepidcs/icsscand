This is the user mode daemon for Intrepid Control Systems' SocketCAN support. This daemon requires that ```intrepid.ko``` is loaded on your system. For instructions on building and loading the kernel object, see [intrepid-socketcan-kernel-module](https://github.com/intrepidcs/intrepid-socketcan-kernel-module). In addition, you will need a version of ```libicsneoapi```. For the purposes of SocketCAN support the open source version will suffice; see [libicsneoapi](https://github.com/intrepidcs/icsneoapi). 

To build, simply run ```make```.

```
$ make
```

To enable SocketCAN for your attached Intrepid devices, run the daemon. Use ```-D``` to run as a daemon, or pass no arguments to run in the foreground.

```
$ sudo ./icsscand -D
```

Most Intrepid devices support more than one CAN channel. Your interfaces will be named ```icsXcan_typeY``` where ```X``` is an incrementing identifier for different devices, ```can_type``` identifies the type of CAN channel, and ```Y``` is an incrementing identifier for each device. For example, the first neoVI FIRE you plug into your system will create the interfaces ```ics0can0```, ```ics0can1```, ```ics0can2```, ```ics0can3```, ```ics0lsftcan0```, ```ics0swcan0```.

Before you can read or write messages on an interface you'll need to bring it up

```
$ sudo ifconfig ics0can0 up
```
