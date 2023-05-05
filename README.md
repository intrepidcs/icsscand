Version 3.1.0

This is the usermode daemon for the Intrepid Control Systems SocketCAN support. This daemon requires that ```intrepid.ko``` is loaded on your system.

1. Build and load the kernel module follwowing the instructions in [intrepid-socketcan-kernel-module](https://github.com/intrepidcs/intrepid-socketcan-kernel-module).

2. Install the dependencies needed. These are CMake 3.2+, GCC 4.8+, git, libusb-1.0-0-dev, and libpcap.

On Ubuntu or other Debian-based systems, run `sudo apt install git cmake gcc libusb-1.0-0-dev libpcap-dev build-essential`.

3. Clone this repository recursively by running `git clone --recursive https://github.com/intrepidcs/icsscand.git`

4. Switch into the cloned directory, `cd icsscand`

5. Make a build directory and switch into it, `mkdir build && cd build`

6. Invoke CMake, `cmake .. -DCMAKE_BUILD_TYPE=Release`

7. Build the daemon, `make`

8. The daemon is now available as `libicsneo-socketcan-daemon`

9. Start the daemon up to begin connecting to devices. Use `sudo ./libicsneo-socketcan-daemon`. If you're happy with the results and would like to run in the background, run `sudo ./libicsneo-socketcan-daemon -d` to run in daemon mode.

10. CAN interfaces will have been created, but are "down" or, in other words, not enabled for transmit and receive yet. You can see them with `ip link`. They will be labelled `can0`, `can1`, and etc. They will have an alias listed which corresponds to the serial number of the device and network on that device.

11. Enable the CAN interface with `sudo ip link set can0 up`, replacing `can0` with whichever interface you'd like to enable

12. You can now use any SocketCAN application with this interface. A good package for testing is the `can-utils` package. You can get this package with `sudo apt install can-utils`. A good testing tool which comes with this package is `candump`. Running `candump can0` will print a line for every incoming frame.
