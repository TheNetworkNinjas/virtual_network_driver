# Virtual Network Driver for Linux

This project provides a simple, virtual network driver for the Linux kernel. It is primarily intended for educational purposes and demonstrates various concepts such as kernel module programming, network device operations, and ethtool operations.

### The Team
- [Craig Opie](https://craigopie.github.io/)
- Jake Imanaka
- Lydia Sollis

## Table of Contents
1. [Overview](#overview)
1. [Features](#features)
1. [Prerequisites](#prerequisites)
1. [Building](#building)
1. [Usage](#usage)
1. [Developer Information](#developer-information)
1. [Troubleshooting](#troubleshooting)

## Overview

This project provides a simple, virtual network driver for the Linux kernel. It is primarily intended for educational purposes and demonstrates various concepts such as kernel module programming, network device operations, and ethtool operations.

The virtual network driver can be loaded and unloaded using standard Linux kernel module utilities, such as `insmod` and `rmmod`. It also provides basic networking functionality, allowing users to interact with the virtual network device using standard network commands like `ifconfig` and `iwconfig`.

## Features

- Basic virtual network device implementation
- Network device open, close, and transmit operations
- IOCTL command handling
- Timer-based callbacks
- Virtual FIFO buffer for packet transmission

## Prerequisites

To compile a Loadable Kernel Module, you need the kernel headers matching the kernel version you are working with. To resolve this issue, install the kernel headers and development tools for your Linux distribution. For Fedora, you can use the following command:

```bash
sudo dnf install kernel-devel kernel-headers gcc make iw hostapd
```

## Building

To build the virtual network driver module, update the Makefile to use the correct path for the kernel build directory. The build directory is typically found under /lib/modules/$(uname -r)/build, where $(uname -r) returns the currently running kernel version.  Finally, run the make command from the project directory.

```bash
make
```

This will generate a kernel module file called `virt_net_driver.ko`.

## Usage

Load the cfg80211 kernel module:
```bash
sudo modprobe cfg80211
```

Load the virtual network driver module using `insmod`:

```bash
sudo insmod virt_net_driver.ko
```

To check if the module has been loaded successfully, run:

```bash
lsmod | grep virt_net_driver
```

Check if the virtual interfaces were created:
```bash
ifconfig
```

You should see three new interfaces like the example below:
```bash
vif0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        ether 00:76:69:66:30:00  txqueuelen 1000  (Ethernet)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

vif1: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        ether 00:76:69:66:31:00  txqueuelen 1000  (Ethernet)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

vif2: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        ether 00:76:69:66:32:00  txqueuelen 1000  (Ethernet)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
```

Check if the wifi devices have been properly configured. You should see the same interface names as the `ifonfig` output.

```bash
sudo iw dev
sudo iw list
```

Check if the `get_station` function works:

```bash
sudo iw dev vif0 station get 00:76:69:66:30:00
```

The output should look something like:

```bash
Station 00:76:69:66:30:00 (on vif0)
	rx bytes:	1
	rx packets:	1
	tx bytes:	1
	tx packets:	1
	tx failed:	1
	current time:	1682978276569 ms
```

To test if the scan and connect functions work properly, we must put each interface in their own network namespaces.

Add network namespaces:

```bash
sudo ip netns add v0
sudo ip netns add v1
sudo ip netns add v2
```
run `sudo iw dev` again and look and note the physical device numbers (`phy#{NUMBER}`). There should be three consecutive physical numbers (e.g. `phy#0 phy#1 phy#2`). Use these physical device numbers to add each device to its respective namespace:

```bash
sudo iw phy phy0 set netns name v0
sudo iw phy phy1 set netns name v1
sudo iw phy phy2 set netns name v2
```

Ensure all the interfaces are UP:

```bash
sudo ip netns exec v0 ip link set vif0 up
sudo ip netns exec v1 ip link set vif1 up
sudo ip netns exec v2 ip link set vif2 up
```

Configure vif0 to be an access point:

```bash
sudo ip netns exec v0 hostapd -B hostapd.conf
```

Check if vif0 has been successfully configured as an AP:

```bash
sudo ip netns exec v0 iw dev
```

The output should say `ssid VAP` and `type AP`:

```bash
phy#0
	Interface vif0
		ifindex 3
		wdev 0x1
		addr 00:76:69:66:30:00
		ssid VAP
		type AP
```

Give each interface an IP:

```bash
sudo ip netns exec v0 ip addr add 10.10.10.1/24 dev vif0
sudo ip netns exec v1 ip addr add 10.10.10.2/24 dev vif1
sudo ip netns exec v2 ip addr add 10.10.10.3/24 dev vif2
```

Perform a scan from vif1:

```bash
sudo ip netns exec v1 iw dev vif1 scan
```

Connect to vif0 from vif1:

```bash
sudo ip netns exec v1 iw dev vif1 connect VAP
```

Check vif1's link status:

```bash
sudo ip netns exec v1 iw dev vif1 link
```

Remove the virtual network driver module:

```bash
sudo rmmod virt_net_driver
```

Re-insert the module:

```bash
sudo insmod virt_net_driver.ko
```

Ping v2 from v1:

```bash
ping -I 10.10.10.2 10.10.10.1
```

## Developer Information

This project was developed and tested using Visual Studio Code on Fedora Core 38 with Linux Kernel 6.2.11.  To set up your Visual Studio Code environment for C/C++ development, debugging, and linting, follow these steps:

1. Install Visual Studio Code from https://code.visualstudio.com/.
1. Open Visual Studio Code and go to the Extensions view by clicking the square icon on the Activity Bar on the side of the window.
1. Search for and install the following extensions:
    1. `C/C++` (by Microsoft)
    1. `C/C++` Advanced Lint (by JBenden)
    1. `Native Debug` (by WebFreak)
1. Configure the C/C++ extension by creating a `c_cpp_properties.json` file in the `.vscode` directory of your workspace. Add the following content to the file, adjusting the `compilerPath` and `includePath` as needed:

```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/usr/lib/gcc/x86_64-linux-gnu/9/include"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c11",
            "cppStandard": "c++17",
            "intelliSenseMode": "gcc-x64"
        }
    ],
    "version": 4
}
```

5. Configure debugging for your project by creating a `launch.json` file in the `.vscode` directory. Add the following content to the file, adjusting the `program` and `miDebuggerPath` as needed:

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Debug Kernel Module",
            "type": "cppdbg",
            "request": "launch",
            "program": "/usr/src/linux-headers-$(uname -r)/scripts/gdb/vmlinux-gdb.py",
            "args": [],
            "stopAtEntry": false,
            "cwd": "${workspaceFolder}",
            "environment": [],
            "externalConsole": false,
            "MIMode": "gdb",
            "miDebuggerPath": "/usr/bin/gdb",
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                }
            ],
            "preLaunchTask": "loadModule"
        }
    ]
}
```

6. Configure the build and load module tasks by creating a `tasks.json` file in the `.vscode` directory. Add the following content to the file:

```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "build",
            "type": "shell",
            "command": "make",
            "group": {
                "kind": "build",
                "isDefault": true
            },
            "problemMatcher": []
        },
        {
            "label": "loadModule",
            "type": "shell",
            "command": "sudo insmod virt_net_driver.ko",
            "dependsOn": "build",
            "problemMatcher": []
        }
    ]
}
```

7. To build and load the kernel module, press `F5` in Visual Studio Code.

## Troubleshooting

If you encounter issues with the virtual network driver, follow these basic debugging steps:
1. Check the output of the `dmesg` command for any error messages or warnings related to the virtual network driver.
1. Use the `printk` function in the virtual network driver source code to output debug information to the kernel log. Make sure to recompile and reload the module after adding or modifying any `printk` statements.
1. Use the `modinfo` command to inspect the virtual network driver module and ensure that it was built with the correct kernel version:

```bash
modinfo virt_net_driver.ko
```

4. If you encounter build issues or other errors in the source code, use the linting and debugging features of Visual Studio Code to help identify and resolve the issues.
1. If the kernel module fails to load or unload, ensure that you are running the commands with the appropriate permissions (e.g., using `sudo`).
1. Consult the Linux kernel documentation and relevant online resources for guidance on how to implement and troubleshoot kernel modules and network drivers.
