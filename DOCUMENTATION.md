# Documentation

## Architecture

## Custom Modules

## Adaptes Modules

## Getting Started

This section will describe how to start, operate and use the TurtleBot project with step-by-step instructions.

### Preparing the TurtleBot hardware

To be able to run and start the TurtleBot, make sure the following components are either ready to use or nearby:
* TurtleBot 3 identified by red tape with `SteFloDom` written on it
* fully charged Li-Po battery
    * if necessary, a suitable Li-Po battery charger and additional Li-Po batteries
* at least one computer, laptop, or other device capable of running ROS
* a network that both the computer and TurtleBot can connect to
    * ***Note:*** The TurtleBot model in use only supports 2.4 GHz WiFi networks

If all of the required components are ready, proceed with the following steps:
1. Make sure the TurtleBot is currently turned off and no battery is connected
    * Should any battery already be connected, make sure to turn of the TurtleBot by switching of the Raspberry Pi and then disconnect the currently connected battery
2. Connect a fully charged Li-Po battery to the Li-Po battery extension cable, located at the bottom of the TurtleBot
3. Turn on the TurtleBot by switching on the Raspberry Pi module

Once the Raspberry Pi modules starts to light up, the rest of the TurtleBot (including the LiDAR sensor and the PixyCam) will start to boot up by either blinking or moving. Once the bootup has been completed (approximately 10-30 seconds), proceed with preparing the TurtleBot software.

### Preparing the TurtleBot software

#### First time setup

Should the TurtleBot get set up for the first time in a new network, proceed with the following steps. Otherwise skip and continue with the general setup.

##### Remote Computer
*The setup guide assumes that the operating system used is either a Linux distribution or that Windows Subsystem for Linux is used.*

1. Determine the IP address of the computer
2. Edit the `~/.bahsrc` file and set
    * `ROS_HOSTNAME` with value `<ip-address>`
    * `ROS_MASTER_URI` with value `http://<ip-addredd>:11311`

##### TurtleBot
1. Connect to the TurtleBot via SSH (`$ ssh ubuntu@<turtlebot-ip-address>`) with default-password `turtlebot`
    * ***Note:*** If the TurtleBot isn't already connected to the new WiFi network, connect a keyboard and monitor directly to the TurtleBot's Raspberry Pi module and add the network's SSID and password accordingly.
2. Determine the IP address of the TurtleBot
3. Edit the `~/.bahsrc` file and set
    * `ROS_HOSTNAME` with value `<turtlebot-ip-address>`
    * `ROS_MASTER_URI` with value `http://<remote-pc-ip-address>:11311`

Please also see the following image taken from `TurtleBot 3 Quick Start Guide (Noetic) - 3.1.5 Network Configuration`:
[![TurtleBot 3 Quick Start Guide - 3.1.5 Network Configuration](https://emanual.robotis.com/assets/images/platform/turtlebot3/software/network_configuration.png)](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start)

#### General setup

To be able to run the software built into the TurtleBot, proceed with the following steps:
1. Run `roscore` on the remote computer
2. Connect to the TurtleBot via SSH (`$ ssh ubuntu@<turtlebot-ip-address>`) with default-password `turtlebot`
3. Run
    ```console
    $ roslaunch turtlebot3_bringup turtlebot3_robot.launch
    ```
4. Run (in a new terminal session)
    ```console
    $ roslaunch pixy_node pixy_usb.launch 
    ```
5. Run (in a new terminal session)
    ```console
    $ roslaunch raspicam_node camerav2_1280x960.launch 
    ```

Once all five steps have been executed, the general setup is completed and the TurtleBot is ready.
