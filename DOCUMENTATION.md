# Documentation

## Architecture

## Custom Modules

### `token_detector`

### `current_pos`

#### Purpose

The `current_pos` package contains all the necessary files and logic to convert the TurtleBot's position, based on the Odometry data, to a position/to coordinates within the map's coordinate system.

#### Functional Principle

The node subscribes to the `/odom` topic, which provides the current pose of the robot. Subsequently, the node's logic is triggered every time a new pose is received on the topic nd converts it to the map frame using functionality provided by the [ROS `tf` package](https://wiki.ros.org/tf).

The provided `lookupTransoform` method is used to get the position and orientation of the robot in the map frame. The position is stored as an x and y coordinate, while the orientation is stored as a yaw angle (representing rotation around the z-axis; calculated using the provided `euler_from_quaternion` method).  
The converted pose is packaged into a custom `PoseInMap` message, which includes the x and y position and the yaw angle. This message is then combined with the original `/odom` pose into a custom `PoseTF` message, which includes a header with a sequence number and timestamp, the original `/odom` pose, and the converted map pose.

Both the `PoseInMap` and `PoseTF` messages are used in other packages related to the TurtleBot's functionality.

#### Usage

The `robot2map_conversion` node can be launched with running the following command on the TurtleBot:

```console
$ roslaunch current_pos launch_transformer.launch
```

##### Parameters

| Parameter    | Default | Format   | Required | Description                                                          |
|--------------|---------|----------|----------|----------------------------------------------------------------------|
| `debug`      | `true`  | `bool`   | No       | Show debug messages                                                  |

### `token_inspector`

### `amcl_loclization`

## Adapted Modules

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
3. Run
    ```console
    $ source ~/.bashrc
    ```

##### TurtleBot
1. Connect to the TurtleBot via SSH (`$ ssh ubuntu@<turtlebot-ip-address>`) with default-password `turtlebot`
    * ***Note:*** If the TurtleBot isn't already connected to the new WiFi network, connect a keyboard and monitor directly to the TurtleBot's Raspberry Pi module and add the network's SSID and password accordingly.
2. Determine the IP address of the TurtleBot
3. Edit the `~/.bahsrc` file and set
    * `ROS_HOSTNAME` with value `<turtlebot-ip-address>`
    * `ROS_MASTER_URI` with value `http://<remote-pc-ip-address>:11311`
3. Run
    ```console
    $ source ~/.bashrc
    ```

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
