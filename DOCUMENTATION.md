# Documentation

## Architecture

### Custom Modules

#### `token_detector` package

The `token_detector` package integrates all the necessary components to perform object detection and localization of tokens, specifically paper or post-it notes with a unique red color, within a labyrinth. The package includes the logic for navigating the labyrinth to identify the tokens and saving their positions within the map in a format compatible with other TurtleBot packages.

The `token_detector` package is organized into various nodes, each designed to carry out specific tasks within the labyrinth, either in an autonomous manner or to aid in the development of its associated functionalities:

##### `image_viewer` node

###### Purpose

The `image_viewer` node is a development tool subscribing to the `/raspicam_node/image/compressed` and `/camera/rgb/image_raw` topics, converting themk to the OpenCV format and subsequently displaying them in a window.

###### Usage

The `image_viewer` node can be launched with running the following command on the TurtleBot:

```console
$ roslaunch token_detector image_viewer.launch
```

##### `token_detector` node

###### Purpose

The `token_detector` node serves two main purposes: mapping the labyrinth in which the TurtleBot is placed, and detecting and saving the positions of tokens within the labyrinth. During the initial mapping phase, the labyrinth is explored using a left-wall following approach. The TurtleBot moves along the walls on its left until the entire labyrinth has been mapped. In the second phase, the TurtleBot traverses the labyrinth once again, detecting and recording the positions of any tokens encountered during its journey. These positions are stored in a format that can be easily reused.

###### Functional Principle

The `token_detector` node implements a reactive and behavior-based architecture, utilizing a subsumption architecture made up of multiple independent behaviors. This design enables the robot to efficiently explore its surroundings and gather tokens. The main node continuously determines which behavior is appropriate to execute based on the sensor data at any given time.

The `token_detector` consists of the following five behaviors (ordered descending by priority):
* `StepOntoToken`: The highest priority behavior responsible for stepping onto a detected token and claiming it.
* `CaptureToken`: The behavior is responsible for detecting and capturing tokens in the labyrinth.
* `MoveTowardsToken`: This behavior drives the robot towards a detected token.
* `WallFollower`: The behavior is responsible for following the labyrinth walls on the left hand side.
* `FindWall`: This behavior is responsible for finding a wall and orienting the robot towards it.

The main node additionally contains logic for creating an initial map of the labyrinth based on the two `WallFollower` and `FindWall` behaviors without paying attention to any existing tokens. Once the labyrinth has been fully mapped, the TurtleBot's set of behaviors is adjusted to include the remaining three behaviors which focus on finding and recording the positions of the tokens. Once all tokens (specified by the `num_tokens` parameter) have been discovered, the `token_detector`  --> TODO <--

###### Usage

The `token_detector` node can be launched with running the following command on the TurtleBot:

```console
$ roslaunch token_detector token_detector.launch num_tokens:=<token-number>
```

###### Arguments

| Argument         | Default | Format | Required | Description                                                 |
|------------------|---------|--------|----------|-------------------------------------------------------------|
| `debug`          | `false` | `bool` | No       | Show debug messages                                         |
| `skip_roundtrip` | `false` | `bool` | No       | Skip the initial token-ignoring roundtrip, if set to `true` |
| `num_tokens`     | N/A     | `int`  | Yes      | Number of tokens present in the labyrinth                   |

#### `current_pos` package

##### Purpose

The `current_pos` package contains the necessary files and logic for converting the TurtleBot's position, as determined from Odometry data, to a coordinate representation within the map's coordinate system.

##### Functional Principle

The node subscribes to the `/odom` topic, which provides the current pose of the robot. Subsequently, the node's logic is triggered every time a new pose is received on the topic nd converts it to the map frame using functionality provided by the [ROS `tf` package](https://wiki.ros.org/tf).

The provided `lookupTransoform` method is used to get the position and orientation of the robot in the map frame. The position is stored as an x and y coordinate, while the orientation is stored as a yaw angle (representing rotation around the z-axis; calculated using the provided `euler_from_quaternion` method).  
The converted pose is packaged into a custom `PoseInMap` message, which includes the x and y position and the yaw angle. This message is then combined with the original `/odom` pose into a custom `PoseTF` message, which includes a header with a sequence number and timestamp, the original `/odom` pose, and the converted map pose.

Both the `PoseInMap` and `PoseTF` messages are used in other packages related to the TurtleBot's functionality.

##### Usage

The `robot2map_conversion` node can be launched with running the following command on the TurtleBot:

```console
$ roslaunch current_pos launch_transformer.launch
```

##### Arguments

| Argument     | Default | Format   | Required | Description                                                          |
|--------------|---------|----------|----------|----------------------------------------------------------------------|
| `debug`      | `true`  | `bool`   | No       | Show debug messages                                                  |

#### `token_inspector` package

TODO

#### `amcl_loclization` package

TODO

### Adapted Modules

TODO - Adapted Modules?

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
    * `ROS_MASTER_URI` with value `http://<ip-address>:11311`
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
    $ roslaunch raspicam_node camerav2_410x308.launch 
    ```

Once all five steps have been executed, the general setup is completed and the TurtleBot is ready.

### Using the TurtleBot

TODO

## Troubleshooting

The TurtleBot is comprised of various hardware and software components, which can experience issues due to external factors, incorrect installation, improper execution sequence, or other causes. Some problems can easily be fixed, while others may require more attention and a technical understanding.

The following list includes frequently encountered problems and respective solutions for hardware and software components in the TurtleBot.

### The TurtleBot is turned on but the LiDAR is not moving

In some cases, the LiDAR may not be operating even if the TurtleBot has been correctly turned on and the setup steps have been followed correctly. To resolve the issue:

**Solution 1:** It's possible that the `roslaunch turtlebot3_bringup turtlebot3_robot.launch` script has crashed or lost connection to the ROS Master. Connect to the TurtleBot using SSH, stop the script and restart it.

**Solution 2:** If the `roscore` script has crashed or lost connection to the network, connect to the remote computer, stop the script and restart it. Then, follow the steps of `Solution 1`.

### The TurtleBot is turned on but the wheels are not moving

In some instances, the Li-Po battery may have a low charge that isn't low enough to trigger the low battery beep, but it still may not be enough to power the TurtleBot's wheels.

**Solution:** To resolve this, turn off the TurtleBot by turning off the Raspberry Pi module. Wait for 10-20 seconds and then disconnect the Li-Po battery. Finally, reconnect a fully charged Li-Po battery.  
Refer to `General setup` for further instructions.

### An error message indicating out-of-sync timestamps appears

In some cases, an error message indicating out-of-sync timestamps may appear, such as:
```
For frame [base_scan]: No transform to fixed frame [map].  
TF error: [Lookup would require extrapolation -0,522100208s into the future.  Requested time 1666254110,220566034 but the latest data is at time 1666254109,698465824, when looking up transform from frame [base_scan] to frame [map]]
```
or
```
Costmap2DROS transform timeout. Current time: 1666254261.9980, global_pose stamp: 1666254261.2392, tolerance: 0.5000 
```

**Solution:** To resolve this issue, synchronize the time on the Raspberry Pi and the remote computer with a NTP time server.

### The `roslaunch turtlebot3_bringup turtlebot3_robot.launch` script is encountering an error

In some instances, the `roslaunch turtlebot3_bringup turtlebot3_robot.launch` script may display the error message `Creation of publisher failed: Checksum does not match` related to `sensor_msgs/JointState`. This can result in the TurtleBot being unable to navigate using Rviz, SLAM or other methods.

**Solution:** To resolve this issue, try reflashing the OpenCR firmware following [these instruction](https://emanual.robotis.com/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup).

### The TurtleBot is turned on, but the LiDAR is not functioning properly and/or keeps shutting down randomly.

In some rare cases, the LiDAR may not return any sensor data and/or shut down unexpectedly, despite attempts to resolve the issue through other solutions.

**Solution 1:** Verify that the LiDAR is securely attached to the TurtleBot's top platform and all its ports are properly connected.

**Solution 2:** Consider replacing the malfunctioning LiDAR with a different, functional LiDAR module to determine if the issue is due to a faulty LiDAR.
