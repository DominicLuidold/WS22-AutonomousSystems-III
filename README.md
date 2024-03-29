# Documentation

<img src="documentation/TurtleBot3_SteFloDom.jpg" alt="Image of the `SteFloDom` TurtleBot 3" style="height: 500px" />

## Table of Contents

1. [Getting Started](#getting-started)
    1. [Preparing the TurtleBot hardware](#preparing-the-turtlebot-hardware)
    1. [Preparing the TurtleBot software](#preparing-the-turtlebot-software)
    1. [Using the TurtleBot](#using-the-turtlebot)
1. [Architecture](#architecture)
    1. [Custom Modules](#custom-modules)
        1. [`token_detector` package](#token_detector-package)
        1. [`current_pos` package](#current_pos-package)
        1. [`token_inspector` package](#token_inspector-package)
    1. [Adapted Modules](#adapted-modules)
        1. [Raspberry Pi camera configuration](#raspberry-pi-camera-configuration)
    1. [The proposed ideas for communicating with other TurtleBots](#the-proposed-ideas-for-communicating-with-other-turtlebots)
        1. [Solution 1: Communicating found tags](#solution-1-communicating-found-tags)
        1. [Solution 2: Communicating next and found tags](#solution-2-communicating-next-and-found-tags)
        1. [Solution 3: Constant monitoring](#solution-3-constant-monitoring)
1. [Troubleshooting](#troubleshooting)
1. [*Internal* development documentation](#internal-development-documentation)

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
* the Raspicam mounted at the top facing forward as shown in the picture.
* the Pixy CMUcam5 mounted at the front facing downwards as shown in the picture.

If all of the required components are ready, proceed with the following steps:
1. Make sure the TurtleBot is currently turned off and no battery is connected
    * Should any battery already be connected, make sure to turn off the TurtleBot by switching off the Raspberry Pi and then disconnect the currently connected battery
2. Connect a fully charged Li-Po battery to the Li-Po battery extension cable, located at the bottom of the TurtleBot
3. Turn on the TurtleBot by switching on the Raspberry Pi module

Once the Raspberry Pi modules start to light up, the rest of the TurtleBot (including the LiDAR sensor and the PixyCam) will start to boot up by either blinking or moving. Once the bootup has been completed (approximately 10-30 seconds), proceed with preparing the TurtleBot software.

### Preparing the TurtleBot software

#### First time setup

Should the TurtleBot get set up for the first time in a new network, proceed with the following steps. Otherwise skip and continue with the [General Setup](#general-setup).

##### Remote Computer
*The setup guide assumes that the operating system used is either a Linux distribution or that a Linux VM running on Windows is used, you either use the commands in the linux terminal or connect to the system via ssh.*

1. Determine the IP address of the computer
2. Edit the `~/.bahsrc` file and set
    * `ROS_HOSTNAME` with value `<ip-address>` (we used `$(ip route get 8.8.8.8 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')` because of changing networks)
    * `ROS_MASTER_URI` with value `http://<ip-address>:11311`
3. Run
    ```console
    $ source ~/.bashrc
    ```
4. Run
    ```console
    $ sudo mkdir /killerrobot && sudo chmod 766 /killerrobot
    ```

#### TurtleBot
1. Connect to the TurtleBot via SSH (`$ ssh ubuntu@<turtlebot-ip-address>`) with default-password `turtlebot`
    * ***Note:*** If the TurtleBot isn't already connected to the new WiFi network, connect a keyboard and monitor directly to the TurtleBot's Raspberry Pi module and add the network's SSID and password accordingly.
2. Determine the IP address of the TurtleBot
3. Edit the `~/.bahsrc` file and set
    * `ROS_HOSTNAME` with value `<turtlebot-ip-address>` (we used `$(ip route get 8.8.8.8 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')` because of changing networks)
    * `ROS_MASTER_URI` with value `http://<remote-pc-ip-address>:11311`
4. Run
    ```console
    $ source ~/.bashrc
    ```

Please also see the following image taken from [`TurtleBot 3 Quick Start Guide (Noetic) - 3.1.5 Network Configuration`](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start):
[![TurtleBot 3 Quick Start Guide - 3.1.5 Network Configuration](https://emanual.robotis.com/assets/images/platform/turtlebot3/software/network_configuration.png)](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start)

#### General Setup

##### Raspicam
To be able to utilize the image stream provided by the raspicam it needs to be set up on the turtlebot first. Refer to [this tutorial](#raspicam-setup).

##### PixyCam

After the PixyCam has been set up for use via usb (see [tutorial](#pixy-cmucam5-setup)) it needs to be trained.
To be able to detect any tokens using the Pixy CMUcam5, the camera has to be trained every time before being able to start running the built-in software. To do so, proceed with the following steps:
1. Make sure the color indication LED is covered with a piece of tape or similar to avoid reflections. As the PixyCam is placed at the bottom facing downwards, the camera may be able to see the reflections of the light in the ground and therefore recognize it wrongfully.
2. Place the PixyCam over a red token.
3. Press and hold the button until the LED turns white (it will switch to all colors after 1 second), then release when it turns red.
4. Move the camera over a the token. (results are best when real conditions are reproduced - put the turtlebot on the ground wth the pixycam directly over a token)
    * ***Note:*** Stable and bright lighting conditions lead to the best results. If the turtlebot casts a little shadow, it does no harm however. It is no different in real situations.
6. Press the button once.
7. Now the LED blinks for a second or two and then shines red if a token is detected or stays off if none is in sight.
7. Make sure that tokens are detected correctly by testing a few of them: Hovering over a token makes the LED turn red (can be seen from above through little holes) and moving away the PixyCam turns off the LED. Make sure that the light is steady and not flickering, otherwise repeat the process.

 ***Note:*** See the [Pixy Documentation](https://docs.pixycam.com/wiki/doku.php?id=wiki:v1:teach_pixy_an_object_2) for more information. Also, the lens sharpness can be adjusted by turning the camera housing, but too much or too little adjustment may affect image quality. Use the PixyMon tool for the initial calibration!

##### Software

To be able to run the software built into the TurtleBot, proceed with the following steps:
1. On the remote computer, run
    ```console
    $ roscore
    ```
    to start the ROS master.
2. Connect to the TurtleBot via SSH (`$ ssh ubuntu@<turtlebot-ip-address>`) with default-password `turtlebot`
3. Run the following command on the TurtleBot to start basic packages and applications like movement controls or the laser scan.
    ```console
    $ roslaunch turtlebot3_bringup turtlebot3_robot.launch
    ```
4. Run the following command in a new terminal session on the TurtleBot to publish found pixy signatures to the ```/my_pixy/block_data``` topic
    ```console
    $ roslaunch pixy_node pixy_usb.launch 
    ```
5. Run the following command in a new terminal session on the TurtleBot to start publishing image information on the ```/raspicam_node/image/compressed topic``` among others.
    ```console
    $ roslaunch raspicam_node camerav2_custom.launch
    ```

Once all steps have been executed, the general setup is completed and the TurtleBot is ready.

### Using the TurtleBot

To start using the TurtleBot, proceed with the following steps:
1. Create a labyrinth
    * ***Note:*** For optimal results, use labyrinth pieces from room `U131 Lab. Auton. Systeme` of Vorarlberg UAS with a minimum labyrinth width of one piece. Tokens *must* be spaced at least 20cm apart and not closer to the wall than 15cm. So called "Islands" do not work and must not be set up.
2. Follow the steps described in the [*General Setup*](#general-setup) chapter.
3. Have the remote computer with the compiled source code and packages ready.

#### Phase 1 - Map labyrinth and detect tokens

1. On the remote computer, run (in a new terminal)
    ```console
    $ roslaunch token_detector custom_slam.launch
    ```
    to start turtlebot navigation package with slam, gmapping, rviz and our custom currentpos positioning package.
2. On the remote computer, run (in a new terminal)
    ```console
    $ roslaunch token_detector token_detector.launch num_tokens:=<number-of-tokens>
    ```
    to make the turtlebot begin its search for the tokens.

After the TurtleBot has completed the first round trip, the following message will be displayed in the terminal:
```
Initial roundtrip complete! Switched to different behaviors
```

The TurtleBot will then start following the left-hand side wall of the labyrinth, detecting all tokens and saving their positions in a reusable format.

***Note:*** Once all tokens have been detected, the TurtleBot will automatically stop. When the TurtleBot has fully stopped for 10-20 seconds, stop the script by entering `CMD+C`.

#### Phase 2 - Localize robot, plan paths & drive to tokens

1. On the remote computer, run (in a new terminal):
    ```console
    $ roslaunch token_inspector custom_navigation.launch map_file:=<path-to-map-file>
    ```
    to start the map server, amcl localization, move_base package as well as rviz to display map, costmaps and path information.
2. On the remote computer, run (in a new terminal) 
    ```console
    $ roslaunch current_pos launch_transformer.launch
    ```
    to start publishing information about the current estimated position of the roboter to the ```pose_tf``` topic.
3. On the remote computer, run (in a new terminal)
    ```console
    $ roslaunch token_inspector find_path.launch
    ```
    to start the pathfinder node calculating paths and path lengths to given targets.
4. On the remote computer, run (in a new terminal)
    ```console
    $ roslaunch token_inspector scheduler.launch token_file:=<path-to-token-file>
    ```
    to start the scheduler node who keeps track of the visited and remaining tokens.
5. On the remote computer, run (in a new terminal)
    ```console
    $ roslaunch token_inspector token_inspector.launch
    ```
    to start the token_inspector node navigating the turtlebot through the labyrinth visiting the tokens one by one.
    

At first the TurtleBot will start to locate itself within the labyrinth by following the wall, regardless of its starting position, and then continue to plan the path and drive to each token in turn.
When the TurtleBot has finished, the turtlebot will stop and the following message will appear in the terminal:
```
Finished
```

## Architecture

The task is divided into two phases, reflecting the structure of our solution which consists of two main parts, both of which require user input to initiate and complete. The two parts work together to allow the TurtleBot to navigate through any given labyrinth (that meets the minimum requirements mentioned in [*Using the TurtleBot*](#using-the-turtlebot)), detect tokens, and ultimately plan a path through the labyrinth to reach each token.

Phase 1, using the `token_detector` package, is the first step in this process. Its primary function is to generate a map of the labyrinth by following the wall on the left and making a full round trip through the labyrinth. During this phase, the TurtleBot travels through the labyrinth and collects LiDAR data about its surroundings, using this information to construct a map of the labyrinth. This map is then stored for future use. The first round trip of Phase 1 is important to ensure that a correct map of the given labyrinth is generated and saved before proceeding with any further steps, as this will be used as the basis for all further operations. Detecting tokens while creating a map would have been possible, but accurately transforming the location based on the TurtleBot's coordinates in an incomplete map would have been a more difficult and error-prone approach.  
Once the map is generated, Phase 1 continues with the TurtleBot driving through the labyrinth again, this time with the goal of detecting tokens and storing their location based on the generated map. The TurtleBot uses the wall following approach to navigate and uses its token detection algorithm (see [*`token_detector` - Functional Principle*](#functional-principle) for more details) to identify each token it encounters and store its location. It will continue to do this until it has found the number of tokens specified by the user.

Phase 2 of the process involves the use of the `token_inspector` package. Its primary goal is to locate the TurtleBot within the labyrinth and plan a path to collect each token in turn. In the first step, the TurtleBot uses the `Advanced Monte Carlo Localisation` algorithm (see [*`inspector_node` - AMCL*](#inspector-node) for more details) to determine its location within the labyrinth. This allows it to start from any point within the labyrinth, effectively solving the kidnapped robot problem.  
Once the TurtleBot has located itself, it uses a goal scheduling algorithm based on the ROS service server-client architecture to obtain new token goals. This algorithm relies on a main runner and a backup runner (see [*`inspector_node`*](#inspector-node) for more details), which provide different strategies for reaching each token with a planned path based on the shortest distances between them. Finally, the TurtleBot runs through the labyrinth until it has collected all the tokens.

### Custom Modules

#### `token_detector` package

The `token_detector` package integrates all the necessary components to map the world using slam and perform detection and localization of tokens, specifically paper or post-it notes with a unique red color, within a labyrinth. The package includes the logic for navigating the labyrinth, to identify the tokens and saving their positions within the map in JSON format. Additionally, the map is saved in a ROS standard `.yaml` and `.pgm` format.

The `token_detector` package is organized into two nodes and various behaviors, each designed to carry out specific tasks within the labyrinth, either in an autonomous manner or to aid in the development of its associated functionalities:

##### `image_viewer` node

###### Purpose

The `image_viewer` node is a development tool subscribing to the `/raspicam_node/image/compressed` and `/camera/rgb/image_raw` (for simulating in Gazebo) topics, converting them to the OpenCV format and subsequently displaying them in a window.

###### Usage

The `image_viewer` node can be launched with running the following command on the TurtleBot:

```console
$ roslaunch token_detector image_viewer.launch
```

##### `token_detector` node

###### Purpose

The `token_detector` node serves two main purposes: mapping the labyrinth in which the TurtleBot is placed, and detecting and saving the positions of tokens within the labyrinth. During the initial mapping phase, the labyrinth is explored using a left-wall following approach. The TurtleBot moves along the walls on its left until the entire labyrinth has been mapped. In the second phase, the TurtleBot traverses the labyrinth once again, detecting and recording the positions of any tokens encountered during its journey. These positions are stored in a locally saved JSON file for reusability (for a more detailed explanation, refer to *`CaptureToken` behavior description*).

###### Functional Principle

The `token_detector` node implements a reactive and behavior-based architecture, utilizing a subsumption architecture made up of multiple independent behaviors. This design enables the robot to efficiently explore its surroundings and gather tokens. The main node continuously determines which behavior is appropriate to execute based on the sensor data at any given time.

The `token_detector` consists of the following five behaviors (ordered descending by priority):
* `StepOntoToken`: The highest priority behavior responsible for stepping onto a detected token.
* `CaptureToken`: The behavior is responsible for capturing tokens in the labyrinth, saving their location.
* `MoveTowardsToken`: This behavior drives the robot towards a detected token.
* `WallFollower`: The behavior is responsible for following the labyrinth walls on the left hand side.
* `FindWall`: This behavior is responsible for finding a wall and orienting the robot towards it.

The specific priority of the behaviors has been chosen to ensure that reacting to a token has the highest priority, as this is one of the core functionalities and requirements of the TurtleBot's software. Following and finding a wall have been given the lowest priority, as finding and following a wall is always possible, whereas missing a token may result in significantly longer runtime.

<details>
<summary><code>StepOntoToken</code> behavior description (click on the arrow to expand the section)</summary>

The `StepOntoToken` behavior is designed to enable the robot to move onto a token within the labyrinth by utilizing the PixyCam. This behavior can only be executed when the PixyCam is detecting a token and when the roboter's location in the map is not too close to any previously recognized tokens. The purpose of this is to prevent the detection of the same token multiple times. There is a tolerance of 20 centimeters, because especially when the robot approaches the token from different angles, the exact location of the roboter varies when it is just at the edge of the token at the beginning. If both conditions are met, the behavior is executed and the robot moves onto and over the token at a noticably slower and steady speed.

To be able to detect when the PixyCam is recognizing a token, the custom `PixycamDetector` class is used which relies on the `/my_pixy/block_data` topic provided by the [pixy_ros package](https://github.com/jeskesen/pixy_ros). It appears to be default behavior that for every three `block_data` messages received, an empty message is included. To ensure that the TurtleBot is not mistakenly on top of a token, a counter for consecutive non-detections has been implemented.

</details>

<details>
<summary><code>CaptureToken</code> behavior description</summary>

This behavior is applicable once the `StepOntoToken` behavior is not executed any more and only executed once per token. Once the `StepOntoToken` behavior has confirmed that the TurtleBot is located on a token, the behavior waits for the custom `PoseTF` (see [*`current_pos` package*](#current_pos-package)) message published on the `/pose_tf` topic which represents the position of the token transformed into the map frame.

The behavior then checks if there is already a registered token within a distance of `20 cm` from the current TurtleBot position. If not, the captured token is registered, assigned a `tagno` and the information (in form of `tagno`, `x`, `y` and `angle`) is saved in a JSON file located at `/killerrobot/token_positions.json` every time a new token is detected.

</details>

<details>
<summary><code>MoveTowardsToken</code> behavior description</summary>

The `MoveTowardsToken` behavior utilizes both the Raspberry Pi camera and sensor data from the LiDAR (obtained by subscribing to the `/scan` topic). This behavior is triggered when the system detects a token that is within reach and has not yet been registered. Upon verifying that the token is new, the behavior uses the available information to pinpoint its location on the map and directs the TurtleBot to move towards it, gradually slowing down as it gets closer to the token.

To be able to detect tokens using the Raspberry Pi camera, the custom `RaspicamDetector` class is used. This class subscribes to the `/raspicam_node/image/compressed` topic and uses the [`cv_bridge` package](https://wiki.ros.org/cv_bridge) to convert incoming images to a format that OpenCV can process.  
Once the image has been converted to OpenCV format, the following steps are performed:
1. Convert the image from BGR to HSV color space
2. Create a mask that highlights the red color in the image using the lower and upper bounds for red color in the HSV color space
3. Remove noise from the mask using morphological operations
4. Find contours in the mask
5. Calculate the centers of the contours, sort them based on the estimated distance from the camera, and set the angle of each token relative to the camera

The location of a token within the map is determined using a custom camera point grid, which assigns a map coordinate (a tuple of `x` and `y` values) to a specific pixel range (also a tuple of `x` and `y` values). For further information, please refer to the footnote and the *Camera Point Grid for `410x308` Pixels Resolution* section below.[^cam-point-grid]  
To estimate the position of the token on the map, the closest grid point to the token is calculated in pixels using the euclidean distance. Knowing how many centimeters apart each grid point is, we created a table of mappings between number of pixels apart and number of centimeters apart (1cm = 0.01 in map coordinates). The estimated distance to the token in map coordinates as well as its angle in relation to the robot itself is then added to the current rotation of the robot and subsequently a position can be estimated within the map. This estimated position is then used to determine (with a certain tolerance) if this token has already been captured.

Camera Point Grid for <code>410x308</code> Pixels Resolution

<img src="documentation/camera_point_grid.jpg" alt="Camera Point Grid for `410x308` Pixels Resolution" style="height: 700px" />


To know whether a token has already been registered, the `any_registered_token_within_distance` helper function is used that takes an estimated token map pose, a list of registered tokens and a distance threshold as input and returns information on wether there is any already registered token that is closer in Euclidean distance to the estimated token than the given distance threshold.

The `MoveTowardsToken` behavior includes parts of the `WallFollower` logic to prevent driving too close or crashing into a wall while steering towards a detected token. For improved accuracy, instead of just the front, front-left and left surroundings, also their pendants on the right side are utilized to prevent crashes on both sides.

</details>

<details>
<summary><code>WallFollower</code> behavior description</summary>

The `WallFollower` behavior is internally controlled by two different classes, namely `TurnTowardsWall` and `FollowWall`, as well as a `RoundtripMonitor` class responsible for switching behaviors once the TurtleBot has completed the first round trip.

The `TurnTowardsWall` logic turns towards a wall if the wall is not in front of the robot but on its left. This is determined by checking the distance to the wall in front, front_left and left directions using the LiDAR data published to the `/scan` topic. If the wall is on its left, the robot turns left towards the wall. The speed of the turn is determined by the distance to the wall. The closer the robot is to the wall, the slower the turn.

The `FollowWall` logic instructs the TurtleBot to move forward and turn slightly left to keep the wall on its left side. The speed of the robot and the degree of the turn is determined by the distance to the wall in front, front_left, and left directions. The closer the robot is to the wall, the slower it moves and the sharper the turn. Comparable to the `TurnTowardsWall` logic, the `/scan` topic is used to get the LiDAR data.

The `RoundtripMonitor` monitors if the robot has completed a roundtrip by keeping track of the initial pose (using the custom `PoseTF` message (see [`current_pos` package](#current_pos-package)) of the robot when it first made contact with the wall, and checking if the robot is near the same pose after leaving the area. If the robot is near the same pose again, it is considered to have completed a roundtrip.  
To determine whether the current position is near the starting area of the roundtrip, the euclidean distance is used as a measure based on the `x` and `y` coordinates of the TurtleBot.

</details>

<details>
<summary><code>FindWall</code> behavior description</summary>

The `FindWall` behavior has the lowest priority among the five available behaviors and is only executed when none of the other behaviors are applicable. When this behavior is triggered, the TurtleBot uses the `/cmd_vel` topic to move with a slight left curve ahead until it reaches a wall, at which point th control is transferred to another behavior.

</details>

The main node contains the logic for creating an initial map of the labyrinth based on the two `WallFollower` and `FindWall` behaviors without paying attention to any existing tokens. Once the labyrinth has been fully mapped, the TurtleBot's set of behaviors is adjusted to include the remaining three behaviors which focus on finding and recording the positions of the tokens. Once all tokens (specified by the `num_tokens` parameter) have been discovered, the `token_detector`  tells the TurtleBot to stop. Then the turtlebot awaits user interaction.

###### Usage

The `token_detector` node can be launched with running the following command on the remote computer:

```console
$ roslaunch token_detector token_detector.launch num_tokens:=<number-of-tokens> skip_roundtrip:=<true|false> debug:=<true|false> 
```

###### Arguments

| Argument         | Default | Format | Required | Description                                                 |
|------------------|---------|--------|----------|-------------------------------------------------------------|
| `num_tokens`     | N/A     | `int`  | Yes      | Number of tokens present in the labyrinth                   |
| `skip_roundtrip` | `false` | `bool` | No       | Skip the initial token-ignoring roundtrip and start the search for tokens immediately, if set to `true` |
| `debug`          | `false` | `bool` | No       | Show debug messages in console output |


#### `current_pos` package

##### Purpose

The `current_pos` package contains the necessary files and logic for converting the TurtleBot's position, as determined from odometry data, to a coordinate representation within the map's coordinate system.

##### Functional Principle

The node subscribes to the `/odom` topic, which provides the current pose of the robot. Subsequently, the node's logic is triggered every time a new pose is received on the topic and converts it to the map frame using functionality provided by the [ROS `tf` package](https://wiki.ros.org/tf)[^tf-package].

The provided `lookupTransform` method is used to get the position and orientation of the robot in the map frame. The position is stored as an `x` and `y` coordinate, while the orientation is stored as a yaw angle (representing rotation around the z-axis; calculated using the provided `tf.euler_from_quaternion` method).  
The converted pose is packaged into a custom `PoseInMap` message, which includes the `x` and `y` position and the yaw angle. This message is then combined with the original `/odom` pose into a custom `PoseTF` message, which includes a header with a sequence number and timestamp, the original `/odom` pose, and the converted map pose.

Both the `PoseInMap` and `PoseTF` messages are used in other custom packages related to the TurtleBot's functionality.

##### Usage

The `robot2map_conversion` node can be launched with running the following command on the remote computer:

```console
$ roslaunch current_pos launch_transformer.launch
```

##### Arguments

| Argument     | Default | Format   | Required | Description                                                          |
|--------------|---------|----------|----------|----------------------------------------------------------------------|
| `debug`      | `true`  | `bool`   | No       | Show debug messages                                                  |

#### `token_inspector` package

The `token_inspector` package integrates all the necessary components to utilize the generated map and token locations from Phase 1 (see [*Architecture*](#architecture)). The package includes the logic for localizing the TurtleBot autonomously inside the labyrinth and planning a path within the labyrinth to drive towards the closest token, regardless of where the TurtleBot is placed.

The `token_inspector` package is organized into various nodes, each designed to carry out specific tasks within the labyrinth:

##### `inspector` node

###### Purpose

The `inspector` node has two main functionalities:
* localizing the TurtleBot within the labyrinth using advanced Monte Carlo localization
* requesting new targets from the [`scheduler_server` node](#scheduler_server-node), triggering runners

###### Functional Principle

As a first step, the `inspector` node creates an instance of the `WallLocalizer` class wich uses the `Advanced Monte Carlo Localization` to localize the TurtleBot within the labyrinth before being able to start requesting new target tokens to plan the path for and to drive to. For more details, refer to the *Localization using `AMCL`* block below.

<details>
<summary>Localization using <code>AMCL</code></summary>

**Purpose**

"AMCL" stands for "Adaptive Monte Carlo Localisation". It is an algorithm used by the TurtleBot to autonomously determine its position within the labyrinth. This is particularly useful when the TurtleBot is placed at a random location within the labyrinth (a scenario known as the "kidnapped robot problem"), or after completing Phase 1 and starting Phase 2 (see [*Architecture*](#architecture)).

**Functional Principle**

The `WallLocalizer` class heavily relies on logic that can also be found in the `WallFollower` and `FindWall` behaviors mentioned in [*`token_detector` - Functional Principle*](#functional-principle) to navigate through the labyrinth while the AMCL algorithm provided by the [`amcl` package](https://wiki.ros.org/amcl) tries to localize the TurtleBot within the given map from Phase 1.  
To make sure that the TurtleBot has navigated through the labyrinth for a sufficient amount of time, a minimum execution time of `3 seconds` needs to be surpassed. Additionally, the uncertanity of the provided localization result has to be below a threshold of `0.007`. These two measurements have been introduced to improve the accuracy of the AMCL algorithm as shorter or too long execution times might result in wrong localization results.

In addition to the logic, the `amcl_package` also relies on a set of parameter files (located in the `param` folder of the package), which define important parameters for the costmap, such as the footprint of the TurtleBot itself.

AMCL uses a combination of an existing map and sensor data (in this case data from the LiDAR sensor) to generate repeated estimates, or "particles", of a robot's position. The algorithm processes the continuous stream of sensor data it receives to determine which particles match the observed surroundings and which can be ignored. New particles are generated that are closer to the actual position while moving, which eventually lead to a more or less certain position of the robot within the existing map. [^amcl]

AMCL's covariance matrix describes the uncertainty across 6 dimensions. We are only interested in the diagonal values as the values do not correlate at all. `z`, `roll` and `pitch` can not occur in our robot's world, leaving only `x`, `y` and `yaw` (rotation around the `z` axis). The robot follows the wall until it is certain of the accuracy of all three values (meaning the value is below threshold).

```python
[
  [xx, xy, xz, xroll, xpitch, xyaw],
  [yx, yy, yz, yroll, ypitch, yyaw],
  [zx, zy, zz, zroll, zpitch, zyaw],
  [rollx, rolly, rollz, rollroll, rollpitch, rollyaw],
  [pitchx, pitchy, pitchz, pitchroll, pitchpitch, pitchyaw],
  [yawx, yawy, yawz, yawroll, yawpitch, yawyaw]
]
```

**Usage**

For the `WallLocalizer` class to function, the following command must be run on the remote computer:

```console
$ roslaunch amcl_localization custom_navigation.launch
```

***Note:*** When using a different map, specify the path using the `map_file` argument.

**Arguments**

| Argument            | Default                       | Format   | Required | Description                  |
|---------------------|-------------------------------|----------|----------|------------------------------|
| `map_file`          | `/killerrobot/saved-map.yaml` | `string` | No       | Location of map file         |
| `open_rviz`         | `true`                        | `bool`   | No       | Open Rviz, if `true`         |
| `move_forward_only` | `false`                       | `bool`   | No       | Only move forward, if `true` |

</details>

After the TurtleBot has successfully localized itself within the labyrinth, the `inspector` node communicates with the `scheduler_server` node (using custom `GimmeGoal` service messages (refer to [*`scheduler_server` node*](#scheduler_server-node))) to receive the next target token. The target token is then passed on to a set of specialized runners responsible for navigating to the token within the labyrinth.  
Somewhat comparable to the behaviors implemented in the [*`token_detector` package*](#token_detector-package), the `inspector` node uses different runner implementations to reach the next token:
* `MoveBaseRunner` as the main runner based on `move_base`
* `WallFollowerRunner` as "backup", should the `MoveBaserRunner` fail to reach the token for more than 10 consecutive tries

For a detailed overview of the functionality of the two runners, please refer to the runner descriptions below.

*During development, a `CustomRunner` was attempted to get implemented with all custom logic. Compared to the `MoveBaseRunner` and `WallFollowerRunner`, the results were not satisfactory which is the reason why the usage was not included.*

Due to limited time, obstacle avoidance involving the avoidance of other TurtleBots navigating through the same labyrinth concurrently has not been implemented. One of the major challenges is that the LiDAR is not able to the detect the other TurtleBots due to them having to low of a profile. Nevertheless, a detection could have been achieved through a similar approach to token detection. For instance, by attaching a distinctive colored token-like object to each TurtleBot and using the RaspberryPi camera to detect the color, the planned path could be adjusted to avoid other TurtleBots.

<details>
<summary><code>MoveBaseRunner</code> runner description</summary>

The `MoveBaserRunner` relies on three different packages:
* [`action_lib` package](https://wiki.ros.org/actionlib)
* [`move_base` package](https://wiki.ros.org/move_base)
* [`move_base_msgs` package](https://wiki.ros.org/move_base_msgs)

as well as the internal `pathfinder` node (see [*`pathfinder`*](#pathfinder-node)).

The runner makes usage of the standardized communcation model (implemented by `action_lib`) of providing a `MoveBaseGoal` to the `move_base`'s `ActionClient`, reacting to `MoveBaseFeedback` and `MoveBaseResult` messages.  
In particular, the runner creates a new `MoveBaseGoal` using the token's `x` and `y` coordinates and waits for the continuous `MoveBaseFeedback` messages to determine whether the TurtleBot is near the token's position using the euclidian distance as measurement.

The `MoveBaserRunner` differentiates between two different states that the `MoveBaseResult` messages returns, indicating that `move_base` has stopped tracking the goal:
* `GoalStatus.SUCCEEDED` implying that the TurtleBot has reached the targeted token
* Any other `GoalStatus` status, which initiates failure state

Once failure state is entered, a timer counts down 12 seconds. The idea is that the wall_follower_runner takes over for this timeframe to move the turtlebot into a different (hopefully closer) position to enable the move_base_runner to launch another try from a different starting position.

The definition of separate parameters for both the local and global costmaps was crucial in achieving proper path planning from the move_base package. Whereas the global costmap has a low inflation radius with higher cost_scaling_factor (0.5 and 80) very high costs for being very close to the wall, the local costmap needs higher inflation_radius (1.0) with smaller cost_scaling_factor (40) to get a broader area around walls. Getting this right took some time but is key to planning a path not too close to the wall that move_base stops and oscillates because it expects to crash and too far away so that in narrow passages no path can be found.

</details>

<details>
<summary><code>WallFollowerRunner</code> runner description</summary>

The `WallFollowerRunner` is somewhat comparable to the `FollowWall` and `FindWall` behaviors implemented in the [*`token_detector` package*](#token_detector-package), sharing some similarities in the logic.

Internally, the runner makes use of four different behaviors (order decending by priorityy):
* `TurnTowardsGoal`
* `TurnTowardsWall`
* `FollowWall`
* `TowardsGoal`

As a first step, the runner uses the token's `x` and `y` coordinates and the TurtleBot's position (using the custom `PoseTF`'s `x`, `y` and `angle` properties (look into the [`current_pos` package](#current_pos-package) for more details)) to turn in place until it is aligned straight towards the token. The alignment is done by calculating a direction vector between the TurtleBot and the token and calculating the angle difference between the two points.

Both the `TurnTowardsWall` and `FollowWall` behaviors are very similar in logic as described before. The only difference is, that the preferred distance to the wall is set higher. This is because the costmap has high costs the closer to the wall and move_base refuses to find a path if it already starts with too high costs.
The `TowardsGoal` behavior is a very simple implementation similar to `FindWall` that steers the TurtleBot straight ahead towards the token

All the behaviors rely on the `/cmd_vel` topic to publish any movement commands calculated while navigating through the labyrinth.

</details>

###### Usage

The `inspector` node can be launched with running the following command on the remote computer:

```console
$ roslaunch token_inspector token_inspector.launch
```

###### Arguments

| Argument         | Default | Format | Required | Description         |
|------------------|---------|--------|----------|---------------------|
| `debug`          | `false` | `bool` | No       | Show debug messages |

##### `scheduler_server` node

###### Purpose

The `scheduler_server` node serves as the primary point of communication for the inspector node. It provides new token targets in the form of custom `GimmeGoal` services and uses the token positions determined in Phase 1 (see [*Architecture*](#architecture)). The node then supplies these positions upon request and marks the tokens that have been found.

Although the `scheduler_server` is designed to handle communication with external TurtleBots from other teams, this functionality was not implemented due to time and WiFi bandwidth constraints. However, a general plan on how the communcation might have worked was planned and is described further below.

###### Functional Principle

The `scheduler_server` node is initialized by reading the locations of tokens detected in Phase 1 and registering itself as a ROS service server, providing the next goal via `give_goals_service`

When a service call is received, mainly from the `inspector` node acting as a ROS service client, the `inspector` node reports whether the targeted token has been found. If the token has been located, it is marked as such. The next step involves finding the shortest path to the next uncollected token. This is done via the `GimmePathLength` service connected to the `pathfinder` node. The tokens are sorted based on the distance to the turtlebot, and the token with the shortest distance is selected as the next goal. The response of the service includes the name and position of the selected token in the map.

The communication between the service server and client is accomplished by using the custom implemented service messages
* `GimmeGoal` containing the `id_found`; `GimmeGoalResponse` containing the `id`, `x` and `y` coordinates
* `GimmePath` containing the token's `id`, `x` and `y` coordinates with a `nav_msgs/Path` message as response
* `GimmePathLength` containing the token's `id`, `x` and `y` coordinates with a `path_length` (represented by a `float`) as response

###### scheduler_server as communcation hub for other TurtleBots

The `scheduler_server` has the potential to facilitate communication between TurtleBots for joint search and path planning within a labyrinth. This could involve broadcasting the current target token and any previously found tokens to a specified topic, allowing other TurtleBots to subscribe and keep track of each other's progress. Although the necessary code structures are in place, this functionality has not yet been implemented and remains a future possibility.

###### Usage

The `scheduler_server` node can be launched with running the following command on the remote computer:

```console
$ roslaunch token_inspector scheduler.launch
```

***Note:*** When using a different map or new token locations, specify the `token_positions.json` path using the `token_file` argument.

##### Arguments

| Argument     | Default                             | Format   | Required | Description          |
|--------------|-------------------------------------|----------|----------|----------------------|
| `token_file` | `/killerrobot/token_positions.json` | `string` | No       | Location of map file |

##### `pathfinder` node

###### Purpose

The `pathfinder` node provides both path planning and path length information for the planned paths. The `pathfinder` node uses custom `GimmePath` and `GimmePathLength` service messages to communicate with the `scheduler_server`. The node should have initially used the the path planning service provided by the `move_base` package (*deprecated*) and calculate the total length of the planned path, which can be used for further analysis and decision making by the `scheduler_server` and `inspector` nodes. Because of a different implementation für driving towards a goal, where `move_base` is already used, it was not possible to use the `/move_base/get_plan` service. The solution was to move to a more simple implementation using the euklidean distance. 
Additionally, the planning for the path itself was not necessary, as the MoveBaseRunner does not need it. Still, the implementation would be there, if changes in the architecture would need them.

###### Functional Principle

The `pathfinder` node acts as to ROS service servers for
* providing path planning as `provide_path_service` (*deprecated*)
* providing path length calculation as `provide_path_length_service`

For detailed information regarding actual path planning, look into the individual runner implementation descriptions available in chapter [*`inspector` package - Functional Principle*](#functional-principle-2). For further details regarding the path length planning, refer to the section below.

<details>
<summary>Path Length Calculation</summary>

The length of the path is calculated using the TurtleBot's current position, which is based on the custom `PoseTF` message (see [*`current_pos` package*](#current_pos-package) for more details), as well as the saved coordinates of the targeted token. The euclidean distance is utilized as the method of measuring the distance between the two points, using both coordinate's `x` and `y` values.

</details>

<details>
<summary><i>Deprecated:</i> Path Planning</summary>

The original intention was to use the `/move_base/make_plan` service which was later on replaced by the custom behavior implemented in the `MoveBaseRunner` class, utilizing the `SimpleActionClient` implementation (pleaser refer to [*`inspector` package - Functional Principle*](#functional-principle-2) for more details).

To calculate the path, the node uses the `/move_base/make_plan` service provided by the [`move_base` package](https://wiki.ros.org/move_base). The path is calculated by letting the TurtleBot's current position using the custom `PoseTF` message (refer to [*`current_pos` package*](#current_pos-package) for more details) which is converted to a `Pose` message provided by the [`geometry_msgs` package](https://wiki.ros.org/geometry_msgs). Finally, the path planning is handed over to the mentioned `make_plan` service, using both the TurtleBot's current location and the target token's coordinates as input.

To enhance the pathfinding process, the TurtleBot only considers every tenth point in the generated path as its target destination. This optimization reduces the number of path calculations and speeds up the time it takes to reach the desired token.

</details>

###### Usage

The `pathfinder` node can be launched with running the following command on the remote computer:

```console
$ roslaunch token_inspector find_path.launch
```

### Adapted Modules

#### Raspberry Pi camera configuration

To enhance the performance of the Raspberry Pi camera when detecting tokens and to avoid overheating, delays, and other limitations, the following configuration modifications were made:
* reduced the resolution from `1280x960` (initially planned resolution) to `410x308` pixels
* set the `saturation` value to `70`
* changed the white balance mode (`awb_mode`) to `incandescent`

```xml
<param name="saturation" value="70"/>
<param name="awb_mode" value="incandescent"/>
```

For customizing the camera configuration, the following two files are necessary which both have been originally copied over from the `camerav2_410x308` launch file pendants:
* `~/catkin_ws/src/raspicam_node/launch/camerav2_custom.launch`
* `~/catkin_ws/src/raspicam_node/camera_info/camerav2_custom.yaml`

## The proposed ideas for communicating with other TurtleBots

This subsection was coordinated with the other teams and will be similar.

### Solution 1: Communicating found tags

The simplest solution that we have considered is that each robot sends only the information about the tag that it has just found. In this scenario, each robot can react to the information of the other robots and cancel its approach if another robot is already working on the same tag. 

#### Advantages

* Ease of implementation: this method is simple to implement and does not require complex communication architecture or protocols. 
* Avoidance of overlap: Since each robot abandons the same tag once another robot has already found it, overlaps are avoided. 

#### Disadvantages: 

* Insufficient coordination: since there is no central control, there may be insufficient coordination between robots, which can lead to inefficiencies. 
* No prioritized processing: since there is no ranking for processing tags, there may be an uneven distribution of work if some tags are more difficult to process than others. 
* No monitoring: Since there is no central monitoring, it can be difficult to track the progress of the robots and ensure that all tags are processed.

### Solution 2: Communicating next and found tags

The second option represents an extended variant of the first solution, sharing not only information about the tags found, but also about the tags currently being approached.  

#### Advantages: 

* Better coordination: since information about approached tags is shared, there can be better coordination between robots, leading to higher efficiency. 
* Prioritized processing: as robots share information about the tags they have approached, an order of priority for processing can be established to ensure that the more difficult tags are prioritized. 
* Monitoring: Since information about the tags approached is shared, it is easier to track the robots' progress and ensure that all tags are processed. 

#### Disadvantages: 

* More complex to implement: this method is more complex to implement and requires more complex communication architecture and protocols. 
* Increased communication requirements: as more information needs to be shared, there may be an increased need for communication, which can lead to higher latency or increased susceptibility to errors.

### Solution 3: Constant monitoring

The third possibility, which the teams have elicited, is a bit more complex because in this possibility a main logic first queries the position of the robots on the map and then creates the plan for the robots to then pass the order and the plan to the tags to the robots, which then approach them and inform the main logic cyclically about the position and the tags found, so that the main logic can react to possible errors or also optimize the paths again if necessary. 

#### Advantages: 

* Optimized path: By monitoring the robot position and the found tags, the Main Logic can optimize the path, resulting in higher efficiency. 
* Error correction: by monitoring the robot position and the found tags cyclically, the Main Logic can react to errors and correct them. 

#### Disadvantages: 

* Higher complexity: this method is more complex to implement and requires a Main Logic, more complex communication architecture and protocols. 
* High resource requirements: the Main Logic requires a lot of resources because it has to constantly monitor and, if necessary, correct the robot position and the tags found. 
* Latency: Due to the cyclic monitoring of the robot position and the found tags, a higher latency can occur.

## Troubleshooting

The TurtleBot is comprised of various hardware and software components, which can experience issues due to external factors, incorrect installation, improper execution sequence, or other causes. Some problems can easily be fixed, while others may require more attention and a technical understanding.

The following list includes frequently encountered problems and respective solutions for hardware and software components in the TurtleBot.

### The TurtleBot navigates insecurley within the labyrinth and/or crashes into walls

In some cases, the TurtleBot may struggle with navigation in the labyrinth, missing walls or even crashing into them, and not detecting clearly visible tokens.

**Solution 1:** Ensure optimal lighting conditions in the room, avoiding direct sunlight which could affect the Raspberry Pi Camera and PixyCam.

**Solution 2:** The walls of the labyrinth may cause reflections if they are illuminated directly. Consider using a reflective coating, such as white paper, to reduce reflections.

### The TurtleBot is turned on but the LiDAR is not moving

In some cases, the LiDAR may not be operating even if the TurtleBot has been correctly turned on and the setup steps have been followed correctly. To resolve the issue:

**Solution 1:** It's possible that the `roslaunch turtlebot3_bringup turtlebot3_robot.launch` script has crashed or lost connection to the ROS Master. Connect to the TurtleBot using SSH, stop the script and restart it.

**Solution 2:** If the `roscore` script has crashed or lost connection to the network, connect to the remote computer, stop the script and restart it. Then, follow the steps of `Solution 1`.

### The TurtleBot is turned on but the wheels are not moving

In some instances, the Li-Po battery may have a low charge that isn't low enough to trigger the low battery beep, but it still may not be enough to power the TurtleBot's wheels.

**Solution:** To resolve this, turn off the TurtleBot by turning off the Raspberry Pi module. Wait for 10-20 seconds and then disconnect the Li-Po battery. Finally, reconnect a fully charged Li-Po battery. Refer to [General Setup](#general-setup) for further instructions.

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

### The bringup script is encountering an error

In some instances, the `roslaunch turtlebot3_bringup turtlebot3_robot.launch` script may display the error message `Creation of publisher failed: Checksum does not match` related to `sensor_msgs/JointState`. This can result in the TurtleBot being unable to navigate using Rviz, SLAM or other methods.

**Solution:** To resolve this issue, try reflashing the OpenCR firmware following [these instruction](https://emanual.robotis.com/docs/en/platform/turtlebot3/opencr_setup/#opencr-setup).

### The TurtleBot is turned on, but the LiDAR is not functioning properly and/or keeps shutting down randomly.

In some rare cases, the LiDAR may not return any sensor data and/or shut down unexpectedly, despite attempts to resolve the issue through other solutions.

**Solution 1:** Verify that the LiDAR is securely attached to the TurtleBot's top platform and all its ports are properly connected.

**Solution 2:** Consider replacing the malfunctioning LiDAR with a different, functional LiDAR module to determine if the issue is due to a faulty LiDAR.

[^cam-point-grid]: https://github.com/DominicLuidold/WS23-AutonomousSystems-III/blob/master/token_detector/src/behaviors/move_towards_token.py#L16

[^tf-package]: *"tf is a package that lets the user keep track of multiple coordinate frames over time. tf maintains the relationship between coordinate frames in a tree structure buffered in time, and lets the user transform points, vectors, etc between any two coordinate frames at any desired point in time."*.  
  See [`tf` package in ROS wiki](https://wiki.ros.org/tf)

[^amcl]: ‘Adaptive Monte Carlo Localization’, Robotics Knowledgebase, Feb. 03, 2020. https://roboticsknowledgebase.com/wiki/state-estimation/adaptive-monte-carlo-localization/ (accessed Feb. 06, 2023).

[^pose-stamped]: https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html

## Internal development documentation

### General ROS Configuration

|Purpose | File/Program | Action|
|---|---|---|
|ROS_Master | .bashrc (Turtlebot & Computer) | ```shell export ROS_MASTER_URI=http://192.168.240.18:11311 ```
ROS_Hostname | .bashrc (Turtlebot & Computer) | 
``` 
export ROS_HOSTNAME=$(ip route get 8.8.8.8 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}') 
```

<table>
<tr>
<td>Purpose</td><td>File/Program</td><td>Action</td>
</tr>
<tr>
<td>ROS_Hostname</td><td>.bashrc (Turtlebot & Computer) </td><td>

``` 
export ROS_MASTER_URI=http://192.168.xx.xx:11311 
```

</td>
</tr>
<tr>
<td>ROS_Master</td><td>.bashrc (Turtlebot & Computer) </td><td>

``` 
export ROS_HOSTNAME=$(ip route get 8.8.8.8 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}') 
```

</td>
</tr>
<tr>
<td>Turtlebot_Model</td><td>.bashrc (Turtlebot & Computer) </td><td>

``` 
export TURTLEBOT3_MODEL=burger
```

</td>
</tr>
<tr>
<td>Netzwerkbrücke (nur bei VM)</td><td>VM Manager -> Change -> Network</td><td> Connected to: Network Bridge <br>
+ expanded -> Promiscuous-Mode: allow for all VMs
</td>
</tr>
</table>


### IP Adresses

Gerät | IP Adresse |
|---|---
Turtlebot | 192.168.0.50
VM Stefan | 192.168.0.55
VM Dominik | 192.168.0.54
Florian | 192.168.0.52

### Commands

<table>
<tr>
<td>Shell</td><td>Command</td><td>Description</td>
</tr>
<tr>
<td>ROS_Master PC</td><td>

``` 
roscore 
```

</td><td>For starting the ROS on the ROS_MASTER.  </td>
</tr>
<tr>
<td>Turtlebot</td><td>

``` 
bringup (roslaunch turtlebot3_bringup turtlebot3_robot.launch)  
```

</td><td>Bring up basic packages to start turtlebot3 applications.   </td>
</tr>
<tr>
<td></td><td>

``` 
roslaunch pixy_node pixy_usb.launch 
```

</td><td>Run pixy node on turtlebot.  </td>
</tr>
<tr>
<td></td><td>

``` 
roslaunch raspicam_node camerav2_1280x960.launch 
```

</td><td>Start publishing images via raspicam.  </td>
</tr>
<tr>
<td>Local</td><td>

``` 
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch 
```

</td><td>For moving the turtlebot with remote keys.    </td>
</tr>
<tr>
<td></td><td>

``` 
roslaunch turtlebot3_slam turtlebot3_slam.launch
```

</td><td>For launching RVIZ with the slam node to create a map. </td>
</tr>
<tr>
<td></td><td>

``` 
rosrun map_server map_saver -f ~/*/map
```

</td><td>For saving your in RVIZ visualized map somewhere.   </td>
</tr>
<tr>
<td></td><td>

``` 
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/*/map.yaml 
```

</td><td>For launching RVIZ with a saved map and use it to automatically navigate the robot somewhere.     </td>
</tr>

<tr>
<td></td><td>

``` 
roslaunch token_detector image_viewer.launch 
```

</td><td>Start script to display image stream of raspicam/simulated cam of waffle. </td>
</tr>
<tr>
<td></td><td>

``` 
roslaunch token_detector token_detector.launch 
```

</td><td>Start script to identify and drive towards tokens with raspicam.</td>
</tr>
<tr>
<tr>
<td></td><td>

``` 
roslaunch wall_follower follow_wall_better.launch
```

</td><td>Starts wallfollower which drives in a proportional way around corners.</td>
</tr>
<tr>
<td></td><td>

``` 
roslaunch wall_follower follow_wall_better_once.launch
```

</td><td>Starts wallfollower which drives in a proportional way around corners and stops when back at starting position.</td>
</tr>
<tr>
<td></td><td>

``` 
roslaunch current_pos launch_transformer.launch
```

</td><td>Starts node which transforms the /odom koordinates to the map koordinates and publishes them via /pose_tf for the map. (visible in turlebot3_slam visualization in RVIZ)</td>
</tr>
<tr>
<td></td><td>

``` 
rosrun move_base move_base
```

</td><td>Starts move_base node which <b>should</b> in theory be able to move the bot to a certain position. </td>
</tr>
<tr>
<td></td><td>

``` 
rostopic echo /my_pixy/block_data 
```

</td><td>Display pixycam data. </td>
</tr>
<tr>
<td></td><td>

``` 
rostopic echo /odom 
```

</td><td>Display odometry data alias the estimated current position. </td>
</tr>
<tr>
<td></td><td>

``` 
rosrun rqt_graph rqt_graph
```

</td><td>Display graph of running nodes and topics - not usefull when to much is running.</td>
</tr>
<tr>
<td></td><td>

``` 
rosrun rqt_reconfigure rqt_reconfigure
```

</td><td>For adjusting parameters of the turtlebot.</td>
</tr>
<tr>
<td></td><td>

``` 
raspistill -v -o test.jpg
```

</td><td>Take a picture with Raspicam</td>
</tr>
<tr>
<td>Local simulation</td><td>

``` 
TURTLEBOT3_MODEL=waffle
roslaunch token_detector labyrinth.launch
```

</td><td>Start Gazebo with standard labyrinth and turtlebot model waffle (to use camera).</td>
</tr>
<tr>
<td></td><td>

``` 
TURTLEBOT3_MODEL=waffle
TURTLEBOT3_MODEL=waffle roslaunch token_detector labyrinth_small.launch 
```

</td><td>Start small labyrinth in gazebo with turtlebot model waffle. </td>
</tr>


</table>

### Probleme
#### catkin_make scheitert nach Installation der Raspicam packages
--> Dependencies fehlen. Die dependencies wurden manuell installiert.   
install: libraspberrypi-dev, libraspberrypi-bin, libraspberrypi0

#### Raspicam Software lässt sich nicht installieren
Durch die Ubuntuinstallation ist die raspi-config und raspistill nicht verfügbar.  
--> Lösung:
1. raspi-config: Run des sh-scripts (https://github.com/EmilGus/install_raspi-config/blob/master/install.sh)   
2. Anschließender Fehler: no start_x.elf -> /boot/firmware/usercfg.txt -> add:
- start_file=/boot/firmware/start_x.elf 
- fixup_file=/boot/firmware/fixup_x.dat 
- gpu_mem=128
3. Reboot
4. Dann mount der camera (https://chuckmails.medium.com/enable-pi-camera-with-raspberry-pi4-ubuntu-20-10-327208312f6e )
5. sudo raspi-config -> Interfacing Options -> Camera -> Enable  
6. Reboot 

### Tutorials
#### Turtlebot Quick-Start
https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup

#### Raspicam Setup
- https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_raspi_cam/
- Enablen der Kamera mit mount: https://chuckmails.medium.com/enable-pi-camera-with-raspberry-pi4-ubuntu-20-10-327208312f6e
- Eintrag von start_x: https://github.com/maxnet/berryboot/issues/82
- Furthermore: https://wiki.freebsd.org/arm/Raspberry%20Pi%20Camera?highlight=%28camera%29 leads to https://launchpad.net/ubuntu/+source/raspberrypi-userland
- install of raspi-config: https://dexterexplains.com/r/20211030-how-to-install-raspi-config-on-ubuntu

#### Pixy CMUcam5 Setup
1. Install libpixyusb:
https://docs.pixycam.com/wiki/doku.php?id=wiki:v1:building_the_libpixyusb_example_on_linux 
2. Install pixymon locally on ubuntu (not turtlebot): https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:installing_pixymon_on_linux   
--> this documentation refers to pixy2 cam, not CMUcam5. Instead of qt5 packages, qt4 are required:   
project: https://github.com/charmedlabs/pixy   
qt4: https://ubuntuhandbook.org/index.php/2020/07/install-qt4-ubuntu-20-04/ 
packages: qt4-qmake, qt4-default, qt4-dev-tools 
3. Pixy_ros: https://github.com/jeskesen/pixy_ros   
` catkin_make install `  
fix missing ros.h and Pixy_msgs.h: https://github.com/jeskesen/pixy_ros/issues/2   
Install missing ros package angles: 
` sudo apt install ros-noetic-angles `  
catkin_make gets stuck: follow 
https://github.com/UbiquityRobotics/raspicam_node/issues/112#issuecomment-982987227 and enable memory swapping 
4. Set permissions for pixy via usb: 
Create file `/etc/udev/rules.d/pixy.rules` with content: 
    ```
    # Pixy device, set permissions 
    SUBSYSTEM=="usb", ATTR{idVendor}=="b1ac", ATTR{idProduct}=="f000", MODE="0666" 
    SUBSYSTEM=="usb", ATTR{idVendor}=="1fc9", ATTR{idProduct}=="000c", MODE="0666" 
    ```
5. Der PixyCam den Token beibringen: https://docs.pixycam.com/wiki/doku.php?id=wiki:v1:teach_pixy_an_object_2 

#### Raspicam Visualization in RViz
Link: https://answers.ros.org/question/391646/no-camera-topics/   
1. Start roscore and bringup 
2. On the Turtlebot, start the raspicam_node: 
    ```
    roslaunch raspicam_node camerav2_1280x960.launch
    ```
3. Then start rviz with turtlebot3_slam
4. Activate image! 

#### For trying to drive towards token function
1. Start roscore and bringup

(Steps 2-5 can be done in one go via the preconfigured launch file)

2. Start SLAM
3. Let wallfollower map whole labyrinth
4. Save map to /killerrobot/saved-map
5. Close SLAM
6. Start navigation with map
7. Find position either via AMCL or 2DNavPose
8. Saving target coordinates via
    1. Starting current_pos
    2. Moving to target position 
    3. Writing the coordinates to /killerrobot/token_positions.json
9. Move to other position
10. Move to position by letting:
``` 
roslaunch token_inspector navigator.launch
```
    
Run for a few seconds.

Current actions can be seen in RVIZ (red arrow) and in the console of the turtlebot3_navigation the current steps are logged.
Alternatively you can look at the status of the Move pase via /move_base/status topic.

##### Issues:
- Transformation of position does not yet work correctly (are in relation to the turtlebots current position as far as I know)
- Path is not free, as turtlebot is to large

Parameters can be changed via:
```
rosrun rqt_reconfigure rqt_reconfigure
```
Manual is in shared OneDrive folder or accessible via http://www.zkytony.com/documents/navguide.pdf 

#### Solution for not driving to target:
``` 
cd /opt/ros/noetic/share/turtlebot3_navigation/param
sudo nano costmap_common_params_burger.yaml

edit> footprint: [[-0.087, -0.057], [-0.087, 0.057], [0.025, 0.092], [0.025, -0.092]]

edit> inflation_radius: 0.05
```
