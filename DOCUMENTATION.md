# Documentation

<img src="documentation/TurtleBot3_SteFloDom.jpg" alt="Image of the `SteFloDom` TurtleBot 3" style="height: 500px" />

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

Should the TurtleBot get set up for the first time in a new network, proceed with the following steps. Otherwise skip and continue with the [General Setup](#general-setup).

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
4. Run
    ```console
    $ sudo mkdir /killerrobot && sudo chmod 766 /killerrobot
    ```

##### TurtleBot
1. Connect to the TurtleBot via SSH (`$ ssh ubuntu@<turtlebot-ip-address>`) with default-password `turtlebot`
    * ***Note:*** If the TurtleBot isn't already connected to the new WiFi network, connect a keyboard and monitor directly to the TurtleBot's Raspberry Pi module and add the network's SSID and password accordingly.
2. Determine the IP address of the TurtleBot
3. Edit the `~/.bahsrc` file and set
    * `ROS_HOSTNAME` with value `<turtlebot-ip-address>`
    * `ROS_MASTER_URI` with value `http://<remote-pc-ip-address>:11311`
4. Run
    ```console
    $ source ~/.bashrc
    ```

Please also see the following image taken from [`TurtleBot 3 Quick Start Guide (Noetic) - 3.1.5 Network Configuration`](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start):
[![TurtleBot 3 Quick Start Guide - 3.1.5 Network Configuration](https://emanual.robotis.com/assets/images/platform/turtlebot3/software/network_configuration.png)](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start)

#### General Setup

##### PixyCam

To be able to detect any tokens using the PixyCam, the camera has to be trained every time before being able to start running the built-in software. To do so, proceed with the following steps:
1. Make sure the color indication LED is covered with a piece of tape or similar to avoid reflections. As the PixyCam is placed at the bottom, facing the surface, it comes to wrong color appearance if a light shines on it.
2. Place the PixyCam over a red token
3. Press and hold the button until the LED turns white (it will switch to all colors after 1 second), then release when it turns red
4. Move the camera over a the token (results are best real conditions are reproduced - put the turtlebot on the ground, pixycam directly over a token)
    * ***Note:*** The better the lighting conditions the better
6. Press the button once
7. Make sure that tokens are detected: Hovering over a token makes the LED turn red (can be seen from above through little holes) and moving away the PixyCam turns off the LED. Make sure that the light is steady and not flickering, otherwise repeat the process.

 ***Note:*** See the [Pixy Documentation](https://docs.pixycam.com/wiki/doku.php?id=wiki:v1:teach_pixy_an_object_2) for more information. Also, the lens sharpness can be adjusted by turning the camera housing, but too much or too little adjustment may affect image quality. Use the PixyMon tool for the initial calibration!

##### Software

To be able to run the software built into the TurtleBot, proceed with the following steps:
1. On the remote computer, run
    ```console
    $  roscore
    ```
2. Connect to the TurtleBot via SSH (`$ ssh ubuntu@<turtlebot-ip-address>`) with default-password `turtlebot`
3. Run (on the TurtleBot)
    ```console
    $ roslaunch turtlebot3_bringup turtlebot3_robot.launch
    ```
4. Run (in a new terminal session on the TurtleBot)
    ```console
    $ roslaunch pixy_node pixy_usb.launch 
    ```
5. Run (in a new terminal session on the TurtleBot)
    ```console
    $ roslaunch raspicam_node camerav2_custom.launch
    ```

Once all steps have been executed, the general setup is completed and the TurtleBot is ready.

### Using the TurtleBot

To start using the TurtleBot, proceed with the following steps:
1. Create a labyrinth
    * ***Note:*** For optimal results, use labyrinth pieces from room `U131 Lab. Auton. Systeme` of Vorarlberg UAS with a minimum labyrinth width of one piece. Tokens *must* be spaced at least 20cm apart and not closer to the wall than 15cm.
2. Follow the steps described in the [General Setup](#general-setup) chapter
3. Have the remote computer with the compiled source code and packages ready

#### Map labyrinth and detect tokens

On the remote computer, run (in a new terminal):

```console
$ roslaunch token_detector token_detector.launch num_tokens:=<number-of-tokens>
```

After the TurtleBot has completed the first round trip, the following message will be displayed in the terminal:
```
Initial roundtrip complete! Switched to different behaviors
```

The TurtleBot will then start following the left-hand side wall of the labyrinth, detecting all tokens and saving their positions in a reusable format.

***Note:*** Once all tokens have been detected, the TurtleBot will automatically stop. Once the TurtleBot has fully stopped for 10-20 seconds, stop the script by entering `CMD+C`.

#### TODO -- TOKEN INSPECTOR -- TODO

## Architecture

The TurtleBot's software is divided into two main phases, each of which requires user input to start and end. The two phases work together to allow the TurtleBot to navigate through any given labyrinth (that meets the minimum requirements mentioned in [*Using the TurtleBot*](#using-the-turtlebot)), detect tokens, and ultimately plan a path through the labyrinth to reach each token in turn.

Phase 1, using the `token_detector` package, is the first step in this process. Its primary function is to generate a map of the labyrinth by following the wall on the left and making a full round trip through the labyrinth. During this phase, the TurtleBot travels through the labyrinth and collects LiDAR data about its surroundings, using this information to construct a map of the labyrinth. This map is then stored for future use. The first round trip of Phase 1 is an important and encapsulated step to ensure that a correct map of the given labyrinth has been generated and saved before proceeding with any further steps, as this will be used as the basis for all further operations. Detecting tokens while creating a map would have been possible, but accurately transforming the location based on the TurtleBot's coordinates and an incomplete map would have been a more difficult and error-prone approach.  
Once the map is generated, Phase 1 continues with the TurtleBot driving through the labyrinth again, this time with the goal of detecting tokens and storing their location based on the generated map. The TurtleBot uses the wall following approach to navigate and uses its token detection algorithm (see [*`token_detector` - Functional Principle*](#functional-principle) for more details) to identify each token it encounters and store its location. It will continue to do this until it has found the number of tokens specified by the user.

Phase 2 -> todo

### Custom Modules

#### `token_detector` package

The `token_detector` package integrates all the necessary components to map the world using slam and perform detection and localization of tokens, specifically paper or post-it notes with a unique red color, within a labyrinth. The package includes the logic for navigating the labyrinth, to identify the tokens and saving their positions within the map in json format. Also the map is saved in ros standard yaml and pgm format.

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

The `token_detector` node serves two main purposes: mapping the labyrinth in which the TurtleBot is placed, and detecting and saving the positions of tokens within the labyrinth. During the initial mapping phase, the labyrinth is explored using a left-wall following approach. The TurtleBot moves along the walls on its left until the entire labyrinth has been mapped. In the second phase, the TurtleBot traverses the labyrinth once again, detecting and recording the positions of any tokens encountered during its journey. These positions are stored in a locally saved JSON file that can be easily reused where needed (for a more detailed explanation, refer to *`CaptureToken` behavior description*).

###### Functional Principle

The `token_detector` node implements a reactive and behavior-based architecture, utilizing a subsumption architecture made up of multiple independent behaviors. This design enables the robot to efficiently explore its surroundings and gather tokens. The main node continuously determines which behavior is appropriate to execute based on the sensor data at any given time.

The `token_detector` consists of the following five behaviors (ordered descending by priority):
* `StepOntoToken`: The highest priority behavior responsible for stepping onto a detected token.
* `CaptureToken`: The behavior is responsible for capturing tokens in the labyrinth, saving their location.
* `MoveTowardsToken`: This behavior drives the robot towards a detected token.
* `WallFollower`: The behavior is responsible for following the labyrinth walls on the left hand side.
* `FindWall`: This behavior is responsible for finding a wall and orienting the robot towards it.

The specific priority of the behaviors has been chosen to ensure that reacting to a token has the highest priority, as this is one of the core functionalities and requirements of the TurtleBot's software. Following and finding a wall have been given the lowest priority, as finding and following a wall is always possible, whereas missing a token may result in significantly longer runtime and an increased likelihood of errors or missed tokens.

<details>
<summary><code>StepOntoToken</code> behavior description</summary>

The `StepOntoToken` behavior relies on the PixyCam camera/sensor and allows the robot to move onto a token in the labyrinth. The bahavior is only applicable when the PixyCam is detecting a token AND when it's position in the map is not within a certain distance to all already recognized tokens to avoid detecting tokens twice when approaching from different angles.
If both conditions are met, the robot is moved towards the token at a slower speed.

To be able to detect when the PixyCam is recognizing a token, the custom `PixycamDetector` class is used which relies on the `/my_pixy/block_data` topic provided by the [pixy_ros package](https://github.com/jeskesen/pixy_ros). As it seems to be normal behavior that every 3 block_data messages there is an empty message, a counter for consecutive non-detection is implemented to be sure, that the turtlebot is indeed not above a token any more.

</details>

<details>
<summary><code>CaptureToken</code> behavior description</summary>

This behavior is applicable once the StepOntoToken behavior is not executed any more. It only executed once to capture a token
Once the `StepOntoToken` behavior has confirmed that the TurtleBot is located on a token, the behavior waits for the custom `PoseTF` (see [*`current_pos` package*](#current_pos-package)) message published on the `/pose_tf` topic which represents the position of the token transformed into the map frame.

The behavior then checks if there is already a registered token within a distance of `15 cm` from the current TurtleBot position. If not, the captured token is registered, assigned a `tagno` and the information (in form of `tagno`, `x`, `y` and `angle`) is saved in a JSON file located at `/killerrobot/token_positions.json`.

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
The closest grid point is then calculated by finding the euclidean distance between the token's `x` and `y` coordinates and these map coordinates. This information is used to determine the token's location within the map frame and determine if it has already been recognized based on its calculated position and previously calculated positions of other tokens.

<details>
<summary>Camera Point Grid for <code>410x308</code> Pixels Resolution</summary>

<img src="documentation/camera_point_grid.jpg" alt="Camera Point Grid for `410x308` Pixels Resolution" style="height: 700px" />

</details>

To know whether a token has already been registered, the `any_registered_token_within_distance` helper function is used that takes an estimated token map pose, a list of registered tokens, and a distance threshold as input and returns information whether there is any already registered token that is closer in Euclidean distance to the estimated token than the given distance threshold.

The `MoveTowardsToken` behavior includes parts of the `WallFollower` logic to prevent driving too close or crashing into a wall while steering towards a detected token. For improved accuracy, instead of just the front, front-left, and left surroundings, the complete 360° range of the LiDAR is used.

</details>

<details>
<summary><code>WallFollower</code> behavior description</summary>

The `WallFollower` behavior is internally controlled by two different classes, namely `TurnTowardsWall` and `FollowWall`, as well as a `RoundtripMonitor` class responsible for switching behaviors once the TurtleBot has completed the first round trip.

The `TurnTowardsWall` logic turns towards a wall if the wall is not in front of the robot but is on its left. This is determined by checking the distance to the wall in front, front_left, and left directions using the LiDAR data published to the `/scan` topic. If the wall is on its left, the robot turns left towards the wall. The speed of the turn is determined by the distance to the wall. The closer the robot is to the wall, the slower the turn.

The `FollowWall` logic instructs the TurtleBot to move forward and turn slightly left to keep the wall on its left side. The speed of the robot and the degree of the turn is determined by the distance to the wall in front, front_left, and left directions. The closer the robot is to the wall, the slower it moves and the sharper the turn. Comparable to the `TurnTowardsWall` logic, the `/scan` topic is used to get the LiDAR data.

The `RoundtripMonitor` monitors if the robot has completed a roundtrip by keeping track of the initial pose (using the custom `PoseTF` message (see [`current_pos` package](#current_pos-package)) of the robot when it first made contact with the wall, and checking if the robot is near the same pose after leaving the area. If the robot is near the same pose again, it is considered to have completed a roundtrip.  
To determine whether the current position is near the starting area of the roundtrip, the euclidean distance is used as a measure based on the `x` and `y` coordinates of the TurtleBot.

</details>

<details>
<summary><code>FindWall</code> behavior description</summary>

The `FindWall` behavior has the lowest priority among the five available behaviors and is only executed when none of the other behaviors are applicable. When this behavior is triggered, the TurtleBot uses the `/cmd_vel` topic to move slowly towards a wall until it reaches one, at which point control is transferred to another behavior.

</details>

The main node contains the logic for creating an initial map of the labyrinth based on the two `WallFollower` and `FindWall` behaviors without paying attention to any existing tokens. Once the labyrinth has been fully mapped, the TurtleBot's set of behaviors is adjusted to include the remaining three behaviors which focus on finding and recording the positions of the tokens. Once all tokens (specified by the `num_tokens` parameter) have been discovered, the `token_detector`  tells the TurtleBot to stop upon which user interaction is required.

###### Usage

The `token_detector` node can be launched with running the following command on the remote computer:

```console
$ roslaunch token_detector token_detector.launch num_tokens:=<number-of-tokens>
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

The `token_inspector` package integrates all the necessary components to utilize the generated map and token locations from Phase 1 (see [*Architecture*](#architecture)). The package includes the logic for localizing the TurtleBot autonomously inside the labyrinth and planning a path within the labyrinth to drive to the tokens in the order they were found, regardless of where the TurtleBot is placed.

The `token_inspector` package is organized into various nodes, each designed to carry out specific tasks within the labyrinth:

##### `inspector` node

###### Purpose

The `inspector` node has two main functionalities:
* localizing the TurtleBot within the labyrinth using advanced Monte Carlo localization
* requesting new targets from the [`scheduler_server` node](#scheduler_server-node), triggering runners

###### Functional Principle

As a first step, the `inspector` node creates an instance of the `WallLocalizer` class wich uses `Advanced Monte Carlo Localization` to localize the TurtleBot within the labyrinth before being able to start requesting new target tokens to plan the path for and to drive to. For more details, refer to the *Localization using `AMCL`* block below.

<details>
<summary>Localization using <code>AMCL</code></summary>

**Purpose**

"AMCL" stands for "Adaptive Monte Carlo Localisation". It is an algorithm used by the TurtleBot to autonomously determine its position within the labyrinth. This is particularly useful when the TurtleBot is placed at a random location within the labyrinth (a scenario known as the "kidnapped robot problem"), or after completing Phase 1 and starting Phase 2 (see [*Architecture*](#architecture)).

**Functional Principle**

The `WallLocalizer` class heavily relies on logic that can also be found in the `WallFollower` and `FindWall` behaviors mentioned in [*`token_detector` - Functional Principle*](#functional-principle) to navigate through the labyrinth while the AMCL algorithm provided by the [`amcl` package](https://wiki.ros.org/amcl) tries to localize the TurtleBot within the given map from Phase 1.  
To make sure that the TurtleBot has navigated through the labyrinth for a sufficient amount of time, a minimum execution time of `3 seconds` needs to be surpassed. Additionally, the uncertanity of the provided localization result has to be below a threshold of `0.007`. These two measurments have been introduced to improve the accuracy of the AMCL algorithm as shorter or too long execution times might result in wrong localization results.

In addition to the logic, the `amcl_package` also relies on a set of parameter files (located in the `param` folder of the package), which define important parameters for the costmap, or for example the footprint of the TurtleBot itself.

AMCL uses a combination of an existing map and sensor data (in this case data from the LiDAR sensor) to generate repeated estimates, or "particles", of a robot's position. The algorithm processes the continuous stream of sensor data it receives to determine which particles match the observed surroundings and which can be ignored. New particles are generated that are closer to the actual position while moving, which eventually lead to a more or less certain position of the robot within the existing map. [^amcl]

**Usage**

For the `WallLocalizer` class to function, the following command must be run on the remote computer:

```console
$ roslaunch amcl_localization custom_navigation.launch
```

***Note:*** When using a different map than the pre-defined development map, specify the path using the `map_file` argument.

**Arguments**

| Argument            | Default                                   | Format   | Required | Description                                                 |
|---------------------|-------------------------------------------|----------|----------|-------------------------------------------------------------|
| `map_file`          | `maps/3token_corner_d0.02/saved-map.yaml` | `string` | No       | Location of map file (relative to `token_inspector` package |
| `open_rviz`         | `true`                                    | `bool`   | No       | Open Rviz, if `true`                                        |
| `move_forward_only` | `false`                                   | `bool`   | No       | Only move forward, if `true`                                |

</details>

After the TurtleBot has successfully localized itself within the labyrinth, the `inspector` node continuously communicates with the `scheduler_server` node (using custom `GimmeGoal` and `GimmeGoalResponse` messages (refer to [*`scheduler_server` node*](#scheduler_server-node))) to receive the next target token. The target token is then passed on to a set of specialized runners responsible for navigating to the token within the labyrinth.  
Somewhat comparable to the behaviors implemented in the [*`token_detector` package*](#token_detector-package), the `inspector` node uses different runner implementations to reach the next token:
* `MoveBaseRunner` as the main runner based on `move_base`
* `WallFollowerRunner` as "backup", should the `MoveBaserRunner` fail to reach the token for more than 10 consecutive tries

For a detailed overview of the functionality of the two runners, please refer to the runner descriptions below.

*During development, a `CustomRunner` was attempted to get implemented with all custom logic. Compared to the `MoveBaseRunner` and `WallFollowerRunner`, the results were not satisfactory which is the reason why the usage was not included.*

<details>
<summary><code>MoveBaseRunner</code> runner description</summary>

The `MoveBaserRunner` relies on three different packages:
* [`action_lib` package](https://wiki.ros.org/actionlib)
* [`move_base` package](https://wiki.ros.org/move_base)
* [`move_base_msgs` package](https://wiki.ros.org/move_base_msgs)

as well as the internal `pathfinder` node (see [*`pathfinder`*](#pathfinder-node)).

The runner makes usage of the standardized communcation model (implemented by `action_lib`) of providing a `MoveBaseGoal` to the `move_base`'s `ActionClient`, reacting to `MoveBaseFeedback` and `MoveBaseResult` messages.  
In particular, the runner creates a new `MoveBaseGoal` using the token's `x` and `y` coordinates and waits for the continuous `MoveBaseFeedback` messages to determine whether the TurtleBot is near the token's position using the euclidian distance as measurment.

The `MoveBaserRunner` differentiates between two different states that the `MoveBaseResult` messages returns, indicating that `move_base` has stopped tracking the goal:
* `GoalStatus.SUCCEEDED` implying that the TurtleBot has reached the targeted token
* Any other `GoalStatus` status, which counts toward an internal failure count

Should the internal failure count fail for more than 10 (consecutive) times, the runner disables itself preemtively to avoid any further runs not resulting in finding the specified token. As a result, the `WallFollowerRunner` will get called by the `inspector` node to guarantee that the targeted token will get reached.

</details>

<details>
<summary><code>WallFollowerRunner</code> runner description</summary>

The `WallFollowerRunner` is somewhat comparable to the `FollowWall` and `FindWAll` behaviors implemented in the [*`token_detector` package*](#token_detector-package), sharing some similarities in the logic. Compared to the behavior, the runner's implementation allows for receiving a token as the target which is used to position the TurtleBot within the labyrinth.

Internally, the runner makes use of four different behaviors (order decending by priorityy):
* `TurnTowardsGoal`
* `TurnTowardsWall`
* `FollowWall`
* `TowardsGoal`

As a first step, the runner uses the token's `x` and `y` coordinates and the TurtleBot's position (using the custom `PoseTF`'s `x`, `y` and `angle` properties (refer to [`current_pos` package](#current_pos-package) for more details)) to align itself in a straight line towards the token. The alignment is done by calculating a direction vector between the TurtleBot and the token and calculating the angle difference between the two points.

Both the `TurnTowardsWall` and `FollowWall` behaviors are designed to keep the TurtleBot as close to the wall as possible, taking into account that any sharp edges or corners may drive the robot further away for which counter-steering is applied, while navigating through the labyrinth and reaching the targeted tokens.    
The `TowardsGoal` behavior is a very simple implementation that steers the TurtleBot towards the token.

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

The `scheduler_server` node serves as the primary point of communication for the inspector node. It provides new token targets in the form of custom `GimmeGoal`, `GimmeGoalResponse`, and `GimmePathLength` messages and uses the token positions determined in Phase 1 (see [*Architecture*](#architecture)). The node then supplies these positions upon request and marks the tokens that have been found.

Although the `scheduler_server` is designed to handle communication with external TurtleBots from other teams, this functionality was not implemented due to time constraints.

###### Functional Principle

The `scheduler_server` node is initialized by reading the locations of tokens detected in Phase 1 and registering itself as a ROS service server.

When a service call is received, mainly from the `inspector` node acting as a ROS service client, the `inspector` node reports whether the targeted token has been found. If the token has been located, it is marked as such. The next step involves finding the shortest path to the next uncollected token. The tokens are sorted based on the length of their respective paths, and the token with the shortest path is selected as the next goal. The response of the service includes the name and position of the selected token in the map.

The communication between the service server and client is accomplished by using the customm implemented messages
* `GimmeGoal` containing the `id_found`; `GimmeGoalResponse` containing the `id`, `x` and `y` coordinates
* `GimmePath` containing the token's `id`, `x` and `y` coordinates with a `nav_msgs/Path` as response
* `GimmePathLength` containing the token's `id`, `x` and `y` coordinates with a `path_length` (represented by a `float`) as response

###### Usage

The `scheduler_server` node can be launched with running the following command on the remote computer:

```console
$ roslaunch token_inspector scheduler.launch
```

***Note:*** When using a different map than the pre-defined development map or new token locations, specify the `token_positions.json` path using the `token_file` argument.

###### Arguments

| Argument     | Default                                         | Format   | Required | Description                                                 |
|--------------|-------------------------------------------------|----------|----------|-------------------------------------------------------------|
| `token_file` | `maps/3token_corner_d0.02/token_positions.json` | `string` | No       | Location of map file (relative to `token_inspector` package |

##### `pathfinder` node

###### Purpose

TODO

###### Functional Principle

TODO

### Adapted Modules

#### Raspberry Pi camera configuration

To enhance the performance of the Raspberry Pi camera when detecting tokens and to avoid overheating, delays, and other limitations, the following configuration modifications were made:
* reduced the resolution from `1280x960` to `410x308` pixels.
* set the `saturation` value to `70`.
* changed the white balance mode (`awb_mode`) to `incandescent`.

```xml
<param name="saturation" value="70"/>
<param name="awb_mode" value="incandescent"/>
```

For customizing the camera configuration, two following two files are necessary which both have been originally copied over from the `camerav2_410x308` launch file pendants that were later adapted:
* `~/catkin_ws/src/raspicam_node/launch/camerav2_custom.launch`
* `~/catkin_ws/src/raspicam_node/camera_info/camerav2_custom.yaml`

#### SLAM parameters

todo

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

### The `roslaunch turtlebot3_bringup turtlebot3_robot.launch` script is encountering an error

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
