# Behavior
## Mapping and Detection Phase

### Raspicam Configuration
Due to overheating and massive delays, the raspicam resolution needed to be reduced massively from initially 1280x960 to 410x308. Tweaking the token detection accuracy with ```rosrun rqt_reconfigure rqt_reconfigure``` we found that it works best, if saturation property is upped to 80 in the camerav2_custom.launch file:
```
<param name="saturation" value="80"/>
```


# Configuration

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


## IP Adressen

Gerät | IP Adresse |
|---|---
Turtlebot | 192.168.0.50
VM Stefan | 192.168.0.55
VM Dominik | 192.168.0.54
Florian | 192.168.0.52

# Commands

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

# Probleme
### Lidar liefert keine Werte
Lidar liefert manchmal keine Werte und schaltet sich gelegentlich selbst ab (dreht sich nicht mehr)   
--> Austausch des gesamten LIDAR mit Ersatzgerät 

### Bringup scheitert an “creation of publisher failed checksum does not match sensor_msgs/JointState”
In weiterer Folge funktioniert die Navigation des Roboters nicht. Er findet keinen Pfad zum Ziel   
--> erneuter Firmware-Upload des OpenCR Boards. Hier scheint beim ersten Mal etwas schiefgelaufen zu sein. (siehe Quick-Start Tutorial)

### Timestamps sind Out of Sync
Siehe Fehlermeldungen: 

    For frame [base_scan]: No transform to fixed frame [map].  
    TF error: [Lookup would require extrapolation -0,522100208s into the future.  Requested time 1666254110,220566034 but the latest data is at time 1666254109,698465824, when looking up transform from frame [base_scan] to frame [map]]  
    und Costmap2DROS transform timeout. Current time: 1666254261.9980, global_pose stamp: 1666254261.2392, tolerance: 0.5000 

--> Versuchen, den Raspi mit einem Zeitserver zu synchronisieren. Hat geholfen aber nicht gelöst. Schlussendlich ist Problem mit fixen vom Bringup-Problem behoben. 

### catkin_make scheitert nach Installation der Raspicam packages
--> Dependencies fehlen. Die dependencies wurden manuell installiert.   
install: libraspberrypi-dev, libraspberrypi-bin, libraspberrypi0

### Raspicam Software lässt sich nicht installieren
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

# Tutorials
### Turtlebot Quick-Start
https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup

### Raspicam Setup
- https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_raspi_cam/
- Enablen der Kamera mit mount: https://chuckmails.medium.com/enable-pi-camera-with-raspberry-pi4-ubuntu-20-10-327208312f6e
- Eintrag von start_x: https://github.com/maxnet/berryboot/issues/82
- Furthermore: https://wiki.freebsd.org/arm/Raspberry%20Pi%20Camera?highlight=%28camera%29 leads to https://launchpad.net/ubuntu/+source/raspberrypi-userland
- install of raspi-config: https://dexterexplains.com/r/20211030-how-to-install-raspi-config-on-ubuntu

### Pixy CMUcam5 Setup
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

### Raspicam Visualization in RViz
Link: https://answers.ros.org/question/391646/no-camera-topics/   
1. Start roscore and bringup 
2. On the Turtlebot, start the raspicam_node: 
    ```
    roslaunch raspicam_node camerav2_1280x960.launch
    ```
3. Then start rviz with turtlebot3_slam
4. Activate image! 

### For trying the drive towards token function
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

#### Issues:
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

