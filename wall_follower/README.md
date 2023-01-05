# Wall Follower

## Implementation documentation
All of the following specifications assume that these commands have been executed:
```sh
roscore
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

### Node details

Allows a turtlebot to follow  a wall on the right hand side. The node publishes `geometry_msgs/Twist` messages to the topic `cmd_vel` using pre-defined `twist.linear.x` & `twist.angular.z` values.

The node is started with the following command and the parameters defined below to adjust the behaviour:
```sh
roslaunch wall_follower follow_wall.launch
```

| Parameter    | Default | Format | Required | Description                                                           |
|--------------|---------|--------|----------|-----------------------------------------------------------------------|
| `debug`      | `false` | `bool` | No       | Shows debug messages                                                  |
| `lidar_data` | `false` | `bool` | No       | Shows lidar data in the console, requires `debug` to be set to `true` |
