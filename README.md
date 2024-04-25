# mirobot_ros2
WLKATA Mirobot ROS 2 Pkgs

* rviz control

```
sudo chmod 777 /dev/ttyUSB0
ros2 launch mirobot_description mirobot_rviz_control.launch.py 
```

* isaac sim control

```
sudo chmod 777 /dev/ttyUSB0
ros2 run mirobot_description mirobot_gcode_writer 
ros2 launch mirobot_description mirobot_rviz_control.launch.py 
```