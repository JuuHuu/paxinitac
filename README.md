For paxini tactile sensor read and visualize in ROS2
# Usage
## communication with ur
ros2 launch ur_bringup ur_control.launch.py ur_type:=ur7e robot_ip:=192.168.100.162 launch_rviz:=false


## control using moveit
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur7e

## read the torque data
### start receive and plot program in remote computer:
python3 plot_joint_torque.py

### send the script to ur:
python3 external_control_server.py read_torque.script 

### start the external control program in UR