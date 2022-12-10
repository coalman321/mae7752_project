# mae7752_project

## helpful notes

* Cartesian path: 
    https://answers.ros.org/question/256803/moveit-problem-about-cartesian-paths-of-move_group_interface/
* Moveit stopping execution: 
    https://github.com/ros-planning/moveit2/issues/1761
* UR driver needs rt-kernel and no VM: 
    https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/507
* RT elevation with RT kernel:
    https://en.cppreference.com/w/cpp/thread/thread/native_handle
* Notes on setting up an RT kernel: 
    https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/real_time.md



need to have a local copy of moveit2 due to an odd bug with the controllers causing a timeout abort

## Startup instructions -- sim

1. build the src dir with `colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`

2. run the UR5 sim with `ros2 run ur_robot_driver start_ursim.sh`

3. run the driver launch file i made up `ros2 launch moveit_planner_connector ur5e.launch.py robot_ip:=192.168.56.101`

4. confirm driver spits out thread priority 99 and SCHED_FIFO OK 3 times. if not run a realtime kernel

5. start up rviz with the config in this repo with the command `rviz2`

6. in the fake pendant ui launched in step 2 over the VNC server, turn on the robot, and set it into run. Once running start a program that contains the external control block to allow the driver to connect

7. confirm the driver is fully started by seeing `Robot connected to reverse interface. Ready to recieve control commands`

8. run the actual code... like this `ros2 launch moveit_planner_connector ur_moveit.launch.py launch_rviz:=false`
