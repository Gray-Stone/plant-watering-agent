

* The issue appear after tried to run https://gist.github.com/hello-binit/4e7059721a05c6714e30ec7b7551ef4a script for a few time. with ros driver `ros2 launch stretch_core stretch_driver.launch.py mode:=trajectory broadcast_odom_tf:=True`

The draw_circle script is modified slightly. (comment out the `rclpy.init()` and to get it working on robot.)

* Stopped the driver.
* Launch the driver again 
* called `ros2 service call /home_the_robot std_srvs/srv/Trigger {}` to home robot 
* Tried to run draw-circle again, Same issue still happening. No change. 

* run `sudo reboot`
* Launch robot driver,
* Then tested with draw-circle, same issue.
* Called `ros2 service call /home_the_robot std_srvs/srv/Trigger {}` to home robot again.
* Then run draw-circle, again, same issue.

* run `sudo shutdown now`
* Turn the power button off then on (after lidar stopped spinning) 
* Launch robot driver,
* Called `ros2 service call /home_the_robot std_srvs/srv/Trigger {}` to home robot again.
* run draw_circle, robot did move but lift move quickly and stopped. (could be an issue of start location of trajectory is too far from current location)
* Robot did not go into the un-responsive state.

* relaunch driver in position mode and was able to move around according to the draw-circle script
* called `ros2 service call /switch_to_trajectory_mode std_srvs/srv/Trigger {}` to put robot into trajectory mode.


After robot being back to working and responsive state, tried again to reproduce the error.


* Launch in position mode, commanded robot using both moveit and draw_circle. Robot moved no problem.
* Stopped the stretch_driver
* Launch again in trajectory mode,  
* Arm and lift become completely not responsive.