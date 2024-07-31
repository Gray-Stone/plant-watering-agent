
Stretch is not moving when given a valid trajectory from mvoeit.

driver is launch  using `ros2 launch stretch_core stretch_driver.launch.py mode:=trajectory broadcast_odom_tf:=True`. Moveit side actually isn't involved in this.

## Stretch Driver issue. 

After tracking down the code, the stretch_driver is just feeding each trajectory point to the underlying objects in `stretch_body`. And seems like it is the motor side stuck in some kind of initializing state which doesn't increment the trajectory index counter. 

I have made a post on stretch form asking about this. https://forum.hello-robot.com/t/stretch3-lift-arm-does-not-move-in-trajectory-mode/1061

After further tracing, seems like it is up to the motor controller firmware to put the motor into the sync mode.

## Solving the issue.

After updating firmware on the robot using provided script, it works now (despite the firmware reported everything is up to date before).

