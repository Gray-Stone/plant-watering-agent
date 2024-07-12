

## Tutorial is not remote

Most the tutorial on hello robot website is assuming the command is run on the stretch, and with a monitor plugged in to see rviz.

However most launch files does include options to turn on or off rviz. So it seems obvious to launch it with rviz off

## Missing mesh

When launching rviz from my own machine (doesn't have the whole stretch dev-env setup). Rviz is complaining about not finding meshes. I have clone the entire stretch_ros workspace locally and built them. However the mesh seems to not be part of the ros library.

According to this, I should be able to generate those mesh to my machine (uncalibrated ones). And since URDF is published by robot side, the calibration shouldn't matter.

https://docs.hello-robot.com/0.3/extending_stretch/urdf_management/

```
stretch_urdf_ros_update.py --model SE3 --tool sg3
```

The model and tool are required. The tool name could be find by reverse engineering [their code](https://github.com/hello-robot/stretch_urdf/blob/main/tools/stretch_urdf_ros_update.py#L126)

This script seems to have hard-corded understanding of ros workspace structure, and insists on finding the src.

https://github.com/hello-robot/stretch_urdf/blob/main/tools/stretch_urdf_ros_update.py#L66

And it also depends on `stretch_body` package.

It is just a lot easier to copy everything directly from the robot's mesh folder.



## Collision in processes 

When trying to launch the rtabmap: 

```
[stretch_driver-3] [INFO] [1720220829.726550729] [stretch_driver]: stretch_driver started
[stretch_driver-3] Another process is already using Stretch. Try running "stretch_free_robot_process.py"
[stretch_driver-3] [FATAL] [1720220830.739144293] [stretch_driver]: Robot startup failed.
```

and results in 

```
▶ stretch_free_robot_process.py
Failed because robot is being used by another user. To force kill the process, try running "sudo -E env PATH=$PATH stretch_free_robot_process.py"
```

Which means something on the `hello-robot` user is the one using the robot. Since the robot boot with ability to be driven with joy-stick, that is likely the situation.

Launch files will launch: `stretch_nav2::launch::teleop_twist.launch.py`

Which the flow of info is `/joy_node` -> `/joy: sensor_msgs/msg/Joy` -> `/teleop_twist_joy_node` -> `/stretch/cmd_vel`

https://docs.hello-robot.com/0.3/getting_started/demos_mapping_and_navigation/

And the joystick have a different key mapping now.

After done, run `stretch_gamepad_teleop.py` to start control of the robot again.

# Problems 

The control of the robot is hogged by which ever process/launch file is trying to control/move the robot.

Which means I will lost joy-stick control after running some custom launch files. In this case, The "shutdown" button is also not access-able. If the robot some-how lost remote connection, then it must plug in a monitor and keyboard to operate it or bring up the joy-stick control again. But this is suppose to be a mobile robot without tether.