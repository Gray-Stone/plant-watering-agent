
## List of action for new robot integration  

https://moveit.picknik.ai/main/doc/examples/examples.html#integration-with-a-new-robot



* MoveIt Setup Assistant
GUI app that setup SRDF for robot.
* URDF and SRDF
* Low Level Controllers
Controller for moving the joint. 
Minimal setup: 
    * A YAML config file, `moveit_controllers.yaml` to tell movit which controller are available, which joints are which.
    * A Launch file, loads the `moveit_controllers.yaml` and moveit simple controller manager. 
    * Launch `ros2_control` `JointTrajectoryControllers`. which provides an action interface. (Not required to use ros2_control, can write a special action interface, but a common practice.)

* Perception Pipeline Tutorial
* Hand-Eye Calibration
* IKFast Kinematics Solver

## Finding community moveit port

We can find forks of the `hello-robot/stretch_ros2` repo, and see which forks have what commit added on top. https://github.com/hello-robot/stretch_ros2/network

fluentrobotics seems to have a good chunk of additional commits on top of the `hello-robot::stretch_ros2` humble branch. It also have a moveit branch created not too far behind the current humble branch.

* Try directly copy just the moveit folder over.
* Check what have fluentrobotics changed and consider getting those changes.
    * Branch elvout/hrc-wip : seem to have add teleop keyboard to nav mode

PickNikRobotics (owner of Moveit) Also have a stretch_ros fork with Moveit. https://github.com/PickNikRobotics/stretch_ros. 

The latest commits seems to be a contribution from another fork https://github.com/Abishalini/stretch_ros/tree/pr-humble-rolling-support.

Should compare code across all of them to create a stretch_moveit package.



| | PickNik | fluentrobotics | |  
|-- |-- | -- |  -- | 
| srdf | stretch_description.srdf | stretch_description_standard.srdf |  same link and joint name, just some preset difference. Picknik have extra pair of disabled collision check for gripper. |
| sensors_3d.yaml | -- | Have an extra DelpthImageOctomapUpdater | Both have PointCloudOctomapUpdater |
| ros_controllers.yaml | -- | extra joint_wrist_pitch and roll | other the same |
|stretch_base/head.ros2_control.xacro| -- |-- | same |
| stertch_arm.ros2_control.xacro| -- | extra wrist pitch roll | -- |
| stertch.xacro | use fake default false | use fake default true | slight file name difference |

## Stretch 2 3 change.

The above packages seems to be for stretch 2. 

When launching them, moveit complained about `link_gripper` is not in URDF. The SRDF is missing joint `link_gripper_s3_body`

`link_head_tilt` and `link_head_nav_cam` seems to be in collision (which they are mounted together). and `link_head_pan` should also disable collision with `link_head_tilt`. Also `link_head_til` and `camera_link` shouldn't collide.

