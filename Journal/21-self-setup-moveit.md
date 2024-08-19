 
# Try setting up myself 

I also tried to setup the moveit for it myself. 

Basically just follow the tutorial. mostly default, with a virtual joint called "position" between odom and base_link.

The moveit setup assistant setup everything with lots of content. After running their launch file. the rviz interface did run and showed a robot. But it is not plan-able.

There are lots of error showed up 

`[move_group-3] [WARN] [1721449929.965998153] [moveit.pilz_industrial_motion_planner.joint_limits_aggregator]: Multi-DOF-Joint 'position' not supported.`

```
[ros2_control_node-5] [WARN] [1721449929.148171719] [mock_generic_system]: Parsing of optional initial interface values failed or uses a deprecated format. Add initial values for every state interface in the ros2_control.xacro. For example: 
[ros2_control_node-5] <state_interface name="velocity"> 
[ros2_control_node-5]   <param name="initial_value">0.0</param> 
[ros2_control_node-5] </state_interface>
```

## EE 

The `stretch_arm` group's end effector needs explicit says the `parent_group="stretch_arm"` 


## Understand generated launch file 

looking at `demo.launch.py`

* launch `static_virtual_joint_tfs` which is just `static_transform_publisher` with parent and child of the virtual joint.

* launch `rsp.launch.py` which launch `robot_state_publisher` with urdf text is given as a parameter.

* launch `move_group.launch.py`. Launches `moveit_ros_move_group::move_group` with some common parameter.

* launch `moveit_rviz.launch.py` Launches rviz using `config/moveit.rviz` with param `planning pipelines` and kinematics as string.

* launch `controller_manager::ros2_control_node` with param of URDF as string.

* launch `spawn_controllers.launch` This spawn a bunch of ros2_controller manager by looking at the `moveit_conroller.yaml` file.


## Fix SRDF

The default generated SRDF from moveit setup assistant is actually not bad.

Because the collision model is a bunch of squares. The `link_wrist_pitch` and `link_arm_l0` and `link_wrist_yaw` are likely to get into collsion (with the sharp corner of the squares) when rotating the hand down (pitch). The SRDF needs to be modified to remote collision between them.

Also `link_head_pan` and `link_head_nav_cam` also have the same issue. 


According to this tutorial `https://moveit.picknik.ai/humble/doc/examples/mobile_base_arm/mobile_base_arm_tutorial.html` There needs to be some `joint_property` set for the planner joint or the motion planner will not be able to do planning with it. Despite in the log, some of the node might not understand it and complained. 


## Moveit controller 

The auto generated controller have the name `stretch_arm_controller` for group `stretch_arm`. On the other hand, the stretch driver actually provides a action: `/stretch_controller/follow_joint_trajectory: control_msgs/action/FollowJointTrajectory`. This mean the moveit controller need to be renamed to `stretch_controller` (which exists in the generated moveit_controllers.yaml)

Also stretch_driver doesn't provide any other action interface for trajectory to gripper or base. According to how PickNikRobotics and fluentrobotics did it, all joints are declared under this controller.


## Joint limit. 

The default generated joint limit is just 1 and 0s. 

Which you will get `[stretch_driver-3] [WARNING] [lift]: Joint traj not valid: segment 0 exceeds dynamic bounds of (0.150000 vel | 0.300000 acc ) with max of (0.125926 vel | 0.407407 acc )` error if directly try to execute a plan.

The PickNikRobotics have a much better joint limits. The acceleration is lower then what stretch_driver said, but seems to be a good starting point. but don't forget the wrist roll and pitch joint which doesn't exsits in PickNik's version (not sure the position/x and position/theta it decleared in this is doing anything.)

However, changing the joint limit file doesn't seems to do much of anything. Which is strange.

Thus, ignoreing the joint limit part, will just use a much vel and acc scaler

## stretch Driver param

In stretch's driver code, there is this param
```
self.declare_parameter('fail_out_of_range_goal', False)
self.fail_out_of_range_goal = self.get_parameter('fail_out_of_range_goal').value
```

Maybe we should set this to true? to guard against un-feasible goal.

## Getting octomap.

looking at this tutorial https://moveit.picknik.ai/main/doc/examples/perception_pipeline/perception_pipeline_tutorial.html

Seems like there are two types of input.
* PointCloud Occupancy Map Updater, takes in point clouds.  
* depth image occupancy map updater. Take in Depth image.

However, the floor is going to be count into the octomap by moveit, since moveit have no idea what a floor is (it's not designed for mobile robot). Which mean the base won't be able to move anywhere when generating point cloud in this way.

### Using rtabmap's octomap

Since updating the octomap is actually eats a lot of computing power. one should directly give the existing 3d map from rtabmap to moveit. 

Moveit's `/planning_scene_world` topic could be a good option. Directly take a full octomap.
https://answers.ros.org/question/405978/how-to-construct-a-planningsceneworld-message-in-moveit2/

Another option is to make a "fake" perception pipeline publisher. Take in the rtabmap point cloud and Publish as a fake sensor value 

The worse option is to have a "relay" of sensor point cloud. Only turn it on when we are able to plan for the plant.

### Collision with own octomap.

One big problem with directly giving moveit the octomap from rtbmap is the voxels of the arm itself.

when looking at the arm's direction, there will be voxels generated from seeing the arm. When we give the data to moveit by `planning_scene_world`, these voxels will end up making moveit think the arm is currently in collision with something, ignoring and refusing to plan.

moveit's own updating does ignore voxels that;s part of the robot. However, it counts the floor in, so we can't use it.

The plus side is the arm extends out to the side of the arm, so we can only update the `planning_scene_world` when not looking at the side. 

The best solution would be, only sync the `planning_scene_world` once before starting to move.

### Floor

Somehow regardless how the 

## kinematics

```
[move_group-3] [WARN] [1723770238.962241645] [kdl_parser]: The root link base_link has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF.
[move_group-3] [ERROR] [1723770238.962534111] [kinematics_plugin_loader]: The kinematics plugin (mobile_base_arm) failed to load. Error: According to the loaded plugin descriptions the class stretch_kinematics_plugin/StretchKinematicsPlugin with base class type kinematics::KinematicsBase does not exist. Declared types are  cached_ik_kinematics_plugin/CachedKDLKinematicsPlugin cached_ik_kinematics_plugin/CachedSrvKinematicsPlugin kdl_kinematics_plugin/KDLKinematicsPlugin lma_kinematics_plugin/LMAKinematicsPlugin srv_kinematics_plugin/SrvKinematicsPlugin
[move_group-3] [ERROR] [1723770238.962545594] [moveit_ros.robot_model_loader]: Kinematics solver could not be instantiated for joint group mobile_base_arm.
```

There needs to be a stretch kinematic plugin, which is not standard available via apt package.

PickNikRobotics have a package https://github.com/PickNikRobotics/stretch_moveit_plugins/tree/main/stretch_kinematics_plugin

The fluentrobotics also have one https://github.com/fluentrobotics/stretch_ros2/tree/moveit2/stretch_moveit2/stretch_moveit_plugins/stretch_kinematics_plugin/include/stretch_kinematics_plugin 

Seems to be almost exactly the same excpet one invert of logic in `stretch_kinematics_plugin.cpp:232` 

I'll try the PickNikRobotics one first. The `pick_place_task` demo package is causing build error so i COLCON_IGNORE it.

