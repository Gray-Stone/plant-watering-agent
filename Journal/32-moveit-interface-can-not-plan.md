

According to this line: 

multi joint info are obtained with TFs, so there shouldn't be a need to sepeartlly publish a multidof-joint-state message

https://moveit.picknik.ai/main/doc/examples/planning_scene_monitor/planning_scene_monitor_tutorial.html#currentstatemonitor

After printing out the root link and multiDof joint list in robot_model object, I can confirm the `position` joint is taken into account and does have multiDOF joints.



When trying to get the current ee pos, got this output

```
[INFO] [1724118003.334478944] [moveit_ros.current_state_monitor]: Didn't receive robot state (joint angles) with recent timestamp within 1.000000 seconds. Requested time 1724118002.334350, but latest received state has time 1724118002.111688.
Check clock synchronization if your are running ROS across multiple machines!
[ERROR] [1724118003.334566150] [move_group_interface]: Failed to fetch current robot state
```

`Failed to fetch current robot state` is from https://moveit.picknik.ai/humble/api/html/move__group__interface_8cpp_source.html#l00592

https://moveit.picknik.ai/humble/api/html/current__state__monitor_8cpp_source.html#l00231

`state_update_condition_` is the conditional variable that's locking.


## IK not solve-able.

After trying many differnt things, I think the problem is at the IK solver. 

The regular joint constrained planning actually work (which is created when draginging robot around in rviz). However I can't drag the planning robot around with sphere, even with approximate IK. 

Same when sending planning from move_group_interface. 

I tried both `stretch_kinematics_plugins` package I could find, neither work.