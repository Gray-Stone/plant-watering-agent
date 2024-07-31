
## Issue with rtabmap 

After leave the rtabmap running for a while, the system give folloing error. 

```
[rtabmap-6] [INFO] [1721783929.930066976] [rtabmap]: rtabmap (442): Rate=1.00s, Limit=0.000s, Conversion=0.0008s, RTAB-Map=0.1183s, Maps update=0.0065s pub=0.0000s (local map=1, WM=1)
[rtabmap-6] [WARN] [1721783930.339449995] [rtabmap]: We received odometry message, but we cannot get the corresponding TF odom->base_link at data stamp 1721783930.049695s (odom msg stamp is 1721783930.045145s). Make sure TF of odometry is also published to get more accurate pose estimation. This warning is only printed once.
[rtabmap-6] [ERROR] [1721783931.275949544] [rtabmap]: Could not convert rgb/depth msgs! Aborting rtabmap update...
[rtabmap-6] [ WARN] (2024-07-23 18:11:23.174) MapsManager.cpp:863::publishMaps() Graph has changed! The whole cloud is regenerated.
[rtabmap-6] [ WARN] (2024-07-23 18:18:50.339) MsgConversion.cpp:1989::getTransform() (can transform odom -> base_link?) Lookup would require extrapolation into the future.  Requested time 1721783930.049695 but the latest data is at time 1721783930.001486, when looking up transform from frame [base_link] to frame [odom]. canTransform returned after 0.202511 timeout was 0.2. (wait_for_transform=0.200000)
[rtabmap-6] [ WARN] (2024-07-23 18:18:50.543) MsgConversion.cpp:1989::getTransform() (can transform odom -> base_link?) Lookup would require extrapolation into the future.  Requested time 1721783930.249839 but the latest data is at time 1721783930.087853, when looking up transform from frame [base_link] to frame [odom]. canTransform returned after 0.202147 timeout was 0.2. (wait_for_transform=0.200000)
[rtabmap-6] [ WARN] (2024-07-23 18:18:50.810) MsgConversion.cpp:1989::getTransform() (can transform odom -> base_link?) Lookup would require extrapolation into the future.  Requested time 1721783930.516696 but the latest data is at time 1721783930.087853, when looking up transform from frame [base_link] to frame [odom]. canTransform returned after 0.202307 timeout was 0.2. (wait_for_transform=0.200000)
[rtabmap-6] [ WARN] (2024-07-23 18:18:51.073) MsgConversion.cpp:1989::getTransform() (can transform odom -> base_link?) Lookup would require extrapolation into the future.  Requested time 1721783930.716840 but the latest data is at time 1721783930.087853, when looking up transform from frame [base_link] to frame [odom]. canTransform returned after 0.202206 timeout was 0.2. (wait_for_transform=0.200000)
[rtabmap-6] [ WARN] (2024-07-23 18:18:51.275) MsgConversion.cpp:1989::getTransform() (can transform base_link -> camera_color_optical_frame?) Lookup would require extrapolation into the future.  Requested time 1721783930.783555 but the latest data is at time 1721783930.001486, when looking up transform from frame [camera_color_optical_frame] to frame [base_link]. canTransform returned after 0.202308 timeout was 0.2. (wait_for_transform=0.200000)
[rtabmap-6] [ERROR] (2024-07-23 18:18:51.275) MsgConversion.cpp:2172::convertRGBDMsgs() TF of received image 0 at time 1721783930.783555s is not set!
[rtabmap-6] [ WARN] (2024-07-23 18:18:51.540) MsgConversion.cpp:1989::getTransform() (can transform odom -> base_link?) Lookup would require extrapolation into the future.  Requested time 1721783931.250551 but the latest data is at time 1721783930.087853, when looking up transform from frame [base_link] to frame [odom]. canTransform returned after 0.202446 timeout was 0.2. (wait_for_transform=0.200000)
[rtabmap-6] [ WARN] (2024-07-23 18:18:51.744) MsgConversion.cpp:1989::getTransform() (can transform odom -> base_link?) Lookup would require extrapolation into the future.  Requested time 1721783931.450692 but the latest data is at time 1721783930.087853, when looking up transform from frame [base_link] to frame [odom]. canTransform returned after 0.20237 timeout was 0.2. (wait_for_transform=0.200000)
[rtabmap-6] [ WARN] (2024-07-23 18:18:52.413) MsgConversion.cpp:1989::getTransform() (can transform odom -> base_link?) Lookup would require extrapolation into the future.  Requested time 1721783931.650831 but the latest data is at time 1721783930.087853, when looking up transform from frame [base_link] to frame [odom]. canTransform returned after 0.202212 timeout was 0.2. (wait_for_transform=0.200000)
[rtabmap-6] [ WARN] (2024-07-23 18:18:52.616) MsgConversion.cpp:1989::getTransform() (can transform odom -> base_link?) Lookup would require extrapolation into the future.  Requested time 1721783932.051112 but the latest data is at time 1721783930.087853, when looking up transform from frame [base_link] to frame [odom]. canTransform returned after 0.20291 timeout was 0.2. (wait_for_transform=0.200000)
[rtabmap-6] [ERROR] [1721783932.818845843] [rtabmap]: Could not convert rgb/depth msgs! Aborting rtabmap update...
[rtabmap-6] [ERROR] [1721783933.777202769] [rtabmap]: Could not convert rgb/depth msgs! Aborting rtabmap update...
[rtabmap-6] [ERROR] [1721783934.958387251] [rtabmap]: Could not convert rgb/depth msgs! Aborting rtabmap update...
[rtabmap-6] [ WARN] (2024-07-23 18:18:52.818) MsgConversion.cpp:1989::getTransform() (can transform base_link -> camera_color_optical_frame?) Lookup would require extrapolation into the future.  Requested time 1721783932.117827 but the latest data is at time 1721783930.001486, when looking up transform from frame [camera_color_optical_frame] to frame [base_link]. canTransform returned after 0.202428 timeout was 0.2. (wait_for_transform=0.200000)
[rtabmap-6] [ERROR] (2024-07-23 18:18:52.818) MsgConversion.cpp:2172::convertRGBDMsgs() TF of received image 0 at time 1721783932.117827s is not set!
[rtabmap-6] [ WARN] (2024-07-23 18:18:53.166) MsgConversion.cpp:1989::getTransform() (can transform odom -> base_link?) Lookup would require extrapolation into the future.  Requested time 1721783932.851696 but the latest data is at time 1721783930.087853, when looking up transform from frame [base_link] to frame [odom]. canTransform returned after 0.202337 timeout was 0.2. (wait_for_transform=0.200000)
[rtabmap-6] [ WARN] (2024-07-23 18:18:53.370) MsgConversion.cpp:1989::getTransform() (can transform odom -> base_link?) Lookup would require extrapolation into the future.  Requested time 1721783932.985127 but the latest data is at time 1721783930.087853, when looking up transform from frame [base_link] to frame [odom]. canTransform returned after 0.202716 timeout was 0.2. (wait_for_transform=0.200000)
[rtabmap-6] [ WARN] (2024-07-23 18:18:53.574) MsgConversion.cpp:1989::getTransform() (can transform odom -> base_link?) Lookup would require extrapolation into the future.  Requested time 1721783933.251983 but the latest data is at time 1721783930.087853, when looking up transform from frame [base_link] to frame [odom]. canTransform returned after 0.202439 timeout was 0.2. (wait_for_transform=0.200000)
[rtabmap-6] [ WARN] (2024-07-23 18:18:53.777) MsgConversion.cpp:1989::getTransform() (can transform base_link -> camera_color_optical_frame?) Lookup would require extrapolation into the future.  Requested time 1721783933.251983 but the latest data is at time 1721783930.001486, when looking up transform from frame [camera_color_optical_frame] to frame [base_link]. canTransform returned after 0.202484 timeout was 0.2. (wait_for_transform=0.200000)
[rtabmap-6] [ERROR] (2024-07-23 18:18:53.777) MsgConversion.cpp:2172::convertRGBDMsgs() TF of received image 0 at time 1721783933.251983s is not set!
[rtabmap-6] [ WARN] (2024-07-23 18:18:54.142) MsgConversion.cpp:1989::getTransform() (can transform odom -> base_link?) Lookup would require extrapolation into the future.  Requested time 1721783933.785696 but the latest data is at time 1721783930.087853, when looking up transform from frame [base_link] to frame [odom]. canTransform returned after 0.20228 timeout was 0.2. (wait_for_transform=0.200000)
[rtabmap-6] [ WARN] (2024-07-23 18:18:54.346) MsgConversion.cpp:1989::getTransform() (can transform odom -> base_link?) Lookup would require extrapolation into the future.  Requested time 1721783933.985837 but the latest data is at time 1721783930.087853, when looking up transform from frame [base_link] to frame [odom]. canTransform returned after 0.202769 timeout was 0.2. (wait_for_transform=0.200000)
[rtabmap-6] [ WARN] (2024-07-23 18:18:54.756) MsgConversion.cpp:1989::getTransform() (can transform odom -> base_link?) Lookup would require extrapolation into the future.  Requested time 1721783934.319411 but the latest data is at time 1721783930.087853, when looking up transform from frame [base_link] to frame [odom]. canTransform returned after 0.202188 timeout was 0.2. (wait_for_transform=0.200000)
[rtabmap-6] [ WARN] (2024-07-23 18:18:54.958) MsgConversion.cpp:1989::getTransform() (can transform base_link -> camera_color_optical_frame?) Lookup would require extrapolation into the future.  Requested time 1721783934.386126 but the latest data is at time 1721783930.343714, when looking up transform from frame [camera_color_optical_frame] to frame [base_link]. canTransform returned after 0.202218 timeout was 0.2. (wait_for_transform=0.200000)
[rtabmap-6] [ERROR] (2024-07-23 18:18:54.958) MsgConversion.cpp:2172::convertRGBDMsgs() TF of received image 0 at time 1721783934.386126s is not set!
[rtabmap-6] [WARN] [1721783935.479211432] [rtabmap]: rtabmap: Did not receive data since 5 seconds! Make sure the input topics are published ("$ ros2 topic hz my_topic") and the timestamps in their header are set. If topics are coming from different computers, make sure the clocks of the computers are synchronized ("ntpdate"). If topics are not published at the same rate, you could increase "queue_size" parameter (current=10).
[rtabmap-6] rtabmap subscribed to (approx sync):
[rtabmap-6]    /odom \
[rtabmap-6]    /camera/color/image_raw \
[rtabmap-6]    /camera/aligned_depth_to_color/image_raw \
[rtabmap-6]    /camera/color/camera_info
[rtabmap-6] [ERROR] [1721783936.330797729] [rtabmap]: Could not convert rgb/depth msgs! Aborting rtabmap update...
[rtabmap-6] [ERROR] [1721783937.681838703] [rtabmap]: Could not convert rgb/depth msgs! Aborting rtabmap update...
[rtabmap-6] [ WARN] (2024-07-23 18:18:55.161) MsgConversion.cpp:1989::getTransform() (can transform odom -> base_link?) Lookup would require extrapolation into the future.  Requested time 1721783934.386126 but the latest data is at time 1721783930.087853, when looking up transform from frame [base_link] to frame [odom]. canTransform returned after 0.202528 timeout was 0.2. (wait_for_transform=0.200000)
```


## Moveit 

Moveit rviz plugin able to show correct current robot pase and plan a new move, but Moveit rviz is missing the interactive marker at the EE


Able to ask for executing a plan, and the stretch_driver side show log of receiving command and executing.

```
[stretch_driver-3] -------------------------------------------
[stretch_driver-3] Finished goal
[stretch_driver-3] [INFO] [1721781432.356127379] [stretch_driver]: stretch_driver joint_traj action: traj succeeded!
[stretch_driver-3] [INFO] [1721782615.537647066] [joint_trajectory_action]: Received goal request
[stretch_driver-3] [INFO] [1721782615.544559264] [stretch_driver]: stretch_driver joint_traj action: new traj with 107 points over 10.54 seconds
[stretch_driver-3] -------------------------------------------
[stretch_driver-3] Finished goal
[stretch_driver-3] [INFO] [1721782627.499749228] [stretch_driver]: stretch_driver joint_traj action: traj succeeded!
```

When executing, the trajectory sent from move_group to stretch_driver. Stretch_driver did move the wrist, but didn't move the lift/arm or base at all.