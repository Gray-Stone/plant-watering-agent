
## numpy version


get this error when launching funmap 

```
[funmap-6]   File "/home/leogray/ament_ws/install/stretch_funmap/lib/python3.10/site-packages/stretch_funmap/numba_compare_images.py", line 1, in <module>
[funmap-6]     from numba import jit, njit
[funmap-6]   File "/usr/lib/python3/dist-packages/numba/__init__.py", line 205, in <module>
[funmap-6]     _ensure_critical_deps()
[funmap-6]   File "/usr/lib/python3/dist-packages/numba/__init__.py", line 145, in _ensure_critical_deps
[funmap-6]     raise ImportError("Numba needs NumPy 1.21 or less")
[funmap-6] ImportError: Numba needs NumPy 1.21 or less

```

So I downgrade the numpy to 1.21, but then 

```
hello-robot-stretch-body-tools 0.7.12 requires numpy==1.23.2, but you have numpy 1.21.0 which is incompatible.
```


```
[funmap-6] /home/leogray/ament_ws/install/stretch_funmap/lib/python3.10/site-packages/stretch_funmap/funmap.py:33: UserWarning: A NumPy version >=1.22.4 and <2.3.0 is required for this version of SciPy (detected version 1.21.0)
```



## funmap crashes


The crash is not deterministic. Seems to be related to ros threading as well as user input. There has been different crashes with different backtraces.


## No collision check

The procedure of running funmap is: 

* issue scan command.
    * Robot spin the camera head around, also move the base once, to get 360 deg scan.
* From the scan, build a point cloud of the environment around robot.
* For tele-op via rviz: 
    * User use Nav2d goal to move the base.
    * When published a clicked point, robot will move all joints, including base to reach the gripper onto that spot.

By default the arm will be at storage pose.

If the given position is on table top, the arm will extend and rise at the same time. And directly hit the btm side of the table. When looking from rviz, it is also obvious the arm is in contact with the point cloud of the table.


If look into the source code of `funmap.py`

Mouse click topic is handled by `FunmapNode.reach_to_click_callback`. Which does a IK to get robot's pose at the target using `FunmapNode.plan_to_reach`. Then drive the robot to the base XY using `FunmapNode.navigate_to_map_pixel` and `FunmapNode.move_to_pose` to move the arm.

None of the functions above did any collision checking.

