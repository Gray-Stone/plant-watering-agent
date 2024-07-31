
* Moveit needs more work to port over.

* Rtabmap is mostly figured out.

* Rtabmap, Nav2, and Moveit seems to independently building maps. Didn't see a good option to let them share the same octomap.

    * Nav2 should just directly take a map object. Turn off localization plugin.
    * Localmap should only be additional obstical. Could just turn it off.


Not worry about dynamic obstacle


Two option on motion planning 

* Fix moveit 
* Make custom motion planner. Let path planning to line up the base rotation with the plant. Then it's very few DOF to plan. It is also cartesian, so easy to do IK solving and do rectangle check the octomap to find plan.

* If the 2d map is free, it is always free in 3d. So base don't need 3d octomap checking. Just find a free space to place a big long rectangle that's the same cross section as the gripper. Spin the rectangle's tip around the plant to find a free spot, if the rectangle is collision free and reaching the empty space in 2D map, the rectangle also ensure the path of extending the arm. No path planning is needed, just solve one IK.