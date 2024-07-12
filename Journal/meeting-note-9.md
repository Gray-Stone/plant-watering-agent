## problem 

### FUNMAP
FUNMAP doesn't do collision check. So we can't use that.

Option now is to fall back to rtabmap with moveit.

moveit does seems to have a planning with mobile manipulator feature. 

https://moveit.picknik.ai/humble/doc/examples/mobile_base_arm/mobile_base_arm_tutorial.html

### Bandwidth

Using WIFI to transport pointclouds will likely hog all ros node's cycle time.

Have the option of point cloud transport.

Maybe the actual octomap would be memory efficient and can be transported over wifi?


## Planned Action

* Check what's in `stretch_ros::stretch_octomap` package, see if it generates a good env-mapping.

* Get moveit working.

* Draw the work flow diagram