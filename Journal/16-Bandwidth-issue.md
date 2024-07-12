# Bandwidth issue.

## large point cloud

Funmap just directly gives out a point cloud, which take a few minute to show up on remote machine.

Maybe we could make it into a octomap then send it? would octomap save more space?
https://answers.ros.org/question/215136/how-to-convert-point-cloud-data-to-map-using-octomap/

Point cloud transport is also a thing

https://discourse.ros.org/t/point-cloud-transport-is-here/31990



## Tried octomap package

Have tried octomap package. This package have `COLCON_IGNORE` in the folder, and is not mentioned in any of hello robot's published document.

To experiment with this, I change the stretch_body node's mode to gamepad to able to drive the robot around.

The octomap generated looks ok. However there are no slam in this, which means everything is based on 

With the default setting and default rviz config from their octomap package, The bandwidth issue is still not solved. As soon as I launch a rviz on remote machine, the systems starts to be very not responsive. This is because the default rviz can only take pointcloud format, which is much larger then octomap. 

When installed `octomap_rviz_plugins`, rviz is then able to display binary octomap. Which is a lot better in terms of bandwidth usage. When using rviz like this, the system is still responsive (occasional jitter)


