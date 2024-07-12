
## Rtabmap

According to the topic name, Rtabmap should be already publishing octomap. It only show up after installing the octomap rivz plugin.

There are tones of topic rtabmap is publishing. 

The one that are `octomap_msgs/msg/Octomap` formatted are `/octomap_binary` and `/octomap_full`. The `octomap_full` is colored and take lots of bandwidth. The `/octomap_binary` does not have color info and is transmitted able over wifi.

The `/octomap_binary` given from rtabmap have the same probability for all voxels. Doesn't seem to have probability update. Also this output is not updating moved objects. (verified over test). Seems like this octomap is just a compressed representation of it's pointcloud map.



## Build octomap from rtabmap's local map

rtabmap does have a bunch of local maps

```
    /local_grid_empty: sensor_msgs/msg/PointCloud2
    /local_grid_ground: sensor_msgs/msg/PointCloud2
    /local_grid_obstacle: sensor_msgs/msg/PointCloud2
```

Which might be a good idea to make them into a octomap: 

https://answers.ros.org/question/215136/how-to-convert-point-cloud-data-to-map-using-octomap/
https://answers.ros.org/question/137186/storing-pointcloud2-type-msgs-in-an-octomap/


## 2D map

For a driving around navs, 2D map is what matter. 

rtabmap gives this `/grid_prob_map: nav_msgs/msg/OccupancyGrid`, which seems to be an updating map that will free up grids that object have moved away.


## Ground labeling.

The `stretch_rtabmap` launched an `rtabmap_slam::rtabmap` with no argument changed.

When visually inspecting the point cloud from `local_grid_ground`, we can see any flat surface is being counted towards a ground plane.

The problem with this is the 2D maps `grid_prob_map` also marked the area under the table not obstructed. If the `grid_prob_map` is directly feed to nav2, it would be very bad for navigation. Thus needs to find better way in general.

![alt text](medias/stretch-rtabmap-ground-voxels.png)