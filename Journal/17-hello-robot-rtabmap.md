
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


## Elevation map

`/elevation_map: grid_map_msgs/msg/GridMap`

The elevation map gives a 2.5D mesh ![alt text](medias/stretch-elevation-map.png). basically took the max height at each grid-cell location. Problem with this is as soon as camera saw the celling, the celling value overwhelm everything else.

On the plus side, this map is updated over time, which mean after looking at celling, if it look at ground for another long period, it will refresh the values.


## Rtabmap arguments

**Rtabmap ros1 and ros2 are very different in structure**

Tracking the `rtabmap_slam::CoreWrapper.cpp` code, the params are write into a private member of type `rtabmap::ParametersMap`. The source code of this `ParametersMap` can be found online `https://docs.ros.org/en/noetic/api/rtabmap/html/Parameters_8h_source.html`. It have about 800 lines of parameters, with most having a line of description. Starting at line 820 is the OccupancyGrid and Octomap params, which is what we need to solve issues.

Params I think worth using

**RGBD-SLAM section:**

```
RTABMAP_PARAM(RGBD, LinearUpdate,             float, 0.1,  "Minimum linear displacement (m) to update the map. Rehearsal is done prior to this, so weights are still updated.");
RTABMAP_PARAM(RGBD, AngularUpdate,            float, 0.1,  "Minimum angular displacement (rad) to update the map. Rehearsal is done prior to this, so weights are still updated.");
```
Changing this two to be smaller will help seeing map update during debug

```
RTABMAP_PARAM(RGBD, CreateOccupancyGrid,          bool, false,  "Create local occupancy grid maps. See \"Grid\" group for parameters.");
```
Might need to turn this on to generate a local map for Nav2 

**Odometry Section:**

```
RTABMAP_PARAM(Odom, Holonomic,              bool, true,   "If the robot is holonomic (strafing commands can be issued). If not, y value will be estimated from x and yaw values (y=x*tan(yaw)).");
```
For our diff drive robot, this might need to be turned off.

**Occupancy Grid Section** 
```
RTABMAP_PARAM(Grid, RangeMin,                float,  0.0,     "Minimum range from sensor.");
RTABMAP_PARAM(Grid, RangeMax,                float,  5.0,     "Maximum range from sensor. 0=inf.");
```
Since the realsense camera accuracy drop over distance, might want to constrain the max range. Also default 5 meter might be too large for indoor, could consider using a smaller value.

```
RTABMAP_PARAM(Grid, FootprintLength,         float,  0.0,     "Footprint length used to filter points over the footprint of the robot.");
RTABMAP_PARAM(Grid, FootprintWidth,          float,  0.0,     "Footprint width used to filter points over the footprint of the robot. Footprint length should be set.");
RTABMAP_PARAM(Grid, FootprintHeight,         float,  0.0,     "Footprint height used to filter points over the footprint of the robot. Footprint length and width should be set.");
```
Should pick a good value for these, so the stretch robot itself, specially the base is not count as obstacle 

```
RTABMAP_PARAM(Grid, CellSize,                float,  0.05,    "Resolution of the occupancy grid.");
RTABMAP_PARAM(Grid, PreVoxelFiltering,       bool,   true,    uFormat("Input cloud is downsampled by voxel filter (voxel size is \"%s\") before doing segmentation of obstacles and ground.", kGridCellSize().c_str()));
```
Change this when grid is not fine enough for navigation. Not sure what effect does voxel filtering have on grid.

```
RTABMAP_PARAM(Grid, NormalsSegmentation,     bool,   true,    "Segment ground from obstacles using point normals, otherwise a fast passthrough is used.");
RTABMAP_PARAM(Grid, MaxObstacleHeight,       float,  0.0,     "Maximum obstacles height (0=disabled).");
RTABMAP_PARAM(Grid, MinGroundHeight,         float,  0.0,     "Minimum ground height (0=disabled).");
RTABMAP_PARAM(Grid, MaxGroundHeight,         float,  0.0,     uFormat("Maximum ground height (0=disabled). Should be set if \"%s\" is false.", kGridNormalsSegmentation().c_str()));
RTABMAP_PARAM(Grid, MaxGroundAngle,          float,  45,      uFormat("[%s=true] Maximum angle (degrees) between point's normal to ground's normal to label it as ground. Points with higher angle difference are considered as obstacles.", kGridNormalsSegmentation().c_str()));
RTABMAP_PARAM(Grid, NormalK,                 int,    20,      uFormat("[%s=true] K neighbors to compute normals.", kGridNormalsSegmentation().c_str()));
RTABMAP_PARAM(Grid, ClusterRadius,           float,  0.1,     uFormat("[%s=true] Cluster maximum radius.", kGridNormalsSegmentation().c_str()));
RTABMAP_PARAM(Grid, MinClusterSize,          int,    10,      uFormat("[%s=true] Minimum cluster size to project the points.", kGridNormalsSegmentation().c_str()));
RTABMAP_PARAM(Grid, FlatObstacleDetected,    bool,   true,    uFormat("[%s=true] Flat obstacles detected.", kGridNormalsSegmentation().c_str()));
```
These are important parameter to generate a good map.

If `NormalsSegmentation` is off, table top will certainly not be count as a floor. However, if we set `MaxGroundHeight`, might be able to keep this around without effecting much.

`MaxObstacleHeight` should be set to some amount higher then robot. This way celling won'be count as obstacle.

Other parameters here are less important be related. Not sure what `FlatObstacleDetected` is doing.


**OctoMap Section**

```
RTABMAP_PARAM(Grid, 3D,                      bool,   true,    uFormat("A 3D occupancy grid is required if you want an OctoMap (3D ray tracing). Set to false if you want only a 2D map, the cloud will be projected on xy plane. A 2D map can be still generated if checked, but it requires more memory and time to generate it. Ignored if laser scan is 2D and \"%s\" is 0.", kGridSensor().c_str()));
```
This actually live in a compile flag for default true and false. We should force this to be true. 

```
RTABMAP_PARAM(Grid, GroundIsObstacle,           bool,   false,   uFormat("[%s=true] Ground segmentation (%s) is ignored, all points are obstacles. Use this only if you want an OctoMap with ground identified as an obstacle (e.g., with an UAV).", kGrid3D().c_str(), kGridNormalsSegmentation().c_str()));
RTABMAP_PARAM(Grid, NoiseFilteringRadius,       float,   0.0,    "Noise filtering radius (0=disabled). Done after segmentation.");
RTABMAP_PARAM(Grid, NoiseFilteringMinNeighbors, int,     5,      "Noise filtering minimum neighbors.");
```
`GroundIsObstacle` should be off. The filtering param could be useful if the octomap become super noisy.

```
RTABMAP_PARAM(Grid, RayTracing,                 bool,   false,   uFormat("Ray tracing is done for each occupied cell, filling unknown space between the sensor and occupied cells. If %s=true, RTAB-Map should be built with OctoMap support, otherwise 3D ray tracing is ignored.", kGrid3D().c_str()));
```
This is what really sets octomap apart, should consider enabling this.

```
RTABMAP_PARAM(GridGlobal, OccupancyThr,         float,  0.5,     "Occupancy threshold (value between 0 and 1).");
RTABMAP_PARAM(GridGlobal, ProbHit,              float,  0.7,     "Probability of a hit (value between 0.5 and 1).");
RTABMAP_PARAM(GridGlobal, ProbMiss,             float,  0.4,     "Probability of a miss (value between 0 and 0.5).");
RTABMAP_PARAM(GridGlobal, ProbClampingMin,      float,  0.1192,  "Probability clamping minimum (value between 0 and 1).");
RTABMAP_PARAM(GridGlobal, ProbClampingMax,      float,  0.971,   "Probability clamping maximum (value between 0 and 1).");
RTABMAP_PARAM(GridGlobal, FloodFillDepth,       unsigned int, 0, "Flood fill filter (0=disabled), used to remove empty cells outside the map. The flood fill is done at the specified depth (between 1 and 16) of the OctoMap.");
```
These are octomap specific parameters.


**When using these parameter, remember to set them as string**

Basically all parameter that need to be pass through to the rtabmap itself needs to be quoted into string. Other ros related parameters don't need this treatment.

For example:
```
    rtabmap_parameters = {
        "RGBD/LinearUpdate": '0.01',
        "RGBD/AngularUpdate": '0.01',

        "Odom/Holonomic": 'False',
        "Grid/RangeMax": '4.0',
        
        "Grid/MaxObstacleHeight": '2.0',
        "Grid/MaxGroundHeight": '0.01',
        "Grid/RayTracing": 'True',
    }
```