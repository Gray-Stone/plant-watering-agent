
# Auto Plant Watering with Mobile Manipulation

This is a project aimed at using mobile manipulation robot: hello-robot stretch3, to autonomously discover and water potted plants in home/office environment.

Project post: [Autonomous-Plant-Watering-with-Mobile-Manipulator](https://portfolio.limerobotlab.com/nw/autonomous-plant-watering-with-mobile-manipulator/)

--- 

## Dependencies

This project is designed to run on the stretch3 robot. 

This repo holds the packages for top level robot control and scripts for YOLO model training and runtime processing.

Dependencies include:
* ultralytics: pip package for YOLO training and runtime
* fiftyone: pip package for fetching OpenImages7 datasets 
* coco2yolo-seg: my custom [forked version](https://github.com/Gray-Stone/coco2yolo-seg-mod)
* stretch_ros2: Stretch robot driver with [a fork containing my custom changes](https://github.com/Gray-Stone/stretch_ros2_custom). This package basically only work when on stertch3 robot as URDF are local to each robot.

And also other common ROS2 packages like `rtabmap` `nav2` etc. 



## Folder structure 
```
./
├── Journal: detail development journals documented design decisions and pitfalls. 
├── plant_detection: YOLO related scripts for training and validating models
├── ros-ws: 
│   └── stretch_mover: ros package hosting top level state machines that controls robot, and script to apply yolo with depth camera.
├── Source_data_process: Helper scripts for data source gathering and processing
└── Trained_Models: Various YOLO training output.
```

## YOLO

Object detection is a significant part of this project. For more detail on how to use the scripts I create dataset, train, and use models, see ![plant_detection/README.md](plant_detection/README.md)

To get YOLO integrate into ROS, cv_bridge is the key. Then to convert from a segmentation to object's location in the world with RGBD camera, I simply find the centroid pixel's location, and use the depth image to find out this point in the world. This usually result to a point on the surface of the object, but is good enough for this project.

To see the detail implementation: checkout [stretch_mover/launch/yolo_subsystem.py](ros-ws/stretch_mover/launch/yolo_subsystem.py) and the two nodes it depends on.


## Running this project

**This repo is suppose to be running on the stretch3 robot!**

Clone this repo anywhere on the stretch3, navigate into `ros-ws` and build the workspace.

The following instruction are in [stretch_mover/README.md](ros-ws/stretch_mover/README.md)


## Moveit

It will be natural to think moveit should be able to plan for the arm movement, specially since moveit actually have a [tutorial](https://moveit.picknik.ai/humble/doc/examples/mobile_base_arm/mobile_base_arm_tutorial.html) for it.

However, this tutorial is for stretch2. I have spent a lot of time trying to get it work on stretch3 but I didn't get all the way through. [Here is my repo](https://github.com/Gray-Stone/stretch3-moveit)

The fundamental problem is the IK solver: StretchKinematicPlugin doesn't work for stretch3. 

Stretch3 is now full 8DOF capable including base, so it should be able to reach any 6DOF pose within work envelop without a passive DOF. But the arm part is only 5DOF. Which mean it can't be simply split up to solving the arm then moving the base styled solver. All 8 joints need to be considered together.

I have got a little success with the PickIK solver, when I create "fake" joints in URDF for moveit side only, to pretend an extra rotation joint at the base center. However doing this meaning changing how stretch driver interprets trajectory, and I'm not sure if this fake joint won't cause other things to broke.

With the tight timeline of the project, I didn't get to further explore the moveit possibility. But I think it should be capable. 