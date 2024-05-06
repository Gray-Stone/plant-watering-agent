
# Goal

The in-out on the detection side is actually sensor data in -> watering pose out.

Input data would most likely be rgbd camera, and output is a 6/4dof pose of where to put the water hose to water things.

## The plan

Existing pre-labeled ML data are 2d images. We want 3d segmentation of the point clouds of where the pot is.

The end to end ML of rgbd input and pose output will be hard to get labeled data, and hard to manually label.

The task is cut up as 
* Find 3d point clouds of the pot.
* Use the 3d point cloud, find a valid spot to water.

To find the 3d point cloud, the quickest solution would be let YOLO do segmentation on the 2d image. Then map the result mask to 3d point cloud on the rgbd camera. With knowing how the camera have moved, the following images will keep providing 3d point cloud that could be part of the pot. Accumulate these over time with probability could give us a confident map of the pot's voxels.

The known voxels for the pot is then put into point cloud processing to find the top rim of it. Then we find a way to check if a pose will be valid for the given rim.