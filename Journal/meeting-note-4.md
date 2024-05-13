

https://pointclouds.org/documentation/classpcl_1_1_iterative_closest_point.html

Will stich multiple point clouds together. Works well with lots of overlap. No need to know the camera pose.



# slam

rgbd slam will give a full octomap. 

Use this map, segment out the flowerpot afterwords. this way only one copy of point cloud is used. and camera location is known. 


##
 Look at how Hello robot do slam


# E-E machine option

For each image, given the mask, manually add a dot on the 2d image as the desired watering point. 

Train extra layers on the existing segmentation network to give this watering point directly. With RGBD, this directly translate to a 3d point for watering.

Extra layer after the segmentation output, takes the mask and image in, output a watering dot.

# Action:

<!-- Look at Hello robot's slam. See how to do segmentation onto it. -->


Try the E-E model.