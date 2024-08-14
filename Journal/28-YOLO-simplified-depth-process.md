## Alternative simpler method.

single point space centroid tracking.


Very simple option is to just get a single centroid point from each mask, use depth to find this single point's space loc, then keep merging space-centroid point. 

Compare to the masked depth-image to form segmentation point cloud, and merge point clouds. This is a lot a lot faster.

With the simple single space centroid method.

The logic for merging is simply check if the new space centroid is close enough to the existing point. Simply add it when its close enough.

The constrain or pre-request is each object cannot be too close to each other. the detection error margin + threshold need to be bigger then actual distance between two object.



## Finding a center for watering area.

The watering area is usually a U shape. Which mean the centroid will be outside the mask. This could cause the depth to land on random thing like leaves and be miles away from the actual area.

what need to happen is using skeleton technique to find centroid instead.

https://homepages.inf.ed.ac.uk/rbf/HIPR2/skeleton.htm