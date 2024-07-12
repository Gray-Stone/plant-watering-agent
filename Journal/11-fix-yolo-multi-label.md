
## Validate the process with pre-labeled data that doesn't require merging.

### New 

* Exported Flowerpot and Flower from Open images 7, 500 total 
* Do the coco to yolo segmentation conversion.
* Train a brand new seg model with this data.

Able to detect both flower and flowerpot in the same image.

This prove the new model training does work.


### continue

* Same data as before, 500 images 
* train on previously trained flowerpot model.

Same as above, it also is capable of labeling both.

this likely indicate the training setting is right.

### Reduced number of images.


When image number dropped down to 150, everything still worked!


### Test merging process

Make a yolo dataset with only flowerpot images.

Then merge in the dataset with flower labeled on it.


## File Check

### Need to check the filelist fiftyone exported are the same or not pre-iteration.

A rough quick check on a 100 image patch. They are the same.


### Check image data on disk

500 images from Open Images with "Flowerpot" segmentation are picked out to be labeled in cvat.

Around 150 of them have watering are label, the rest are considered "background" in watering area case, which have flowerpot but not watering area.

The merging process require taking Flowerpot labels from the original open images folder and add it to labels in the watering area output.

However the data management itself is a mess, so need to check if the merging of Flowerpot label actually covers all the images Watering area needs. 

Result is some labels are missing.

Somehow, there are 7 images exists in the watering area doesn't exits in the Flowerpot label.

These images are made up by exporting a lot extra images.