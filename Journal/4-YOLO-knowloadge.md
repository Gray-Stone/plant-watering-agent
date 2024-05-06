
# New model 

tutorial:

```
# Create a new YOLO model from scratch
model = YOLO('yolov8n.yaml')
```

https://github.com/ultralytics/ultralytics/issues/2328



# Pretrained YOLO model

YOLOv8 Detection, segmentation are pre-trained on COCO dataset.

The classes in COCO contains `potted plant` and `vase` which are both what we want for detection.

## Yolo validation

Yolo made validation a requirement. However, it is not used to train the weights. It is used to generate metrics after training. 

https://github.com/ultralytics/yolov5/issues/7655

This link https://github.com/ultralytics/yolov5/discussions/6023 Have similar problem as mine. The real data, is actual pot the robot need to find. However they will not be put into training process. So the resulting model will be very general and more useful.



## Recommendation on settings for training 

https://docs.ultralytics.com/yolov5/tutorials/tips_for_best_training_results/#training-settings

### Background images. 

Recommendation from Ultralytics: 
> Background images are images with no objects that are added to a dataset to reduce False Positives (FP). We recommend about 0-10% background images to help reduce FPs (COCO has 1000 background images for reference, 1% of the total). No labels are required for background images.


# Key settings might need to change. 

## Train

Arguments to Training.

https://docs.ultralytics.com/usage/cfg/#train-settings

* `patience` Seems like validation is used for early stopping during training.

This means I should re-consider using real-stage image in validation, as those might give-away the stopping point.

* `save_period` default to -1 (off) I think this should always be on to actually save some stuff.

* `project` Project directory to save training output. better organize storage of different experiments.

* `name` name of training run, subdir of project.

* `resume` Auto resume from last checkpoint, Should have it on.

* `freeze` freeze first N layer of model. Use this when fine-tuning.

* `val` default True. validation during training. 

Should I be turning this off? 

* `plots` defualt False, Generate and save plots of training and validation metrics.


## Predict

`save` save the annotated images or videos to file. 

This should be part of the work flow for evaluating model after train.

# Note for segmentation:

Model for segmentation is done with `-seg` sufix : `yolov8n-seg.yaml`