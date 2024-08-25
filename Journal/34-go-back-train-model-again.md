


To improve the training, we want to have yolo auto generate some of these augmentation.
https://github.com/ultralytics/yolov5/issues/5307

Here is a list of many many arguments.

https://docs.ultralytics.com/usage/cfg/#augmentation-settings


## Previous problem

The old model used was trained with all parameters set to default.

The model cannot reliably detect pots at a decent distance away, and is quite sensitive to the angle of the frame. 

## Parameters:

```
scale=0.3,
degrees=70,
perspective= 0.0005,
auto_augment= "autoaugment",
```

## Bigger dataset

I also included a much much bigger dataset to train a base model that's only doing flower pot detection (10k images) 

Then with this, I continue training it with the flowerpot+watering area dataset.

## result 

The model gets a lot a lot better, specially for detecting at distances.