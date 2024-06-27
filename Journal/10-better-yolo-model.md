Two things to try: 

1. Two stage training, train flower pot first, then take the weights and train watering area next.

2. Directly train flowerpot and watering area together.


## Getting data

Problem with data: Have huge amount of pre-labeled flower pot data. But the watering area needs self labeling, which is lot less and slow. 

Watering area images are a sub-set of flowerpot. There needs to be background images. which means I can't use flowerpot image without vetting them first, so the large amount of pre-labeled image needs to be fully labeled with watering area before use.

Combined, images have these states:

1 No flower pot at all
2 Have flower pot, but no watering area
3 Have flower pot, also have watering area

Images directly downloaded is unknown for its flower pot status.

exported image from cvat gives data type 2 and 3, but missing the flowerpot label

So the best route to do is add flowerpot labels to the data export from cvat.

## Result:


Both option, continue training or a new model, after training with a combined flowerpot and watering-area labels (450 imgs), have reduced false positive. However neither model gives flowerpot label, which according to the post, it should kept all labels around.


Post about continue training. It should support continue training after combining all datasets.
https://github.com/ultralytics/ultralytics/issues/6340

Only thing missing from what I did is a nc in dataset.yaml