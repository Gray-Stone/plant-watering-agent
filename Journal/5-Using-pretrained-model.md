
## Use the correct label ID

The pre-trained yolo model have 80 classes already, including the potted plant we want.


According to this post: https://github.com/ultralytics/ultralytics/issues/10057, to keep on adding to the pre-trained model with custom classes, one need to start with the id after last id in pre-trained module

I have forked `labelme2yolo` to add ability to specify the class id for it. https://github.com/Gray-Stone/labelme2yolo-mod.

A renaming problem with this is YOLO doesn't like it when label number jumped:

```
error ❌ '1-class dataset requires class indices 0-0, but you have invalid class indices 58-58 defined in your dataset YAML.'
```

So I need to copy the leading labels from COCO dataset into the file to pad it.
https://docs.ultralytics.com/datasets/detect/coco/#dataset-yaml

## Rename the label class


Here is the instruction for swapping out custom names.
https://github.com/ultralytics/ultralytics/issues/4956#issuecomment-2038655766

The catch is the renaming must be done from the tensor side. changeing it in YOLO doesn't affect anything. 


## Quick look at effectiveness of the pretrained model

Using just the pre-trained model, It is able to detect the plants to some degree.

This image show it's detection of the plant using just the pertained `yolov8n.pt` model.

![](medias/YOLO-pretrain-pottedplant-detect.png)

We can see it is doing ok on finding the plants, with the catch being it includes all the leaves part into it. This could be a problem in future when a plant's leave is super spread out, the detection might not be able to find the pot properly. (but this would then also depends on the point cloud cutting of things.)