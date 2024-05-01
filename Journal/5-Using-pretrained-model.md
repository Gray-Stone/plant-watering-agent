
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