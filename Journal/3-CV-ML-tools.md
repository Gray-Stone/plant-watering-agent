
## Learning tool choice

YOLO seems to be the most commonly used tool for live detection.


Alternative to YOLO

clip model
https://github.com/facebookresearch/detectron2

## Dataset fetching

https://storage.googleapis.com/openimages/web/download_v7.html

## Dataset visualization  

https://github.com/voxel51/fiftyone



## 

Full end to end tool: cvat


## cvat

The yolo exporting from cvat is still downloading (even with locally hosted). Images does have .txt label file next to it, but the top level yaml doesn't exists.

**Putting cvat on hold for now until other methods has been blocked**


## fiftyone

Getting image from open-images-v7 and exporting to yolo kinda working using the following python code. With the problem of only exporting `val` splits.

```python
dataset: fiftyone.core.dataset.Dataset = foz.load_zoo_dataset(
    "open-images-v7",
    split="train",
    max_samples=100,
    classes=["Flowerpot"] , # vase does have some good image, but might not be best
    label_types=["detections","segmentations"],
    label_field="Flowerpot"
)
dataset.export(export_dir="../../image_fiftyone/yolo5",
               dataset_type=fiftyone.types.YOLOv5Dataset,
               classes=["Flowerpot"])
```

## trainYolo

This is a online only tool, exporting from trainYolo needs to use their commandline tool. 

`trainyolo project pull "pot" --format yolov8` However the exported data automatically have the splits. And I can manually set them on the online panel.

## AnyLabeling 

This is an offline tool, having auto labeling feature.

https://github.com/vietanhdev/anylabeling

There seems to be a fork-ish called x-anylabeling. which is very similar but slightly different. 

https://github.com/CVHub520/X-AnyLabeling

Second one has a lot lot more ML models to choose from.



The labeled output is a json file. Not sure the exact format, but most likely a labelme JSON format. Since labelme is an internal component and it does save as JSON.

Which there already have a project for it. https://pypi.org/project/labelme2yolo/

As a conclusion, This process seems to work, just the data.yaml need to be touched up where folder path have the folder itself in it.