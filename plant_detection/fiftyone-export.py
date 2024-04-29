#! /usr/bin/python3

import fiftyone as fo
import fiftyone.zoo as foz
import fiftyone.utils.openimages as foo
import fiftyone.core.dataset
import fiftyone.types

import fiftyone.utils.yolo as fy
# https://docs.voxel51.com/integrations/open_images.html#open-images


# https://docs.voxel51.com/user_guide/dataset_zoo/index.html

# Options are actually mostly in here:
# https://docs.voxel51.com/tutorials/open_images.html


# print(foo.get_classes())

# exit(1)
dataset: fiftyone.core.dataset.Dataset = foz.load_zoo_dataset(
    "open-images-v7",
    split="train",
    max_samples=100,
    classes=["Flowerpot"] , # vase does have some good image, but might not be best
    label_types=["detections","segmentations"],
    label_field="Flowerpot"
)

# fy.YOLOv5DatasetExporter(export_dir="../../image_fiftyone/" , split="train" , export_media="symlink",)

dataset.export(export_dir="../../image_fiftyone/yolo5",
               dataset_type=fiftyone.types.YOLOv5Dataset,
               classes=["Flowerpot"])
# dataset.export(export_dir="../../image_fiftyone/coco",dataset_type=fiftyone.types.COCODetectionDataset ,classes=["Flowerpot"])
