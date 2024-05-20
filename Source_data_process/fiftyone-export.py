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
import argparse
import pathlib

parser = argparse.ArgumentParser()
parser.add_argument("output_dir" , type=pathlib.Path ,help="path to Yolo yaml file")

args = parser.parse_args()
output_dir : pathlib.Path = args.output_dir


dataset: fiftyone.core.dataset.Dataset = foz.load_zoo_dataset(
    "open-images-v7",
    split="train",
    max_samples=1000,
    classes=["Flowerpot"] , # vase does have some good image, but might not be best
    label_types=["segmentations"],
    label_field="Flowerpot"
)

# session = fo.launch_app(dataset)
# while True:
#     pass


# print(f"label _fields { dataset._get_label_field_type('Flowerpot')}")

dataset.export(export_dir=str(output_dir),
               dataset_type=fiftyone.types.COCODetectionDataset,
               progress = True,
               classes=["Flowerpot"],
               )
# dataset.export(export_dir="../../image_fiftyone/coco",dataset_type=fiftyone.types.COCODetectionDataset ,classes=["Flowerpot"])
