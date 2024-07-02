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
parser.add_argument("-s" ,"--samples", type=int ,help="number of samples" , default = 500)

args = parser.parse_args()
output_dir : pathlib.Path = args.output_dir
max_samples=args.samples

print(f"Exporting {max_samples} samples to {output_dir}")



# class_list = ["Flowerpot" , "Flower"]
class_list = ["Flowerpot"]
# class_list = ["Flower"]

# https://docs.voxel51.com/user_guide/dataset_zoo/datasets.html#dataset-zoo-open-images-v7
dataset: fiftyone.core.dataset.Dataset = foz.load_zoo_dataset(
    "open-images-v7",
    split="train",
    max_samples=args.samples,
    classes=class_list , # vase does have some good image, but might not be best
    label_types=["segmentations"],
    label_field="Flowerpot"
)

# Don't try to directly do bbox with yolo either. The output is always val only.
dataset.export(export_dir=str(output_dir),
               dataset_type=fiftyone.types.COCODetectionDataset,
               progress = True,
               classes=class_list,
               )
