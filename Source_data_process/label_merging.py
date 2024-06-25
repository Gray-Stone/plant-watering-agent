#! /usr/bin/python3

'''
| Case | Dest a.png | Dest a.txt | Source a.png | Source a.txt | What to do     |
| ------ | ------------ | ------------ | -------------- | -------------- | ---------------- |
| 1    | yes        | yes        | yes          | yes          | Merge          |
| 2    | yes        | no         | yes          | yes          | new label_file |
| 3    | no         | yes        | yes          | yes          | Not possible   |
| 4    | no         | no         | yes          | yes          | New            |
| 5    | --         | --         | yes          | no           | Nothing        |
| 6    | --         | --         | no           | yes          | not possible   |
| 7    | --         | --         | no           | no           | Nothing        |

Only case 1 2 and 4 needs handling. Case 3 and 6 are no possible if dataset is properly formed.

Case 1 merging, should be able to just add each line of source a.txt to dest a.txt.
However, we might want to check if the line to add is exactly the same, in which case, we should skip adding duplicated line.

When merging, if source and dest have same train/val, then it's good.
if mis-match, use the dest's value.

Case 2 and 4, the file is added for the firs time, create the file accordingly.
When Creating, if image exists, make label next to it (train/val). 
If image doesn't exists, do it base on source's type, 

'''
# Target dataset.yaml bust be given. Specially the classes list, this is hard to figure out.

# Can take a file list as "filter" list, aka white-list for merging. All other images are ignored.




# Case
#


import json
import os
import pathlib
import argparse

import dataclasses
from typing import Any, Optional
import yaml
import enum
import random

# This is still used for folder name.
class ImageType(enum.Enum):
    VAL = "val"
    TRAIN = "train"

# Do ImageType("val") to get a enum value out. If the given value doesn't hit, will throw.

@dataclasses.dataclass
class ImageInfo():
    image_file :pathlib.Path
    label_file :Optional[pathlib.Path] = None

def GetImageType(image_path : pathlib.Path):

    type_folder = image_path.parent.name
    value = ImageType.TRAIN
    try:
        value = ImageType(type_folder)
    except Exception as e :
        print(f"{e} when sorting {image_path}")

    return value

class DataSet():
    """
    Assume following structure of yaml

    names:
    0: Flowerpot
    1: Water-area
    train: ./images/train
    val: ./images/val

    For simple ness Note: assume image and label are placed right next to each other.
    # TODO do a iteration of all sub-folder's path + file name to search for label file.
    """
    # Structure of the yaml file:


    def __init__(self, dataset_yaml: pathlib.Path) -> None:
        self.yaml_path = dataset_yaml.resolve()
        self.root_path = self.yaml_path.parent

        dataset_content = {}
        with open(self.yaml_path , "r") as file:
            dataset_content = yaml.safe_load(file)

        print(f"Dataset content: \n{dataset_content}")
        self.class_lists = dataset_content["names"]

        self.train_dir: pathlib.Path = self.root_path / dataset_content["train"]
        # assume we always have val and train.
        self.val_dir: pathlib.Path = self.root_path / dataset_content["val"]




        # Go through and find all the files

        self.train_id_files = self.build_id_file_dict(self.train_dir)
        self.val_id_files = self.build_id_file_dict(self.val_dir)

    def build_id_file_dict(self , image_search_folder:pathlib.Path):
        id_file_dict = {}
        for f in image_search_folder.iterdir():
            if f.suffix.lower() == ".txt":
                continue
                # skip this. We ignore file with label but no image.
            # Any other file will be count as image.
            id = f.stem
            if id in id_file_dict:
                raise RuntimeError(f"Duplication under {image_search_folder} Image {f} already exists ! with {id_file_dict[id]}")            
            # Find the corresponding file 
            maybe_label = self.find_label_file(id)
            id_file_dict[id] = ImageInfo(f, maybe_label)

        return id_file_dict

    def find_label_file(self,id):
        maybe_files =list(self.root_path.glob(f"**/{id}.txt"))
        if maybe_files:
            return maybe_files[0]
        else:
            return None

def CollectAllEntries(data_sets :list[DataSet]):

    for ds in data_sets:
        # Get all images files
        # Get all corresponding data file.
        # Save to dict 

        pass


def MoveDataSet(source_data_set : DataSet):
    # We always try to go through training dataset first.

    # a.png and a.txt could cause duplicated process, so cache things.
    processed_names = {} # dict for quick insert and look up.
    for pic in source_data_set.train_dir.iterdir():
        pic_id= pic.stem
        if pic_id in processed_names:
            # skip already processed picture
            continue



if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument("-i" , "--input_yamls", nargs='+', type=pathlib.Path, help="path to input source dataclass yaml")

    parser.add_argument("output_yaml", type=pathlib.Path, help="path to dataclass yaml of destination")
    # We assume all segmentation now.
    # parser.add_argument("--bbox" , default=False , action="store_true" , help="Default doing segmentation, if set, then do bounding box.")


    args = parser.parse_args()

    input_datasets :list[DataSet] = []

    for dataset_yaml in args.input_yamls:
        dataset_yaml:pathlib.Path
        if not dataset_yaml.is_file:
            print(f"input dataset_yaml {dataset_yaml} is not a file")
            exit(1)

        input_datasets.append(DataSet(dataset_yaml))
        print(f"Taking source dataset from {dataset_yaml}")
    print(f"Dict: {input_datasets[0].val_id_files}")
    output_yaml :pathlib.Path = args.output_yaml.absolute()
    if not output_yaml.is_file():
        print(f"Output yaml {output_yaml} is not a file")
        exit(1)
    output_dataset = DataSet(output_yaml)
    print(f"Merging into dataset {output_yaml}")

