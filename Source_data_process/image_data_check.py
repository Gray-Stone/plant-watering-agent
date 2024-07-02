#! /usr/bin/python3

import json
import os
import pathlib
import argparse

import dataclasses
from typing import Any, Callable
import yaml
import enum
import random
import copy

def WalkAllFiles( folder : pathlib.Path , file_cb: Callable):
    for item in folder.iterdir():
        if item.is_dir():
            WalkAllFiles(item , file_cb)
            continue
        file_cb(item)

class ImgCollector():

    def __init__(self) -> None:
        self.id_img_dict : map[str,pathlib.Path] = {}

    def TryAddImage(self,img_path:pathlib.Path):
        if img_path.suffix.lower() == ".txt":
            return
        stem = img_path.stem

        if stem in self.id_img_dict:
            print(f"!!! id {stem} already exists.\nOld {self.id_img_dict[stem]}\nNew {img_path} ")

        self.id_img_dict[stem] = img_path

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("folder", type=pathlib.Path, help="folder to find images")
    parser.add_argument("-c","--coco-check" , type=pathlib.Path, help="coco json to check if th images in folder exists in coco json")

    args = parser.parse_args()
    folder: pathlib.Path = args.folder

    collector = ImgCollector()
    WalkAllFiles(folder , collector.TryAddImage)

    for key,value in    collector.id_img_dict.items():
        print(f"{key} ,")

    print(f"Total of {len(collector.id_img_dict)} items")
    print()

    if args.coco_check is None:
        exit(0)
    coco_file :pathlib.Path = args.coco_check
    with open(coco_file,'r') as file:
        coco_data = json.load(file)

    img_folder_dict:dict = copy.deepcopy(collector.id_img_dict)

    coco_img_dict : dict[str,pathlib.Path] = {}
    shared_ids : set[str] = set()

    for img in coco_data['images']:
        file = pathlib.Path(img["file_name"])
        id = file.stem
        # Start checking files.
        # We assume no dupe.
        if id in img_folder_dict:
            # Delete if match, whatever remain is missing ones.
            img_folder_dict.pop(id)
            shared_ids.add(id)
            continue
        if id in shared_ids:
            print(f"id {id} showed up second time !")
        coco_img_dict[file.stem] = coco_file.parent / "data" / file

    # Now conclude results.

    print(f"{len(img_folder_dict)} item exists in source folder but not in coco")
    print(f"{len(coco_img_dict)} item exists in coco but not in source folder")

    print(f"img folder unique ids {img_folder_dict.keys()}")
    print(f"coco unique ids {coco_img_dict.keys()}")