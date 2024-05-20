#! /usr/bin/python3

import argparse
import pathlib

from ultralytics import YOLO
import ultralytics.engine
import ultralytics.engine.results

import pathlib

def list_model_classes(model):
    print(f"Model have following classes: {model.names} ")


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("data_yaml" , type=pathlib.Path ,help="path to Yolo yaml file")

    args = parser.parse_args()
    data_yaml_path : pathlib.Path = args.data_yaml

    print(f"reading data file at {data_yaml_path}")

    if not data_yaml_path.is_file():
        print(f"Given path {data_yaml_path} => {data_yaml_path.resolve()} is not a file")
        exit(1)


    # WTF?  YOLO' model does not support '_new' mode for 'segmentation' task yet.
    model = YOLO('yolov8n-pose.yaml')
    # https://docs.ultralytics.com/reference/engine/model/#ultralytics.engine.model.Model.train
    model.train(
        data=data_yaml_path.resolve(),
        epochs=200,
        project="keypoint_train",
        pretrained = True,
        save=True,
        save_period=5,
        plots = True,
    )

    # If I used     model = YOLO('yolov8n-seg.yaml'), then I can't save
    
    model.save("keypoint_train.pt")
    list_model_classes(model)

