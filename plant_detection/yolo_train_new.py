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
    parser.add_argument("--proj" , type=str , help="project name" ,default="train-new")
    parser.add_argument("--name" , type=str , help="instance name" ,)
    parser.add_argument("-e" , "--epoch" , type=int , help="number of epoch to train" ,default="100")
    parser.add_argument("--seg" , action="store_true" , help="do segmentation when set" ,default=False)
    parser.add_argument("--device" , help="yolo device argument" ,default="0")


    args = parser.parse_args()
    data_yaml_path : pathlib.Path = args.data_yaml

    print(f"reading data file at {data_yaml_path}")

    if not data_yaml_path.is_file():
        print(f"Given path {data_yaml_path} => {data_yaml_path.resolve()} is not a file")
        exit(1)


    # WTF?  YOLO' model does not support '_new' mode for 'segmentation' task yet.

    if args.seg:
        print("Training segmentation model")
        model = YOLO('yolov8n-seg.yaml')
    else: 
        print("Training bbox model")
        model = YOLO('yolov8n.yaml')
    # https://docs.ultralytics.com/reference/engine/model/#ultralytics.engine.model.Model.train
    model.train(
        data=data_yaml_path.resolve(),
        epochs=args.epoch,
        project=args.proj,
        name=args.name,
        device = args.device,
        pretrained = True,
        save=True,
        save_period=5,
        plots = True,
        scale=0.3,
        degrees=70,
        perspective= 0.0005,
        auto_augment= "autoaugment",
    )

    # If I used     model = YOLO('yolov8n-seg.yaml'), then I can't save
    list_model_classes(model)
    model.save("newly_trained.pt")

