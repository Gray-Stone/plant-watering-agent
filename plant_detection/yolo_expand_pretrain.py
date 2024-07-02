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
    parser.add_argument("base_model", type = pathlib.Path , help="existing model file")
    parser.add_argument("data_yaml" , type=pathlib.Path ,help="path to Yolo yaml file")
    parser.add_argument("--proj" , type=str , help="project name" ,default="train-cont")
    parser.add_argument("--name" , type=str , help="instance name" ,)

    args = parser.parse_args()
    data_yaml_path : pathlib.Path = args.data_yaml
    base_model : pathlib.Path = args.base_model

    print(f"using base model {base_model} ")
    print(f"reading data file at {data_yaml_path}")

    if not base_model.is_file():
        print(f"Given path {base_model} is not a file")
        exit(1)


    if not data_yaml_path.is_file():
        print(f"Given path {data_yaml_path} => {data_yaml_path.resolve()} is not a file")
        exit(1)

        

    model = YOLO(model=base_model)
    # https://docs.ultralytics.com/reference/engine/model/#ultralytics.engine.model.Model.train
    model.train(
        data=data_yaml_path.resolve(),
        epochs=300,
        project=args.proj,
        name=args.name,

        # WTF is this ? 
        pretrained = False,
        save=True,
        save_period=5,
        plots = True,
    )

    list_model_classes(model)
    model.save("cont_training.pt")


