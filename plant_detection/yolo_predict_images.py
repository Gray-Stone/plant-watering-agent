#! /usr/bin/python3

import argparse
import pathlib

from ultralytics import YOLO
import ultralytics.engine
import ultralytics.engine.results

import pathlib

if __name__ == "__main__":

    # Let user
    parser = argparse.ArgumentParser()
    parser.add_argument("target_img_dir" , type=pathlib.Path ,help="path to Yolo yaml file")
    parser.add_argument("--model" , default='yolov8n-seg.pt' , type=str ,help="model.pt file")

    args = parser.parse_args()
    target_image_dir : pathlib.Path = args.target_img_dir
    print(f"reading data file at {target_image_dir}")

    if not target_image_dir.exists():
        print(f"Target directory doesn't exists")
        exit(1)

    # Load the pretrained model

    model = YOLO(args.model , task="segmentation")
    # 58: 'potted plant' this is the existing class.
    # https://docs.ultralytics.com/reference/engine/model/#ultralytics.engine.model.Model.predict
    results: list[ultralytics.engine.results.Results] = model.predict(target_image_dir,save=True , )

    # current_path = pathlib.Path().resolve()

    # for r in results:
    #     file_path = pathlib.Path(r.path).resolve()
    #     # maybe save this.
    #     # r.show()
    #     print(f"masks \n{r.masks}")

def list_model_classes(model):
    print(f"Model have following classes: {model.names} ")
