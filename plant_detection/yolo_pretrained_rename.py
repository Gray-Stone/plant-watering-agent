#! /usr/bin/python3

from ultralytics import YOLO
import torch


if __name__ == "__main__":
    TARGET_NAME = "Flowerpot"

    model = YOLO('yolov8n.pt')
    print(f"old class names :\n{model.names}")
    map_index = 58
    for index,name in enumerate(model.names):
        if name =="potted plant":
            map_index = index

    print(f"Mapping {map_index} from {model.names[map_index]} to {TARGET_NAME}")

    model = torch.load("yolov8n.pt", map_location="cpu")

    model["model"].names[map_index] = TARGET_NAME

    torch.save(model, "renamed_yolov8n.pt")

    model = YOLO('renamed_yolov8n.pt')
    print(f"new class name {model.names}")
