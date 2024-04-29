#! /usr/bin/python3

# python3 -m pip install ultralytics


import argparse
import pathlib
parser = argparse.ArgumentParser()

parser.add_argument("data_yaml" , type=pathlib.Path ,help="path to Yolo yaml file")

args = parser.parse_args()

data_yaml_path : pathlib.Path = args.data_yaml

print(f"reading data file at {data_yaml_path}")

if not data_yaml_path.is_file():
    print(f"Given path {data_yaml_path} => {data_yaml_path.resolve()} is not a file")
    exit(1)


# Datasets


# From ultralytics example
from ultralytics import YOLO

# Create a new YOLO model from scratch
model = YOLO('yolov8n.yaml')

# Load a pretrained YOLO model (recommended for training)
model = YOLO('yolov8n.pt')

# Train the model using the 'coco8.yaml' dataset for 3 epochs
# Note: If given relative path, yolo will always assume the relative path start with this file's location. 
# Unless the content of the data.yaml have a "path" key pointing to destination.
results = model.train(data=data_yaml_path.resolve(), epochs=3 , val=False)

# Evaluate the model's performance on the validati on set
# results = model.val()

# Perform object detection on an image using the model
# results = model('https://ultralytics.com/images/bus.jpg')

model()
import cv2
im2 = cv2.imread("../../image_set_manual_collect/test_target/home_many.png")

# https://docs.ultralytics.com/modes/predict/#working-with-results
results = model.predict(source=im2, save=True, save_txt=True)  # save predictions as labels

for r in results:
    r.show()

print("All Done")
while True:
    pass

# Export the model to ONNX format
# success = model.export(format='onnx')



# From train yolo example
# IM_SIZE = 640
# N_EPOCHS = 50 # you can lower this if you have a large dataset
# BATCH_SIZE = 2 # you can increase this if you have a large dataset, > 50 images, you can increase this to 4, >100 images you can increase this to 8
# MODEL = 'yolov8n-seg.pt'
# !yolo task=segment mode=train \
#       model=$MODEL data="{project_location}/dataset.yaml" \
#        epochs=$N_EPOCHS imgsz=$IM_SIZE batch=$BATCH_SIZE
