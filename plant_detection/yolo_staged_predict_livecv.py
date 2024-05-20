#! /usr/bin/python3

import argparse
import pathlib

from ultralytics import YOLO
import ultralytics.engine
import ultralytics.engine.results

import pathlib

import cv2
import math
import numpy as np

# https://github.com/ultralytics/ultralytics/issues/561
def overlay(image, mask, color, alpha, resize=None):
    """Combines image and its segmentation mask into a single image.
    https://www.kaggle.com/code/purplejester/showing-samples-with-segmentation-mask-overlay

    Params:
        image: Training image. np.ndarray,
        mask: Segmentation mask. np.ndarray,
        color: Color for segmentation mask rendering.  tuple[int, int, int] = (255, 0, 0)
        alpha: Segmentation mask's transparency. float = 0.5,
        resize: If provided, both image and its mask are resized before blending them together.
        tuple[int, int] = (1024, 1024))

    Returns:
        image_combined: The combined image. np.ndarray

    """
    color = color[::-1]
    colored_mask = np.expand_dims(mask, 0).repeat(3, axis=0)
    colored_mask = np.moveaxis(colored_mask, 0, -1)
    masked = np.ma.MaskedArray(image, mask=colored_mask, fill_value=color)
    image_overlay = masked.filled()

    if resize is not None:
        image = cv2.resize(image.transpose(1, 2, 0), resize)
        image_overlay = cv2.resize(image_overlay.transpose(1, 2, 0), resize)

    image_combined = cv2.addWeighted(image, 1 - alpha, image_overlay, alpha, 0)

    return image_combined


if __name__ == "__main__":

    # Let user
    parser = argparse.ArgumentParser()
    # parser.add_argument("target_img_dir" , type=pathlib.Path ,help="path to Yolo yaml file")
    parser.add_argument("model", type=str ,help="bbox-model-file")

    args = parser.parse_args()
    # target_image_dir : pathlib.Path = args.target_img_dir
    # print(f"reading data file at {target_image_dir}")

    # if not target_image_dir.exists():
    #     print(f"Target directory doesn't exists")
    #     exit(1)

    # Load the pretrained model

    model = YOLO(args.model)

    cap = cv2.VideoCapture(0)
    cv2.namedWindow("Input" , cv2.WINDOW_NORMAL)
    cv2.namedWindow("region" , cv2.WINDOW_NORMAL)

    if not cap.isOpened():
        raise IOError("Cannot open webcam")
    while True:
        ret, frame = cap.read()

        results = model.predict(frame , stream=True , classes=[58])
        for r in results:
            # boxes = r.boxes

            if r.boxes is not None:
                for box in r.boxes:
                    print(f"box_xyxy {box.xyxy}")
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values


                    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 3)
                #     confidence = math.ceil((box.conf[0]*100))/100

                #     text_origion = [x1, y1]
                #     cv2.putText(frame, f"{confidence}", text_origion , cv2.FONT_HERSHEY_PLAIN , 2,(255, 0, 255) )

                    box_region =     region = frame[y1:y2 , x1:x2 , :]
                    cv2.imshow("region" , box_region)
            if r.masks is not None:
                for mask in r.masks:
                    masked_frame = overlay(frame , mask, color=(0,0,255) , alpha=0.2)
        cv2.imshow('Input', frame)


        c = cv2.waitKey(50)
        if c == 27:
            break


    # 58: 'potted plant' this is the existing class.
    # https://docs.ultralytics.com/reference/engine/model/#ultralytics.engine.model.Model.predict
    # results: list[ultralytics.engine.results.Results] = model.predict(target_image_dir,save=True)
