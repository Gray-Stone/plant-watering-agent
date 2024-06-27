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


def overlay_masks(output_frame: np.ndarray, mask_list :list[any] , color =(30,30,255) , alpha = 0.3):
    """Takes an output frame, list of masks. Overlay all masks onto output


    Args:
        output_frame (_type_): output image
        mask_list (): list of masks.
        color (tuple, optional): color of masked area. Defaults to (30,30,255).
        alpha float: alpha (transparent ness) for each mask, default to 0.3
    """

    color_slice = np.zeros(output_frame.shape , dtype=output_frame.dtype)
    color_slice[:] =color
    for mask in mask_list:
        color_mask = cv2.bitwise_and(color_slice,color_slice , mask=mask)
        output_frame = cv2.addWeighted(output_frame, 1, color_mask , alpha , 0)

    return output_frame



if __name__ == "__main__":

    # Let user
    parser = argparse.ArgumentParser()
    # parser.add_argument("target_img_dir" , type=pathlib.Path ,help="path to Yolo yaml file")
    parser.add_argument("model", type=str ,help="model-file")
    parser.add_argument("--image" , help="Use image instead of cv" , default=None)
    args = parser.parse_args()

    use_image = False
    if args.image:
        use_image = True

    model = YOLO(args.model)
    colors = [(30,30,255),(30,255,30),(255,30,30)]

    cap = cv2.VideoCapture(6)
    cv2.namedWindow("Input" , cv2.WINDOW_NORMAL)
    cv2.namedWindow("bbox" , cv2.WINDOW_NORMAL)
    cv2.namedWindow("mask" , cv2.WINDOW_NORMAL)

    if not cap.isOpened():
        raise IOError("Cannot open webcam")
    while True:

        if use_image:
            frame = cv2.imread(args.image)
        else:
            ret, frame = cap.read()

        frame_size_y , frame_size_x , _ = frame.shape
        results = model.predict(frame , stream=True)
        i = 0
        masked_image = frame.copy()
        bbox_image = frame.copy()

        for r in results:
            # boxes = r.boxes
            bbox_list = []
            mask_list = []
            print(f"Class Name {r.names}")
            color = colors[i]
            i = i+1
            if i == len(colors):
                i=0
            # print(f"bboxs \n {r.boxes}")
            if r.boxes is not None:
                for box in r.boxes:
                    print(f"box_xyxy {box.xyxy}")
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values

                    bbox_list.append((x1, y1, x2, y2))


                    # confidence = math.ceil((box.conf[0]*100))/100
                    # text_origion = [x1, y1]
                    # cv2.putText(frame, f"{confidence}", text_origion , cv2.FONT_HERSHEY_PLAIN , 2,(255, 0, 255) )
            # print(f"masks: \n {r.masks}")
            if r.masks is not None:
                for mask in r.masks:
                    mask_raw = mask.cpu().data.numpy().transpose(1, 2, 0)
                    mask_raw = (mask_raw*255).astype(np.uint8)
                    # print(f" mask raw shape {mask_raw.shape}")
                    mask_scaled = cv2.resize(mask_raw, (frame_size_x, frame_size_y))
                    # print(f" mask scaled shape {mask_scaled.shape}")
                    mask_list.append(mask_scaled)

            
            for b in bbox_list:
                x1, y1, x2, y2 = b
                cv2.rectangle(bbox_image, (x1, y1), (x2, y2), color, 3)


            masked_image = overlay_masks(masked_image ,mask_list ,color)

        cv2.imshow('bbox', bbox_image)
        cv2.imshow('mask', masked_image)

        c = cv2.waitKey(50)
        if c == 27:
            break


    # 58: 'potted plant' this is the existing class.
    # https://docs.ultralytics.com/reference/engine/model/#ultralytics.engine.model.Model.predict
    # results: list[ultralytics.engine.results.Results] = model.predict(target_image_dir,save=True)
