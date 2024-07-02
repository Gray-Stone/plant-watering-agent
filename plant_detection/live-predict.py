#! /usr/bin/python3

import argparse
import pathlib

import dataclasses
from ultralytics import YOLO
import ultralytics.engine
import ultralytics.engine.results

import pathlib

import cv2
import math
import numpy as np

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

@dataclasses.dataclass
class ObjectInfo():
    pix_xyxy : tuple[int]
    cls : int
    confidence : float
    mask : np.ndarray

class Detector():
    def __init__(self , model) -> None:
        self.model = model
        self.color_list :list[tuple] = [(30,30,255),(30,255,30),(255,30,30)]
        # Helper used for plotting.
        self._last_frame_size = None
        self._blank_color_frames = []

    def DecodeBbox(self,box:ultralytics.engine.results.Boxes):
        # Because everything in box is
        # tensor([[446.7046, 365.7438, 651.1225, 635.0509]], device='cuda:0')
        # Thus need this indexing
        x1, y1, x2, y2 = box.xyxy[0]
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values
        obj_class = int(box.cls)
        xyxy_pix = (x1, y1, x2, y2)
        confidence = float(box.conf)
        print(f"box info {xyxy_pix} , cls {obj_class} conf { confidence}")

        return xyxy_pix , obj_class , confidence

    def DecodeMask(self,mask: ultralytics.engine.results.Masks, frame_size:tuple[int,int]):
        frame_size_y , frame_size_x = frame_size
        mask_raw = mask.cpu().data.numpy().transpose(1, 2, 0)
        mask_raw = (mask_raw*255).astype(np.uint8)
        # print(f" mask raw shape {mask_raw.shape}")
        mask_scaled = cv2.resize(mask_raw, (frame_size_x, frame_size_y))
        return mask_scaled

    def DetectOnce(self,frame)->dict[list[ObjectInfo]]:
        frame_size_y , frame_size_x , _ = frame.shape
        results = list(self.model.predict(frame , stream=True))

        if len(results) > 1:
            # I guess the list of result is for handling multiple images.
            print(f"\n============\nWARING!\n ")
            print(f"len of result {len(results)}")

        result = results[0]
        
        if not result.boxes:
            return {} , []
        print(f"len of boxes {len(result.boxes)}")
        print(f"len of mask {len(result.masks)}")

        obj_info_map = {}
        obj_info_list = []
        for box, mask in zip(result.boxes , result.masks):
            xyxy_pix , obj_class , confidence = self.DecodeBbox(box)
            segment_mask = self.DecodeMask(mask , (frame_size_y , frame_size_x))
            obj_info = ObjectInfo(xyxy_pix , obj_class , confidence , segment_mask)
            if obj_class in obj_info_map:
                obj_info_map[obj_class].append(obj_info)
            else:
                obj_info_map[obj_class] = [obj_info]
            obj_info_list.append(obj_info)

        return obj_info_map , obj_info_list

    def PlotObjects(self,frame , object_infos:list[ObjectInfo] ):

        BOX_BORDER_THICKNESS = 3
        MASK_ALPHA = 0.3
        FONT_SCALE = 0.5
        FONT_THICKNESS = 2
        # If size didn't change, then color frame won't change.
        if self._last_frame_size != frame.shape:
            self._last_frame_size = frame.shape
            self._blank_color_frames =[]
            for color in self.color_list:
                color_blank = np.zeros(frame.shape , dtype=frame.dtype)
                color_blank[:] = color
                self._blank_color_frames.append(color_blank)

        for info in object_infos:
            # cv2.imshow("debug" , info.mask)
            x1, y1, x2, y2 = info.pix_xyxy
            cv2.rectangle(frame , (x1, y1), (x2, y2), self.color_list[info.cls],
                          BOX_BORDER_THICKNESS)
            cv2.putText(
                frame,
                f"{info.cls}:_{info.confidence:.2f}",
                (x1, y2),
                cv2.FONT_HERSHEY_SIMPLEX,
                FONT_SCALE,
                self.color_list[info.cls],
                thickness=FONT_THICKNESS,
            )
            color_mask = cv2.bitwise_and(
                self._blank_color_frames[info.cls],
                self._blank_color_frames[info.cls],
                mask=info.mask,
            )
            frame = cv2.addWeighted(frame, 1, color_mask, MASK_ALPHA, 0)
        return frame



if __name__ == "__main__":

    # Let user
    parser = argparse.ArgumentParser()
    # parser.add_argument("target_img_dir" , type=pathlib.Path ,help="path to Yolo yaml file")
    parser.add_argument("model", type=str ,help="model-file")
    parser.add_argument("--image" , help="Use image instead of cv" , default=None)
    args = parser.parse_args()

    use_image = False
    cap = None
    if args.image:
        use_image = True
    else:
        cap = cv2.VideoCapture(6)
        if not cap.isOpened():
            raise IOError("Cannot open webcam")

    cv2.namedWindow("input" , cv2.WINDOW_NORMAL)
    cv2.namedWindow("labeled_image" , cv2.WINDOW_NORMAL)
    # cv2.namedWindow("debug" , cv2.WINDOW_NORMAL)
    # cv2.namedWindow("mask" , cv2.WINDOW_NORMAL)

    model = YOLO(args.model)
    detector = Detector(model)

    while True:

        if use_image:
            frame = cv2.imread(args.image)
        else:
            ret, frame = cap.read()
        
        obj_info_map , obj_info_list = detector.DetectOnce(frame)

        for key,val in obj_info_map.items():
            print(f" Cls: {key} have {len(val)} number of items ")
        
        labeled_frame =detector.PlotObjects(frame.copy() , obj_info_list)

        cv2.imshow("input" , frame)
        cv2.imshow('labeled_image', labeled_frame)

        if args.image is None:
            c = cv2.waitKey(50)
        else :
            c = cv2.waitKey()
            exit(0)
        if c == 27:
            break

    # https://docs.ultralytics.com/reference/engine/model/#ultralytics.engine.model.Model.predict
    # results: list[ultralytics.engine.results.Results] = model.predict(target_image_dir,save=True)
