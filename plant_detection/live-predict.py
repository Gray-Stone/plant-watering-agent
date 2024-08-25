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


@dataclasses.dataclass
class ObjectInfo():
    pix_xyxy : tuple[int]
    cls : int
    confidence : float
    mask : np.ndarray
    outline_xy: np.ndarray



class Detector():
    def __init__(self , model) -> None:
        self.model = model
        self.color_list :list[tuple] = [(30,255,30),(255,30,30),(30,30,255)]
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

        print(f"mask xy type {type(mask.xy)}")
        print(f"mask_len {len(mask.xy[0])}")

        # out_frame = np.ndarray(frame_size,dtype=np.uint8)
        # for xy in mask.xy[0]:
        #     x,y = xy
        #     # out_frame = cv2.circle(out_frame , ( int(x),int(y) ),1,(255),thickness=-1)
        #     outline_xy.append( ( int(x),int(y) ) )
        # cv2.imshow("debug" , out_frame)

        outline_xy = mask.xy[0].astype(np.uint8)

        return mask_scaled, outline_xy

    def DetectOnce(self,frame)->dict[list[ObjectInfo]]:
        frame_size_y , frame_size_x , _ = frame.shape
        results = list(self.model.predict(frame , stream=True))
        self._last_frame_size = frame.shape
        self._blank_color_frames =[]
        for color in self.color_list:
            color_blank = np.zeros(frame.shape , dtype=frame.dtype)
            color_blank[:] = color
            self._blank_color_frames.append(color_blank)

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
            segment_mask , outline_xy = self.DecodeMask(mask , (frame_size_y , frame_size_x))
            obj_info = ObjectInfo(xyxy_pix , obj_class , confidence , segment_mask , outline_xy)
            if obj_class in obj_info_map:
                obj_info_map[obj_class].append(obj_info)
            else:
                obj_info_map[obj_class] = [obj_info]
            obj_info_list.append(obj_info)

        return obj_info_map , obj_info_list

    def PlotObjects(self,frame , object_infos:list[ObjectInfo] , domask = True):

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
            if domask:
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
    parser.add_argument("--m2", type=str ,help="model-file")
    parser.add_argument("--m3", type=str ,help="model-file")

    parser.add_argument("--window-name", type=str ,help="window_name")
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
    window_name = args.window_name 
    if window_name is None:
        window_name = pathlib.Path(args.model).stem

    # cv2.namedWindow("input" , cv2.WINDOW_NORMAL)
    cv2.namedWindow(window_name , cv2.WINDOW_NORMAL)
    # cv2.namedWindow("debug" , cv2.WINDOW_NORMAL)
    # cv2.namedWindow("mask" , cv2.WINDOW_NORMAL)

    model = YOLO(args.model)
    detector = Detector(model)

    while True:

        if use_image:
            frame = cv2.imread(args.image)
        else:
            ret, frame = cap.read()

        frame = cv2.rotate(frame,cv2.ROTATE_90_CLOCKWISE)

        obj_info_map , obj_info_list = detector.DetectOnce(frame)

        for key,val in obj_info_map.items():
            print(f" Cls: {key} have {len(val)} number of items ")

        labeled_frame =detector.PlotObjects(frame.copy() , obj_info_list)

        for obj_info in obj_info_list:
            # Only extreme outer contours
            # cv.CHAIN_APPROX_SIMPLE  cv.CHAIN_APPROX_TC89_L1  cv.CHAIN_APPROX_TC89_KCOS
            contours, hierarchy = cv2.findContours(
                obj_info.mask,
                mode=cv2.RETR_EXTERNAL,
                method=cv2.CHAIN_APPROX_TC89_L1,
            )
            print(f"countours {contours}")
            cv2.drawContours(frame,contours , 0,color=detector.color_list[obj_info.cls])


        # cv2.imshow("input" , frame)
        cv2.imshow(window_name, labeled_frame)

        if args.image is None:
            c = cv2.waitKey(50)
        else :
            c = cv2.waitKey()
            exit(0)
        if c == 27:
            break

    # https://docs.ultralytics.com/reference/engine/model/#ultralytics.engine.model.Model.predict
    # results: list[ultralytics.engine.results.Results] = model.predict(target_image_dir,save=True)
