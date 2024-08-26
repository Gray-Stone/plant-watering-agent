#! /usr/bin/python3

import argparse
import pathlib

import dataclasses
from ultralytics import YOLO
import ultralytics.engine.results

import pathlib

import cv2
import math
import numpy as np

import textwrap

# https://stackoverflow.com/questions/27647424/opencv-puttext-new-line-character
# https://gist.github.com/EricCousineau-TRI/596f04c83da9b82d0389d3ea1d782592
def draw_text(
    img,
    *,
    text,
    uv_top_left,
    color=(255, 255, 255),
    fontScale=0.5,
    thickness=1,
    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
    outline_color=None,
    line_spacing=1.5,
):
    """
    Draws multiline with an outline.
    """
    assert isinstance(text, str)

    uv_top_left = np.array(uv_top_left, dtype=float)
    assert uv_top_left.shape == (2,)

    for line in text.splitlines():
        (w, h), _ = cv2.getTextSize(
            text=line,
            fontFace=fontFace,
            fontScale=fontScale,
            thickness=thickness,
        )
        uv_bottom_left_i = uv_top_left + [0, h]
        org = tuple(uv_bottom_left_i.astype(int))

        if outline_color is not None:
            cv2.putText(
                img,
                text=line,
                org=org,
                fontFace=fontFace,
                fontScale=fontScale,
                color=outline_color,
                thickness=thickness * 3,
                lineType=cv2.LINE_AA,
            )
        cv2.putText(
            img,
            text=line,
            org=org,
            fontFace=fontFace,
            fontScale=fontScale,
            color=color,
            thickness=thickness,
            lineType=cv2.LINE_AA,
        )

        uv_top_left += [0, h * line_spacing]


if __name__ == "__main__":

    
    # Let user
    parser = argparse.ArgumentParser()
    # parser.add_argument("target_img_dir" , type=pathlib.Path ,help="path to Yolo yaml file")
    parser.add_argument("--video" , help="input video" ,type=pathlib.Path, default=None , required=True)
    parser.add_argument("--models",nargs='+', type=pathlib.Path,help="folder of models.",default=None , required=True)

    parser.add_argument("--window-name", type=str ,help="window_name")

    args = parser.parse_args()

    in_vid :pathlib.Path = args.video
    print(f"Input video {in_vid}")

    cap = cv2.VideoCapture( str(in_vid))
    if not cap.isOpened():
        raise IOError("Cannot open video")
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))


    window_name = args.window_name 
    if window_name is None:

        window_name = in_vid.stem

    cv2.namedWindow(window_name , cv2.WINDOW_NORMAL)

    models = []
    titles = []

    for f in args.models:
        f:pathlib.Path
        print(f"Opening model {f} ")
        model = YOLO(f)
        models.append(model)
        warpped_title = textwrap.fill(f.stem,35)
        titles.append(warpped_title)

    pad_size = 90

    
    out = cv2.VideoWriter('output_video.mp4', cv2.VideoWriter.fourcc(*'mp4v'), fps, (width * len(models), height + pad_size))

    while(cap.isOpened()):
    # Capture frame-by-frame
        ret, frame = cap.read()
        if ret == True:
            labeled_frames = []
            for m , title in zip(models , titles):
                results = m.predict(frame)
                if results is not None:
                    labeled_frame = results[0].plot()
                else:
                    labeled_frame = frame.copy()

                padded = cv2.copyMakeBorder( labeled_frame , pad_size,0,0,0, cv2.BORDER_CONSTANT , value=(255,255,255))
                draw_text(padded , text= title , uv_top_left=(2,2) , color=(0,0,0) , fontScale= 1.2 , thickness=2)
                
                labeled_frames.append(padded)
            stack = np.concatenate(labeled_frames, axis=1)


            out.write(stack)
            # cv2.imshow("input" , frame)
            cv2.imshow(window_name, stack)

            c = cv2.waitKey(1)
            if c == 27:
                break
        else:
            break 
    cap.release()
    out.release()
    
    # Closes all the frames
    cv2.destroyAllWindows()
    # https://docs.ultralytics.com/reference/engine/model/#ultralytics.engine.model.Model.predict
    # results: list[ultralytics.engine.results.Results] = model.predict(target_image_dir,save=True)
