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

def resize_with_pad(image: np.array, 
                    new_shape: tuple[int, int], 
                    padding_color: tuple[int] = (255, 255, 255)) -> np.array:
    """Maintains aspect ratio and resizes with padding.
    Params:
        image: Image to be resized.
        new_shape: Expected (width, height) of new image.
        padding_color: Tuple in BGR of padding color
    Returns:
        image: Resized image with padding
    """
    original_shape = (image.shape[1], image.shape[0])
    ratio = float(max(new_shape))/max(original_shape)
    new_size = tuple([int(x*ratio) for x in original_shape])
    image = cv2.resize(image, new_size)
    delta_w = new_shape[0] - new_size[0]
    delta_h = new_shape[1] - new_size[1]
    top, bottom = delta_h//2, delta_h-(delta_h//2)
    left, right = delta_w//2, delta_w-(delta_w//2)
    image = cv2.copyMakeBorder(image, top, bottom, left, right, cv2.BORDER_CONSTANT, value=padding_color)
    return image

def get_one_image(img_list):
    max_width = 0
    total_height = 200  # padding
    for img in img_list:
        if img.shape[1] > max_width:
            max_width = img.shape[1]
        total_height += img.shape[0]

    # create a new array with a size large enough to contain all the images
    final_image = np.zeros((total_height, max_width, 3), dtype=np.uint8)

    current_y = 0  # keep track of where your current image was last placed in the y coordinate
    for image in img_list:
        # add an image to the final array and increment the y coordinate
        image = np.hstack((image, np.zeros((image.shape[0], max_width - image.shape[1], 3))))
        final_image[current_y:current_y + image.shape[0], :, :] = image
        current_y += image.shape[0]
    return final_image

if __name__ == "__main__":

    # Let user
    parser = argparse.ArgumentParser()
    # parser.add_argument("target_img_dir" , type=pathlib.Path ,help="path to Yolo yaml file")
    parser.add_argument("model", type=str ,help="bbox-model-file")
    parser.add_argument("seg_model", type=str ,help="seg-model-file")

    args = parser.parse_args()
    # target_image_dir : pathlib.Path = args.target_img_dir
    # print(f"reading data file at {target_image_dir}")

    # if not target_image_dir.exists():
    #     print(f"Target directory doesn't exists")
    #     exit(1)

    # Load the pretrained model

    print(f"using bbox model: {args.model} ")
    print(f"using seg model: {args.seg_model}")
    bbox_model = YOLO(args.model)
    seg_model = YOLO(args.seg_model)

    cap = cv2.VideoCapture(6)

    # cap = cv2.VideoCapture("/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_435i_Intel_R__RealSense_TM__Depth_Camera_435i_035323050551-video-index0")
    cv2.namedWindow("Input" , cv2.WINDOW_NORMAL)
    cv2.namedWindow("region" , cv2.WINDOW_NORMAL)
    cv2.namedWindow("cropped_yolo_seg" , cv2.WINDOW_NORMAL)
    cv2.namedWindow("masked_region" , cv2.WINDOW_NORMAL)
    cv2.namedWindow("direct_seg" , cv2.WINDOW_NORMAL)

    if not cap.isOpened():
        raise IOError("Cannot open webcam")
    while True:
        ret, frame = cap.read()
        frame_y, frame_x , _ = frame.shape
        results = bbox_model.predict(frame)
        for r in results:
            # boxes = r.boxes
            overall_masks = []
            box_mask_list = []
            box_regions = []
            if r.boxes is not None:
                for box in r.boxes:
                    print(f"box_xyxy {box.xyxy}")
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values
                    print(f"box xyxy {x1}, {y1}, {x2}, {y2}")


                    region_x1 = max(x1-10,0)
                    region_y1 = max(y1-10,0)
                    region_x2 = min(x2+10 , frame_x)
                    region_y2 = min(y2+10 , frame_y)

                    box_region =frame[region_y1:region_y2 , region_x1:region_x2 , :].copy()
                    box_region_size_y , box_region_size_x , _ = box_region.shape


                    cv2.rectangle(frame, (x1-1, y1-1), (x2+1, y2+1), (255, 0, 255), 2)
                    confidence = math.ceil((box.conf[0]*100))/100

                    text_origion = [x1, y1]
                    cv2.putText(frame, f"{confidence}", text_origion , cv2.FONT_HERSHEY_PLAIN , 2,(255, 0, 255) )


                    seg_results = seg_model.predict(box_region)

                    box_region_output = box_region

                    mask_cout = 0
                    for seg_r in seg_results:

                    # Other people's note on making the mask
                    # https://github.com/ultralytics/ultralytics/issues/1407
                        # print(seg_r.boxes)
                        if seg_r.masks is not None:
                            
                            for mask in seg_r.masks:
                                mask_cout+=1
                                mask_raw = mask.cpu().data.numpy().transpose(1, 2, 0)
                                mask_raw = (mask_raw*255).astype(np.uint8)
                                # scaled up 
                                mask_scaled = cv2.resize(mask_raw, (box_region_size_x , box_region_size_y))
                                # M = np.float32([[1, 0, x1], [0, 1, y1]])
                                # shifted = cv2.warpAffine(img, M, size)
                                left_pad = region_x1
                                right_pad = frame_x - region_x2
                                up_pad = region_y1
                                down_pad = frame_y - region_y2
                                padded_mask = cv2.copyMakeBorder(mask_scaled , up_pad , down_pad,left_pad,right_pad,cv2.BORDER_CONSTANT,0 )
                                overall_masks.append(padded_mask)
                                # print(f"frome shape {mask_scaled.shape} , padding:{left_pad} , {right_pad} , {up_pad} , {down_pad} , size {padded_mask.shape}")

                                # contours, hierarchy = cv2.findContours(mask_scaled, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                        # cv2.imshow("cropped_yolo_seg" , seg_r.plot())
                        box_region_output = seg_r.plot()


                    box_mask_list.append(mask_cout)
                    box_regions.append(box_region_output)
                    # cv2.imshow("region" , box_region)

                combined_mask = np.full((frame_y, frame_x) , 0 , dtype=np.uint8)
                red_img = np.zeros(frame.shape , dtype=frame.dtype)
                red_img[:] =(0,0,255)
                print(f"Total of {len(overall_masks)}")
                frame_masked = frame.copy()
                for mask in overall_masks:
                    # print(f"mask_ size {mask.dtype} , combined _amsk {combined_mask.dtype}")
                    # combined_mask= cv2.bitwise_or(combined_mask , mask)
                    red_mask = cv2.bitwise_and(red_img ,red_img, mask=mask)
                    # cv2.imshow("masked_region" , red_mask)
                    
                    frame_masked = cv2.addWeighted(frame_masked,1,red_mask,0.2,0.0)
                

                
                cv2.imshow("cropped_yolo_seg", get_one_image(box_regions))

                #         # masked_frame = overlay(box_region , mask, color=(0,0,255) , alpha=0.2)

                cv2.imshow("masked_region" , cv2.resize(frame_masked,None,fx=0.3,fy=0.3))

        print(f"box_mask_count = {box_mask_list}")

        direct_seg_results = seg_model.predict(frame)
        cv2.imshow('direct_seg', cv2.resize(direct_seg_results[0].plot() , None,fx=0.3,fy=0.3))


        c = cv2.waitKey(150)
        if c == 27:
            break


    # 58: 'potted plant' this is the existing class.
    # https://docs.ultralytics.com/reference/engine/model/#ultralytics.engine.model.Model.predict
    # results: list[ultralytics.engine.results.Results] = model.predict(target_image_dir,save=True)
