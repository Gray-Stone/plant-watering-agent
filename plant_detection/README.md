

## YOLO model training.

Scripts are made to help with creating the yolo model used for detecting flowerpot and watering area.

I recommend running the script at the directory from outside the repo as it generates lots of data.

Scripts are wrote to keep track of the process, they could be found under 

- Source_data_process : Scripts that download data, export label, merge labels, etc
- plant_detection : Scripts that train model, load data and run detection, etc.

The following are a walk through of the overall process. Assume all commands are run from the parent folder of this repo (so resulting output data are not dumped into the repo)

### Image source

Open Images v7 are used as image source. They have massive amount of data with flowerpot as one of the pre-labeled items (and segmented as well).

The data are accessed via tool: `fiftyone`. Example usage are: 

```
./plant-watering-agent/Source_data_process/fiftyone-export.py OI7-Datas/large_batch_export -s 10000
```

Which will download and export 10000 images to `OI7-Datas/large_batch_export`

Since `fiftyone` doesn't provide a yolo compatible export format for segmentation, the script will export everything in coco format. 

#### COCO to YOLO

Both `fiftyone` and `CVAT` doesn't have good tools for exporting yolo images, thus a third party tool is needed.

I have fork the ![coco2yolo-seg](https://github.com/Gray-Stone/coco2yolo-seg-mod.git) project with some added features.

Here is an example of running this conversion script on the images exported by the command above:
```
coco2yolo-seg-mod/COCO2YOLO-seg.py OI7-Datas/large_batch_export/labels.json OI7-Datas/large_batch_export/data/ OI7-Datas/large_batch_export/yolo-format
```

The `labels.json` is the coco label file that contains all label. The script also need the image dir, and it does a val/train split for you. The images are symlinked so no extra copies are generated. The output folder will have everything needed for training a yolo model, the most important `dataset.yaml` is the one need to be provided to yolo.

### Training

```
plant-watering-agent/plant_detection/yolo_train_new.py OI7-Datas/large_batch_export/yolo-format/dataset.yaml --proj new-trains --seg -e 1000 --device 1 
```

The above command will launch the training of a new yolo model on `OI7-Datas/large_batch_export/yolo-format/dataset.yaml` with segmentation, 1000 epoc and on second GPU. It will output result to `new-trains` 

### Merging extra labels

The Open Images 7 dataset only have flower pots labeled. For detecting watering area, we need to label them ourselves. See journal post #3 and #3.5 for why use CVAT.

Note: Only a subset of images from Open Images 7 are used. If any flowerpot image contain watering area but is not labeled, it model will not be able to learn things properly. The training setup here is very sensitive to false label and missing labels.

The self labeled data exported from CVAT are also COCO format. After converting it to yolo, there needs to be a step for merging it with existing data before training.

The script is actually written to be used in reverse, merging old superset's label (from coco.json) onto new subset's yolo dataset. This prevented accidentally adding images that have watering area, but not labeled in CVAT (There are planty of images with only label for flowerpot but no watering area, as they actually don't include watering area)

Here is an example. Output dataset from CVAT for watering area is already converted to YOLO before hand.
```
coco2yolo-seg/COCO2YOLO-merge.py Datas/open-images_7/Open_Images_7/labels.json Datas/open-images_7/cvat-Watering-area/YOLO/dataset.yaml
```