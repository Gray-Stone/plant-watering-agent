#! /usr/bin/python3

import fiftyone as fo
import fiftyone.zoo as foz
import fiftyone.utils.openimages as foo
# https://docs.voxel51.com/integrations/open_images.html#open-images


# https://docs.voxel51.com/user_guide/dataset_zoo/index.html

# Options are actually mostly in here:
# https://docs.voxel51.com/tutorials/open_images.html


# print(foo.get_classes())

# exit(1)
dataset: fiftyone.core.dataset.Dataset = foz.load_zoo_dataset(
    "open-images-v7",
    split="train",
    max_samples=100,
    classes=["Flowerpot"] , # vase does have some good image, but might not be best
    label_types=["detections","segmentations"]
    # shuffle=True,
    ,
    
)
session = fo.launch_app(dataset)

while(1):
    pass