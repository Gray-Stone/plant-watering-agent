
# Auto Plant Watering with Mobile Manipulation

This repo holds the packages for top level robot control and scripts for YOLO model training and runtime processing.

Folder structure 
```
./
├── Journal: detail development journals documented design decisions and pitfalls. 
├── plant_detection: YOLO related scripts for training and validating models
├── ros-ws: 
│   └── stretch_mover: ros package hosting top level state machines that controls robot, and script to apply yolo with depth camera.
├── Source_data_process: Helper scripts for data source gathering and processing
└── Trained_Models: Various YOLO training output.
```


