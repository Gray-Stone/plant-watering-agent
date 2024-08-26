

## Uncommon Design:

Due to network latency, it is usually not possible to get raw image directly through wifi without seriously slowing down all the nodes. Same goes for point clouds.

Thus, a decision is made to run all nodes on the stretch3 as it have a decent good computer.

To get smooth visualizations of camera output and point clouds, rviz is also run on the stretch3. This require the use of moonlight to get a remote desktop connection to the robot. Stretch have [detail instruction](https://docs.hello-robot.com/0.3/getting_started/connecting_to_stretch/) on this. 

**However, remember to run `export DISPLAY=:0` on a ssh session before launch any GUI application (like RVIZ)! Or it will try to launch via ssh X11 which is very slow**


## 

After sourcing the workspace, run
```
ros2 launch stretch_mover full_auto.launch.py
``` 
This launch file will bring up `stretch_driver`, `realsense_driver`, `yolo`, `simple_yolo_depth`, `rtabmap`, `nav2` , `rviz`.

The only other node missing is the state controller. I intentionally leave it out to run on separate terminal is to better see the current state change.

After running 

```
ros2 run stretch_mover StatefulController.py
``` 

The robot should start moving.

**It is by design robot directly start exploring after running State Controller, and robot will start turing to scan environment! Make sure robot start up in a collision free position!**

The state controller have two main stages, exploration and manipulation.

### Exploration

On start up, the state controller automatically run into this stage and will try to explore all possible regions on the map (within robot's C space) using NAV2 only (no arm movement). After its finished, state controller will enter IDLE state, and have no way of going back to exploration state.

The exploration is a loop of 
* Pan camera head to scan
* Frontier explore the global map for any unknown area
* Nav towards it.

If there are no more unknown area, controller will exit the exploration stage.

### Manipulation

While in IDLE state, if user sent a trigger to `/watering_trigger` service `ros2 service call /watering_trigger std_srvs/srv/Trigger {}`. Robot will start going over every plant it has previously seen before and water them.

The general flow as
* For each plant
    * Nav to a free space in front of plant
    * Look at plant until found a valid watering are (z above plant, within cretin xy distance)
    * Lift arm for clearance
    * Turn base to point arm into plant
    * Move camera head to double check watering area
    * Extend the arm out and pour water.
    * Move on to next plant

After all plants are watered, controller will get back to IDLE state. Until user sent another `watering_trigger`, controller will repeat the watering sequence again. 

