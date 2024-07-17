## Over all system flow chart 

The system will have two stages. One is mapping, the other is watering.

In the mapping/explore stage, robot drive around the room just to build a 3d map with plant and water fountain's location labeled. 

In the watering stage, robot navigate to water fountain and plant to perform watering task.

**Mapping/Explore Stage**

```mermaid

flowchart TD
    start_node(["Exploring Start"])

    return_check_node(["Double Check Recorded Location"])

    station_scan["Scan around with Head"]
    nav2_driving["Start Move To Frontier"]
    KeepMappingCheck{"Any\n UnKnown\n Area?"}
    record_plant["Record plant and water \nfountain location \n while mapping"]


start_node --> station_scan
start_node --> record_plant
subgraph Driving["Mapping Unknown Area"]
    station_scan --> KeepMappingCheck

    KeepMappingCheck --"Exists"--> PlaneMove["Plan Move to \nUnknown Frontier"]
    PlaneMove --> nav2_driving
    nav2_driving --"Reached Frontier"--> station_scan


end
    KeepMappingCheck --"None"--> return_check_node
    record_plant --> return_check_node

DriveToPlant["Drive to plant location"]
FindPlantCluster["Make Clusters of Locations"]

subgraph Confirm["Confirm Plant and Water Fountain Location"]
    return_check_node --> FindPlantCluster --> for_each 
    for_each["For Each Cluster"] --> DriveToPlant --> NarrowDown["Narrow Down Individual object's Location"] --> for_each
end

for_each --"No more inaccurate clusters"--> save["Save Maps and \n Location to Disk"]

save --> Done(["Done"])

%% nav2_driving --> record_plant --> dymmy_stop_return

```

**Watering Stages**

```mermaid

flowchart TB

loop1[Foreach Pot]

LoadMap[Load Map and Locations] --> loop1

loop1 --"More to Water"--> DriveWater[Drive to \n Water Fountain] --> fill[Fill water Jar] --> DriveFlower[Drive to Flowerpot] --> PourWater[Pour Water] --> loop1

loop1 --"All Watered"--> Park["Pack Back at Base"]

```