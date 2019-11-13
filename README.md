# no_entry_layer

A ros package that has two nodes: 
- one for modifying a given map with gui parameters and publishing the modified map under /map topic; 
- another node that subscribes to /map topic and maintains a server for /static_map service required for move_base.

## Demo
/no_entry_layer/demo/no_entry_layer2-2019-11-13_12.10.01.mp4 
- Demo video showing the global_costmap in RViz with a static map, along with its real-time modification using user input of cost and modifying region parameters from gui.

## Nodes

### /no_entry_layer_node
1. Subscribes map as nav_msgs/OccupancyGrid from "/originalMap/map". The map_server in the carto_move_base.launch file is launched under namespace "/originalMap".
2. Gets input from dynamic reconfigure to process the map. If mask_on not selected, publishes original map as nav_msgs/OccupancyGrid at "/map" topic.
3. If mask on selected, change the value of the elements inside the rectangle defined by the user. The first two parameters (mask_lb_x, mask_lb_y) are for left bottom vertex of the rectangle and the last two defines the top right vertex of the rectangle considering the map data as an image with same dimension as the map. The algorithm assumes that left bottom vertex x and y coordinates is always less than top right x and y coordinates, respectively. 
4. The node publishes the processed map as nav_msgs/OccupancyGrid at "/map" topic. 

### /no_entry_service
1. Subribes to /map topic
2. Send the /map message as a response for "/static_map" service request


## Installation

Just add to your workspace and do catkin_make. Dependencies are dynamic_reconfigure, move_base and nav_msgs. OpenCV2 is required for another node in progress. Please change the PATHS param for find_package(OpenCV 2 REQUIRED)  to your local path.
## Usage

```In terminal
roslaunch no_entry_layer carto_move_base.launch 

Lauches following nodes:
/move_base
/no_entry_layer_node
/no_entry_service
/originalMap/map_server
/rqt_gui
/rviz
```

Modify the costmap of a desired region with a desired costmap value using rqt_gui window after checking mask_on option.

Modify the cost value and masking region from rqt_reconfigure window.

If the global_cost map does not change, try unselecting and selecting /global_costmap topic from move_base in rViz.

## License
[BSD](http://www.linfo.org/bsdlicense.html)
