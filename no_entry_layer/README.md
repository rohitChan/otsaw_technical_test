# no_entry_layer

A ros node that takes in maps from /map topic and publishes another map with inputs from user from rqt_reconfigure.

## Installation

Just add to your workspace and do catkin_make. Dependencies are dynamic_reconfigure and nav_msgs.
## Usage

```In terminal
roslaunch no_entry_layer map_server.launch
```
Modify the cost value and masking region from rqt_reconfigure window.
## License
[BSD](http://www.linfo.org/bsdlicense.html)
