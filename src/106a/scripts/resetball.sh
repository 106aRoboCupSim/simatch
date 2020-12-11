#!/bin/bash

rosservice call /gazebo/set_model_state "model_state: 
model_name: 'football'
pose:
    position:
    x: 0.0
    y: 0.0
    z: 0.12
"

rostopic pub /gazebo/set_model_state gazebo_msgs/ModelState '{model_name: football, pose: { position: { x: -1, y: 5, z: 1 }, orientation: {x: 0, y: 0, z: 1, w: 0 } } }'
rostopic pub /gazebo/set_model_state gazebo_msgs/ModelState '{model_name: NuBot1, pose: { position: { x: -11, y: 0, z: 0 }, orientation: {x: -1, y: 0, z: 0, w: 0 } } }'
