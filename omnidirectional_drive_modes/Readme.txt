joystick_control: Python script that connects to a controller to publish directions, rotations, and speed to a ROS node.

nodered_control: Python script that reads directions, rotations, or speeds sent from Node-RED via MQTT and forwards them to a ROS node

nodes_to_cmdvel: Python script that subscribes to ROS nodes sent from the nodered_control and joystick_control scripts and publishes them as a /cmd_vel node in ROS.

visual_cmdvel: Python script that graphically represents the movement being published by the /cmd_vel node.



My way of running them is to first execute the joystick_control or nodered_control nodes, and once they have been executed, I then run the Python scripts nodes_to_cmdvel and visual_cmdvel