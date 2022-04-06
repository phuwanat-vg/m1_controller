# dobot_m1
ROS  package for Dobot M1

Original repositories : https://github.com/HarvestX/dobot_m1

extension : Add IO topic to control the output of robot at IO "18". It can used to control the pump of vaccuum.  

**How to use**
1. Enter to directory "m1_launch" and open m1_param.yaml in config folder.
2. Change port name to dobot connection port.
3. run command : `roslaunch m1_controller m1_controller.launch`
4. After dobot start, you can view all topic with command `rostopic list`

Example file:
simple_control.py in src
