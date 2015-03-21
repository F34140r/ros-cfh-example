RoCKin@Work cfh Node Example
===============================

This is the repository of the example node for the cfh of RoCKIn competition (http://rockinrobotchallenge.eu).
Is tested under ROS Indigo. Is a beta version and under construction !


## Installation
To use this node, please install this packages

sudo apt-get install protobuf libprotoc-dev libprotobuf-dev protobuf-c-compiler protobuf-compiler libssl-dev libelf-dev libzmq3-dev

## Compiling the example node
### Cloning the Git repository:

    cd ~/catkin_ws/src/
    git clone https://github.com/F34140r/ros-cfh-example.git ./ros_cfh_example

### Compiling the node:

    cd .. & catkin_make

## Known Errors

Proto Message doesn't build correctly  

  Remove the cfh-msgs-example under devel of your catkin workspace  and try again
