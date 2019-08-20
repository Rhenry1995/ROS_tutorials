# ROS tutorial
## Introduction
This is a brief tutorial for how to setup a ROS package. It discusses the how to setup Robotic operating system (ROS) and create nodes and packages.

Currently this only gives an overview on the C++ elements of ROS. It will be altered in the future to include python based ROS.

## Important
One very important note for everything thing to run smoothly. This package, like all other ROS packages. Must be within a Catkin workspace. This is discussed in greater detain on the [ROS wiki](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). The parent directory must be `~/catkin_ws/src`. It is standard notation to have your workspace simply called `catkin_ws`

To quickly do this, run the following commands.
```
mkdir -p ~/catkin_ws/src
mv mv ../ROS_tutorials ~/catkin_ws/src
cd ~/catkin_ws/src/ROS_tutorials
```

## ROS Install
Currently is it set to install ROS for ubuntu 16.04 and 18.04. These are ROS versions kinetic and melodic respectively. A script has been created to install any required ROS programs with minimal user interaction. To run the script enter the following commands into terminal.

```
chmod +x ros_insall.sh
./ros_install.sh
```

The above script will also build your packages within your workspace and run it. If it has been successful it should show a continuous stream of `Install Successful`

Alternatively, you can install ROS by following the steps found on the [ROS website](http://wiki.ros.org/ROS/Installation).

## Building the packages
Now that ROS is running. Any time that you make any changes to any of your workspace you must now build the package. This can be done when inside of the catkin workspace by running the following command.

`catkin build`

If there are any compile errors with your code. These will be displayed as with any GCC errors.

## Running your code
There are 2 main method for running ROS packages and each with their own benefits.

1) The first method is using `rosrun`. This will run a single node and is very beneficial for the testing process of new nodes. One important point with `rosrun` is that you will need a second terminal with the command `roscore` running.

When this is done you will be able to run your code. The base structure of a `rosrun` is `rosrun package node param:=value`. To run our example code run the following

`rosrun ROS_tutorials ROS_tutorials_node`

For more information on `rosrun` fo to the following [link](http://wiki.ros.org/rosbash#rosrun)

2) The second method that can be used to run ROS nodes is `roslaunch`. The benefit of this over its run counterpart. Is firstly that you do not have to run a `roscore` along side this (but this is sometimes advised). Secondly as it you can run multiple nodes from one command.

Any launch file that you create should be placed in the launch folder. To run a launch file, the command is similar to that of `rosrun`. The structure is `roslaunch package launch_file.launch`. To run our example run the following command

`roslaunch ROS_tutorials example_launch_file.launch`

For more detail on `roslaunch` go to the following [link](http://wiki.ros.org/roslaunch)

## Further Points
Now that you have ROS working here are a couple of things might help

### Useful commands
These are just a couple of useful commands that you can use for making you life slightly easier.

- `roscd` is a command similar to that of `cd` except it will go directly to a ROS package. The usage of it is `roscd package`. A detailed description can be found [here](http://wiki.ros.org/rosbash#roscd)
- `rostopic` allows you to subscribe and publish to any topic that is running on the current `roscore`. This is extremely beneficial for the debugging process. There are a number of sub commands that run alongside `rostopic`. A comprehenistve description of `rostopic` [here](http://wiki.ros.org/rostopic)
	- `list` will list all current topics. The structure of the commange is `rostopic list`
	- `echo` will subscribe to a particular topic and display the message when one is published. The structure of the command is `rostopic echo <topic>`
	- `info` will give information about a particular topic. This includes its message type. As well as nodes that publish and subscribe to it. The structure of the command is `rostopic info <topic>`
	- `pub` will allow you to publish to a topic which can be incredibly useful for debugging. The structure of the command is `rostopic pub <topic> <message type> <message>`
- `roscore` begins the core ROS module which is required for you to run nodes. This is not necessary when using `roslaunch`, but can be beneficial if running multiple at once. This will run a `roscore` as part of the first launch file that is ran. Therefore when it stops the `roscore` will also stop, breaking your whole system. It is therefore beneficial to have a `roscore` constantly running in the background.
- `roslaunch` already discussed about is used to run launch files
- `rosrun` already discussed. Runs individual nodes but requires a `roscore` running in the background.
