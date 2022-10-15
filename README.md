# ascension_ros
ROS driver for NDI/Ascension trakSTAR/driveBAY2 devices

The package was tested in Ubuntu 16.04 with ROS-kinetic and Ubuntu 20.04 with ROS-Noetic.

# Getting 

First, make sure you have installed libusb, if not
```bash
sudo uapt-get install libusb-dev
```

## Create a workspace and clone this package
 
You can follow the instructions on how to create a workspace in catkin here http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Clone the repo to your src directory in the catkin workspace. Build the project through `catkin_make` or `catkin build`

## Copy USB permissions file

Copy the "99-libusb.rules" file into the path "/etc/udev/rules.d/" or you can copy by command: 

```bash
sudo cp ../src/ascension_ros/99-libusb.rules /etc/udev/rules.d/
```

Then reload the udev rules by:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Start the system

Start ROS through terminal: 
```bash
roscore
```

Set the source from the main catkin_ws by: 
```bash
source devel/setup.bash
```

Launch the demo of visualing the sensor pose in Rviz by: 
```bash
roslaunch ascension demo.launch
```

Launch the visualization of the sensor pose with respect to the snake robot in Rviz by: 
```bash
roslaunch ascension transform.launch
```

<!-- Or, start the system only and open with it with ImFusion (or any other preferred software). -->
