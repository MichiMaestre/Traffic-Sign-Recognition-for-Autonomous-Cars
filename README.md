# Traffic Sign Recognition for Autonomous Cars applications using a TurtleBot 2
[![Build Status](https://travis-ci.org/MichiMaestre/traffic_sign_recognition.svg?branch=master)](https://travis-ci.org/MichiMaestre/traffic_sign_recognition)
[![Coverage Status](https://coveralls.io/repos/github/MichiMaestre/traffic_sign_recognition/badge.svg?branch=master)](https://coveralls.io/github/MichiMaestre/traffic_sign_recognition?branch=master)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Overview

The objective of this project was to design and develop a traffic sign recognition algorithm  for autonomous vehicles applications. The self driving car market is growing at a very fast pace. Many companies are working in this problem trying to solve every aspect of it, so that autonomous cars can drive safely on the roads. It is a very complex problem due to the many aspects that it relies on: robotics, path planning, navigation, computer vision, mechanics, etc.

This project is focused mainly in the computer vision aspect of it, a crucial module. If an automated car is going to drive around unpredictable environments, it has to be able to perceive and detect every small detail that surrounds it. Since tests were needed, the algorithm ewas developed using ROS and tested in a robot. The chosen robot is a Turtlebot 2. 

The robot will be driving around a simulated world, searching for traffic signs with its camera. Any time a traffic sign is recognized, the vision algorithm will send to the robot a command on how does it have to react to that sign. For example, if the robot finds a "turn left" sign, it will sop in from of the sign and turn to the left instead of continue going forward or turning to another direction. 

The project was developed using ROS Kinetic, C++ and OpenCV 3.2.0. The following subsection of the Overview will explain the pipeline of the algorithm step by step.

## Algorithm

The simulated world was designed using Gazebo. It simulates a map with delimited roads and a few traffic signs so that the robot's behavior can be tested.
![world](https://github.com/MichiMaestre/traffic_sign_recognition/blob/master/Images/rsz_world_view.png)

The proposed algorithm follows a straight forward pipeline with several steps as shown in the following activity diagrams. A Support Vector Machine (SVM) was used to classify the signs, so the first activity diagram corresponds to the training of the SVM.
 
![activity_train](https://github.com/MichiMaestre/traffic_sign_recognition/blob/master/UML/revised/Training_ActivityUML_revised.png)

The second diagram describes the main algorithm of the project. It involves two different ROS nodes that communicate between each other using a custom message, defined in the msg directory of the repository. ROS publishers and subscribers were used. 

![activity_main](https://github.com/MichiMaestre/traffic_sign_recognition/blob/master/UML/revised/Main_ActivityUML_Revised.png)

#### Training and testing traffic signs
For this project, only three types of signs were used: stop sign, turn sign and forward sign. To train the SVM, numerous and different images from the [Belgian dataset](http://btsd.ethz.ch/shareddata/) were used.  For future work, more types of signs can be added to the training stage easily. Two examples of the type of images that were used can be seen below.

![stop](https://github.com/MichiMaestre/traffic_sign_recognition/blob/master/Images/train_2.png)

![turn](https://github.com/MichiMaestre/traffic_sign_recognition/blob/master/Images/train_1.png)

Once the SVM was trained, it detects the traffic signs in different images. The following images correspond to the robot's point of view of the workspace. A bounding box and the type of sign can be seen in the image to help visualizing the results. This information is then sent to the robot commander node, that, depending on the type of sign detected, will move the robot in one way or another.

![robot_view1](https://github.com/MichiMaestre/traffic_sign_recognition/blob/master/Images/robot_view1.png)

![robot_view2](https://github.com/MichiMaestre/traffic_sign_recognition/blob/master/Images/robot_view2.png)


## Solo Iterative Process

Since this is a single programmer project, the Solo Iterative Process (SIP) is used to manage it. A product backlog, iteration backlog and work log(time log and code defect log) are used as structure of the whole project. The following link contains these logs. They will be updated through the whole development of the project.

[SIP Logs](https://docs.google.com/spreadsheets/d/1DNZGOJKaiTatEUviT9Bw6kG5VXIltuus_LCciD0hvZM/edit#gid=756841136)

The project will consist of three iterations:

* The release for Iteration 1 will be the initialization and design of the package. This includes the research of necessary topics to accomplish the task, design of UMLs and initialization of package files.

* Iteration 2 will consist of the development of the whole algorithm and creation of the workspace. 

* Iteration 3 will release a functioning demo of the project, tests and the documentation.

## Sprint Planning

[Sprint Planning Notes](https://docs.google.com/a/terpmail.umd.edu/document/d/1WolcIaDy4VU08c1Y5FMh5BDelXJmd_aV6ZwrNPDOnlo/edit?usp=sharing)

## Dependencies

* Ubuntu 16.04
* ROS Kinetic
* Catkin
* Gazebo
* Turtlebot Gazebo package
* Packages included in ROS Kinetic:
	* roscpp
	* std_msgs
	* genmsg
	* geometry_msgs
	* OpenCV3 (standard version included in ROS Kinetic, no need to install other version)
	* cv_bridge

## How to build

To build the package, follow the next steps:

```
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
git clone https://github.com/MichiMaestre/traffic_sign_recognition.git
cd ..
catkin_make
cp -R ~/ros_ws/src/traffic_sign_recognition/Images/Training_Images/ ~/ros_ws/devel/lib/traffic_sign_recognition/
cp -R ~/ros_ws/src/traffic_sign_recognition/Images/test_imgs/ ~/ros_ws/devel/lib/traffic_sign_recognition/
```
## How to run

To run the package, follow the next steps:

```
cd ~/ros_ws
source devel/setup.bash
roslaunch traffic_sign_recognition demo.launch
```

## Testing

To run the tests, follow the next steps:

```
cd ~/ros_ws
source devel/setup.bash
catkin_make run_tests
```
The command takes some time to finish.

## Recording a bag file

By default, recording the demo in a bag file is disabled. If wanted, all the topics except for the camera ones can be recorded by following the next commands:

```
cd ~/ros_ws
source devel/setup.bash
roslaunch traffic_sign_recognition demo.launch rosbagFlag :=true
```

This will record the data in  `TSR.bag` in the ~/.ros folder. To access it and see its information:

```
cd ~/.ros
rosbag info TSR.bag
```

## Known issues/bugs

The only two detected issues for now:

* When running the demo, false classification of a sign can happen. This will take the robot to, for example, take a turn when there's no turn sign. From all the simulations, this only happened once.

* The Gazebo model of turtlebot drifts towards the right when moving forward for a long distance. This had to be rectified in the stops the robot does during the demo.

## License

MIT License

Copyright (c) 2017 Miguel Maestre Trueba

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
