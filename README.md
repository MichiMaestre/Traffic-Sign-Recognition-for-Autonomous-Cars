# Traffic Sign Recognition for Autonomous Cars applications using a TurtleBot 2
[![Build Status](https://travis-ci.org/MichiMaestre/traffic_sign_recognition.svg?branch=master)](https://travis-ci.org/MichiMaestre/traffic_sign_recognition)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Overview

The objective of this project was to design and develop a traffic sign recognition algorithm  for autonomous vehicles applications. The self driving car market is growing at a very fast pace. Many companies are working in this problem trying to solve every aspect of it, so that autonomous cars can drive safely on the roads. It is a very complex problem due to the many aspects that it relies on: robotics, path planning, navigation, computer vision, mechanics, etc.

This project is focused mainly in the computer vision aspect of it, a crucial module. If an automated car is going to drive around unpredictable environments, it has to be able to perceive and detect every small detail that surrounds it. Since tests were needed, the algorithm ewas developed using ROS and tested in a robot. The chosen robot is a Turtlebot 2. 

The robot will be driving around a simulated world, searching for traffic signs with its camera. Any time a traffic sign is recognized, the vision algorithm will send a command to the robot telling it how does it have to react to that sign. For example, if the robot finds a "turn left" sign, it will stop in front of the sign and turn to the left instead of continue going forward or turning to another direction. 

The project was developed using ROS Kinetic, C++ and OpenCV 3.2.0. The following subsection of the Overview will explain the pipeline of the algorithm step by step.

## Algorithm

The simulated world was designed using Gazebo. It simulates a map with delimited roads and a few traffic signs so that the robot's behavior can be tested.
![world](https://github.com/MichiMaestre/traffic_sign_recognition/blob/master/Images/rsz_world_view.png)

The proposed algorithm follows a straight forward pipeline with several steps as shown in the following activity diagrams. A Support Vector Machine (SVM) was used to classify the signs, so the first activity diagram corresponds to the training of the SVM.
 
![activity_train](https://github.com/MichiMaestre/traffic_sign_recognition/blob/master/UML/revised/Training_ActivityUML_revised.png)

The second diagram describes the main algorithm of the project. It involves two different ROS nodes that communicate between each other using a custom message, defined in the msg directory of the repository. ROS publishers and subscribers were used. 

![activity_main](https://github.com/MichiMaestre/traffic_sign_recognition/blob/master/UML/revised/Main_ActivityUML_Revised.png)

#### Training and testing traffic signs
For this project, only three types of signs were used: stop sign, turn sign and forward sign. To train the SVM, numerous and different images from the [Belgian dataset](http://btsd.ethz.ch/shareddata/) were used.  More types of signs can be added to the training stage easily for future work. Two examples of the type of images that were used can be seen below.

![stop](https://github.com/MichiMaestre/traffic_sign_recognition/blob/master/Images/train_2.png)

![turn](https://github.com/MichiMaestre/traffic_sign_recognition/blob/master/Images/train_1.png)

Once the SVM was trained, it is able to detect the traffic signs in different images. The following images correspond to the robot's point of view of the workspace. A bounding box and the type of sign can be seen in the image to help visualizing the results. This information is then sent to the robot commander node, that depending on the type of sign detected, will move the robot in one way or another.

![robot_view1](https://github.com/MichiMaestre/traffic_sign_recognition/blob/master/Images/robot_view1.png)

![robot_view2](https://github.com/MichiMaestre/traffic_sign_recognition/blob/master/Images/robot_view2.png)

## Presentation

* [Presentation video](https://www.youtube.com/watch?v=1S6G-v9FFRI&feature=youtu.be)

* [Slides](https://docs.google.com/a/terpmail.umd.edu/presentation/d/1Ou_m6cw1QjKVDYbkBXsE9HtOTUIVvummQW5jkTnbYrw/edit?usp=sharing)

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
cp -R ~/ros_ws/src/traffic_sign_recognition/Images/test_imgs/ ~/ros_ws/devel/lib/traffic_sign_recognition/
source devel/setup.bash
catkin_make run_tests
```
The command takes some time to finish.
If first two tests fail, rerun the command `catkin_make run_tests`.

#### Code coverage

The current coverage for the project is 98.9% (coverage of files written for this project). The details can be seen [here](http://htmlpreview.github.io/?https://github.com/MichiMaestre/traffic_sign_recognition/blob/master/results/out/index.html)

To check the code coverage in the project, follow the next steps:
```
cd ~/ros_ws/build
lcov --directory . --capture --output-file coverage.info
lcov --list coverage.info
```
This last command will output the coverage of each file in the terminal. To generate an html file with the more information:
```
genhtml coverage.info --output-directory out
```
This will generate a folder called `out`. Inside the folder, open `index.html` to view the details of the coverage.

## Recording a bag file

By default, recording the demo in a bag file is disabled. If wanted, all the topics except for the camera ones can be recorded by following the next commands:

```
cd ~/ros_ws
source devel/setup.bash
roslaunch traffic_sign_recognition demo.launch rosbagFlag:=true
```

This will record the data in `TSR.bag` in the ~/.ros folder. To access it and see its information:

```
cd ~/.ros
rosbag info TSR.bag
```

## Doxygen Documentation

The Dogymen generated documentation can be checked here: [Traffic Sign Doxygen Documents](http://htmlpreview.github.io/?https://github.com/MichiMaestre/traffic_sign_recognition/blob/master/docs/html/index.html)

To generate Doxygen Documentation in HTML and LaTEX, follow the next steps:

```
cd <path to repository>
mkdir <documentation_folder_name>
cd <documentation_folder_name>
doxygen -g <config_file_name>
```
Inside the configuration file, update:
```
PROJECT_NAME = 'your project name'
INPUT = ../src ../include ../test
```
Run and generate the documents by running the next command:
```
doxygen <config_file_name>
```

## Known issues/bugs

The only two detected issues at the moment:

* When running the demo, false classification of a sign can happen. This will make the robot, for example, to take a turn when there's no turn sign. This only happened in one experiment. It can also happen that the robot does not detect a traffic signs and ends up crashing. From different trials, it can be inferred that these issues usually happen when the computer is experiencing a high level of computation. When this happens, rerun the demo and it should work.

* The Gazebo model of turtlebot drifts towards the right when moving forward for a long distance. This had to be rectified in the stops the robot does during the demo.

* When running tests, first two tests fail sometimes. Rerun the command `catkin_make run_tests` to make the tests pass.

## Author

Miguel Angel Maestre Trueba. Second year M. Eng in Robotics graduate student, working as research assistant in robotics/computer vision applications in the Autonomy, Robotics and Cognition Lab in the University of Maryland, College Park.

## License

MIT License

Copyright (c) 2017 Miguel Maestre Trueba

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
