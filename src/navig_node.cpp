/** MIT License
Copyright (c) 2017 Miguel Maestre Trueba
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *
 *@copyright Copyright 2017 Miguel Maestre Trueba
 *@file navig_node.cpp
 *@author Miguel Maestre Trueba
 *@brief Navigation node
 */

#include <cstdlib>
#include <string>
// #include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "ros/console.h"
#include "robot.hpp"
#include "traffic_sign_recognition/sign.h"

int main(int argc, char **argv) {
	// Node creation
	ros::init(argc, argv, "robot_move");
	ros::NodeHandle n;

	// Initializations
	robot turtle;
	geometry_msgs::Twist velocity;

	// Subscriber
	ros::Subscriber sub = n.subscribe("/traffic", 
		1, &robot::signCallback, &turtle);

	// Publisher
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>
        ("/cmd_vel_mux/input/teleop", 1000);

    ros::Rate loop_rate(10);
	while (ros::ok()) {
		if (turtle.flag == false)
			ros::spinOnce();

		// Publish velocity depending on type of sign
		turtle.command(velocity, pub, loop_rate);

		loop_rate.sleep();

	}
	return 0;
}

