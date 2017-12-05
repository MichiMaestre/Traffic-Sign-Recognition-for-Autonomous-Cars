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
    turtle.flag = false;
    turtle.count = 0;
	while (ros::ok()) {
		if (turtle.flag == false)
			ros::spinOnce();

		std::cout << "Count: " << turtle.count << std::endl;

		if (turtle.flag == true) {
			
			// Publish velocity depending on type of sign
			// turtle.command(turtle.type, turtle.area, velocity);


			//////// FORWARD //////
			if (turtle.type == 1) {

				ROS_INFO_STREAM("FORWARD SIGN");
				ros::Time start = ros::Time::now();
				while (ros::Time::now() - start < ros::Duration(10.0)) {
					velocity.linear.x = 0.2;
            		velocity.angular.z = 0.0;
            		// std::cout << velocity.linear.x << std::endl;
            		pub.publish(velocity);
            		loop_rate.sleep();
            	}
            	start = ros::Time::now();
				while (ros::Time::now() - start < ros::Duration(3.0)) {
					velocity.linear.x = 0.0;
            		velocity.angular.z = 0.0;
            		// std::cout << velocity.linear.x << std::endl;
            		pub.publish(velocity);
            		loop_rate.sleep();
            	}
            	turtle.type = 0;
            	turtle.area = 0;
            	turtle.flag = false;
			}


			///////// TURN ////////
			if (turtle.type == 2) {

				ROS_INFO_STREAM("TURN SIGN");
				ros::Time start = ros::Time::now();
				while (ros::Time::now() - start < ros::Duration(10.0)) {
					velocity.linear.x = 0.2;
            		velocity.angular.z = 0.0;
            		// std::cout << velocity.linear.x << std::endl;
            		pub.publish(velocity);
            		loop_rate.sleep();
            	}
            	start = ros::Time::now();
				while (ros::Time::now() - start < ros::Duration(3.0)) {
					velocity.linear.x = 0.3;
            		velocity.angular.z = 0.5;
            		// std::cout << velocity.linear.x << std::endl;
            		pub.publish(velocity);
            		loop_rate.sleep();
            	}
            	turtle.type = 0;
            	turtle.area = 0;
            	turtle.flag = false;
			}

			///////// STOP ////////
			if (turtle.type == 3) {

				ROS_INFO_STREAM("STOP SIGN");
				ros::Time start = ros::Time::now();
				while (ros::Time::now() - start < ros::Duration(10.0)) {
					velocity.linear.x = 0.0;
            		velocity.angular.z = 0.0;
            		// std::cout << velocity.linear.x << std::endl;
            		pub.publish(velocity);
            		loop_rate.sleep();
            	}
            	turtle.type = 0;
            	turtle.area = 0;
            	turtle.flag = false;

            	break;
			}



		}

		if (turtle.flag == false) {
			velocity.linear.x = 0.3;
        	velocity.angular.z = 0.0;
        	pub.publish(velocity);
		}

		// std::cout << velocity.linear.x << std::endl;

		loop_rate.sleep();

	}
	return 0;
}

