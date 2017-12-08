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
 *@file test_robot.cpp
 *@author Miguel Maestre Trueba
 *@brief Unit test code for navig_node.cpp
 */

#include <gtest/gtest.h>
#include "ros/ros.h"
#include <vector>
#include "robot.hpp"
#include "traffic_sign_recognition/sign.h"

// bool testing_forwardmove() {
// 	ros::NodeHandle n;

// 	robot turtle;

// 	geometry_msgs::Twist velocity;
// 	ros::Publisher pub = n.advertise<geometry_msgs::Twist>
//         ("/cmd_vel_mux/input/teleop", 1000);
//     ros::Rate loop_rate(10);


//     turtle.type = 1;
//     turtle.area = 5500;


//     turtle.command(velocity, pub, loop_rate);

//     return turtle.flag;
// }

// bool testing_turnmove() {
// 	ros::NodeHandle n;

// 	robot turtle;

// 	geometry_msgs::Twist velocity;
// 	ros::Publisher pub = n.advertise<geometry_msgs::Twist>
//         ("/cmd_vel_mux/input/teleop", 1000);
//     ros::Rate loop_rate(10);


//     turtle.type = 2;
//     turtle.area = 5500;


//     turtle.command(velocity, pub, loop_rate);

//     return turtle.flag;
// }

bool testing_moves(float sign_type, double Area, bool Flag) {
	ros::NodeHandle n;

	robot turtle;

	geometry_msgs::Twist velocity;
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>
        ("/cmd_vel_mux/input/teleop", 1000);
    ros::Rate loop_rate(10);


    turtle.type = sign_type;
    turtle.area = Area;
    turtle.flag = Flag;


    turtle.command(velocity, pub, loop_rate);

    return turtle.flag;
}

float test_callback(double Area) {
	ros::NodeHandle n;

	robot turtle;

	traffic_sign_recognition::sign msg;
	msg.sign_type = 1;
	msg.area = Area;

	turtle.signCallback(msg);

	return turtle.area;
}


TEST(TestCommand, ForwardTest) {
	bool type1 = testing_moves(1, 5500, true);
   
    EXPECT_EQ(false, type1);
}

TEST(TestCommand, TurnTest) {
	bool type2 = testing_moves(2, 5500, true);
   
    EXPECT_EQ(false, type2);
}

TEST(TestCommand, StopTest) {
	bool type3 = testing_moves(3, 5500, true);
   
    EXPECT_EQ(false, type3);
}

TEST(TestCommand, CallbackTest) {
	float area1 = test_callback(5500);
	float area2 = test_callback(4500);
   
    EXPECT_EQ(0, area1);
    EXPECT_EQ(4500, area2);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_robot");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}