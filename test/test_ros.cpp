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
 *@file test_ros.cpp
 *@author Miguel Maestre Trueba
 *@brief Unit tests for publishers and subscribers
 */

#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <gtest/gtest.h>
#include <vector>
#include <boost/thread.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "classifier.hpp"
#include "traffic_sign_recognition/sign.h"
#include "testclass.hpp"

/**
 *@brief Test subscriber and publisher to custom message sign
 *@param none
 *@return none
 */
TEST(TestRos, PubSubTest) {
    ros::NodeHandle n;

    testclass pubsub;

    ros::Rate loop_rate(2);

    // Publisher to sign messages
    ros::Publisher signPub = n.advertise<traffic_sign_recognition::sign>(
        "traffic", 100);

    // Subscriber to traffic topic
    ros::Subscriber sub = n.subscribe("/traffic",
        100, &testclass::signCallback, &pubsub);

    // Publish sign messages
    traffic_sign_recognition::sign msg;
    msg.area = 5000;
    msg.sign_type = 1;
    signPub.publish(msg);


    ros::spinOnce();
    loop_rate.sleep();

    // Tests
    EXPECT_EQ(1, sub.getNumPublishers());
    EXPECT_EQ(1, signPub.getNumSubscribers());
    EXPECT_EQ(1, pubsub.type);
    EXPECT_EQ(5000, pubsub.area);
}

/**
 *@brief Test velocity publisher
 *@param none
 *@return none
 */
TEST(TestRos, velTest) {
    ros::NodeHandle n;

    testclass pubsub;

    ros::Rate loop_rate(2);

    // Publisher to velocity commands
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>
        ("/cmd_vel_mux/input/teleop", 1000);

    // Suscriber to velocity topic
    ros::Subscriber sub = n.subscribe("/cmd_vel_mux/input/teleop",
        1000, &testclass::velCallback, &pubsub);

    // Publish messages
    geometry_msgs::Twist msg;
    msg.linear.x = 0.2;
    msg.angular.z = 0.0;
    pub.publish(msg);


    ros::spinOnce();
    loop_rate.sleep();

    // Tests
    EXPECT_EQ(1, sub.getNumPublishers());
    EXPECT_EQ(1, pub.getNumSubscribers());
    EXPECT_EQ(0.2, pubsub.vel);
}

/**
 *@brief Function that spins callbacks  in tests
 *@param none
 *@return none
 */
void pthread(void) {
    ros::Rate loop_rate(2);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

/**
 *@brief Main function that runs all tests in the file
 *@param argc is the number of arguments
 *@param argv is the arguments characters array
 *@return Tests results
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "test_ros");
    testing::InitGoogleTest(&argc, argv);
    ros::NodeHandle nh;

    boost::thread t(pthread);
    int tests = RUN_ALL_TESTS();
    ros::shutdown();
    t.join();

    return tests;
}
