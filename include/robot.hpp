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
 *@file robot.hpp
 *@author Miguel Maestre Trueba
 *@brief Header file with definitions for class robot.
 */

#pragma once

#include <geometry_msgs/Twist.h>
#include <vector>
#include "ros/ros.h"
#include "traffic_sign_recognition/sign.h"

class robot {
 private:
    int count = 0;  /// Counter used to get number of signs found in the world

 public:
    bool flag;  /// Signal that tells node when to spin and when to stop
    float type;  /// Type of traffic sign being recognized
    double area;  /// Size of the bouding box of the detected traffic sign

    /// SIGN MSG CALLBACK
    /**
     *@brief Callback used in the subscriber for the traffic topic. Gets the message and stores its info in the classes variables
     *@param msg is the custom sign message. Contains type of sign and its size in the image
     *@return none
     */
    void signCallback(traffic_sign_recognition::sign msg);

    // ROBOT BEHAVIOR
    /**
     *@brief Publishes velocity commands for a certain time depending on what type of sign has been detected
     *@param velocity is the Twist() message that has to be published
     *@param pub is publisher to "/cmd_vel_mux/input/teleop" topic
     *@param loop_rate is a ros::Rate of 10 ms
     *@return none
     */
    void command(geometry_msgs::Twist &velocity,
        ros::Publisher &pub, ros::Rate &loop_rate);
};
