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
 *@file testclass.hpp
 *@author Miguel Maestre Trueba
 *@brief Definition for class testclass
 */

#pragma once

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "traffic_sign_recognition/sign.h"

/**
 *@brief Definition of the testclass class. It contains mock callbacks to test publishers and subscribers. 
 */
class testclass {
 public:
    double area;  /// Area of the sign detection
    float type;  /// Type of sign detected
    double vel;  /// Velocity given to test subscriber

    /**
     *@brief Callback used in tests to check if traffic topic is published correctly
     *@param msg is the custom message of type sign with traffic sign information
     *@return none
     */
    void signCallback(traffic_sign_recognition::sign msg);

    /**
     *@brief Callback used in tests to check if velocity is being published correctly
     *@param msg is Twist message with velocity type information
     *@return none
     */
    void velCallback(geometry_msgs::Twist msg);
};
