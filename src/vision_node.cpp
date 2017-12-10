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
 *@file vision_node.cpp
 *@author Miguel Maestre Trueba
 *@brief Vision node where all the main functions of the detection and recognition happen.
 */

#include <cv_bridge/cv_bridge.h>
#include <cstdlib>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "ros/console.h"
#include "classifier.hpp"
#include "traffic_sign_recognition/sign.h"

/**
 *@brief Function main that runs the main algorithm of the traffic sign recognition.
 *@brief It reads images from the robot's camera using a ROS subscriber.
 *@brief A SVM is trained and used to classify new signs features found by the robot.
 *@brief The type of sign and the size of the detection are published using a custom message.
 *@param argc is the number of arguments.
 *@param argv is the arguments characters array.
 *@return 0
 */
int main(int argc, char **argv) {
    // Node creation
    ros::init(argc, argv, "classification");
    ros::NodeHandle n;

    // Initializations
    classifier visual;

    cv::HOGDescriptor hog(cv::Size(64, 64),
                      cv::Size(32, 32),
                      cv::Size(16, 16),
                      cv::Size(32, 32),
                      9, 1, -1, 0, 0.2,
                      1, 64, 1);
    ROS_INFO_STREAM("HOG Descriptor created");

    visual.imagen = cv::Mat::zeros(640, 480, CV_8UC3);
    std::vector<cv::Mat> trainImgs;
    std::vector<int> trainLabels;
    cv::Mat trainHOG;

    cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();
    ROS_INFO_STREAM("Support Vector Machine Created");
    double area;

    cv::Mat img_denoise;
    std::vector<cv::Mat> imgs_mser(100);
    cv::Mat testHOG;


    // Image Subscriber
    ros::Subscriber sub = n.subscribe("/camera/rgb/image_raw",
        1, &classifier::imageCallback, &visual);

    // Custom message Publisher
    ros::Publisher signPub = n.advertise<traffic_sign_recognition::sign>(
        "traffic", 1);
    traffic_sign_recognition::sign msg;

    ///////// TRAINING ////////////
    int trained = visual.trainStage(hog, svm, trainImgs, trainLabels);

    //////// CLASSIFICATION /////////
    ROS_INFO_STREAM("Detection and Classification started...");
    while (ros::ok()) {
        if (!visual.imagen.empty()) {
            // Denoise image with gaussian blur
            img_denoise = visual.deNoise(visual.imagen);

            // Get the detections using MSER
            imgs_mser = visual.MSER_Features(visual.imagen, area);

            // If there are detection in the frame:
            if (imgs_mser.size() != 0) {
                // HOG features of detections
                testHOG = visual.HOG_Features(hog, imgs_mser);

                // Evaluate using the SVM
                visual.traffic_sign = visual.SVMTesting(svm, testHOG);

                // Publish the type of sign through message
                msg.area = area;
                msg.sign_type = visual.traffic_sign;
                signPub.publish(msg);

                imgs_mser.clear();
            }
            // Visualization of the robot view with all the detections
            int flagviz = visual.visualization();
        }
        ros::spinOnce();
    }
    return 0;
}
