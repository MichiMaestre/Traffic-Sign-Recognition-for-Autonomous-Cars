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
 *@file test_vision.cpp
 *@author Miguel Maestre Trueba
 *@brief Unit test code for vision_node.cpp
 */

#include <gtest/gtest.h>
#include "ros/ros.h"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include "opencv2/opencv.hpp"
#include "classifier.hpp"
#include "traffic_sign_recognition/sign.h"

float testing_stop() {
	ros::NodeHandle n;

	// Initializations
	classifier visual;

	cv::HOGDescriptor hog(cv::Size(64, 64), 
					  cv::Size(32,32), 
					  cv::Size(16,16), 
					  cv::Size(32,32), 
					  9, 1,-1, 0, 0.2, 
					  1, 64, 1);

	std::vector<cv::Mat> trainImgs;
	std::vector<int> trainLabels;
	cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();
	double area;

	visual.trainStage(hog, svm, trainImgs, trainLabels);

	visual.imagen = cv::imread("/home/michi/catkin_ws/src/traffic_sign_recognition/test/stop.png");
	
	if (!visual.imagen.empty()) {
		cv::Mat img_denoise = visual.deNoise(visual.imagen);
		std::vector<cv::Mat> imgs_mser = visual.MSER_Features(visual.imagen, area);
		cv::Mat testHOG = visual.HOG_Features(hog, imgs_mser);
		float traffic_sign = visual.SVMTesting(svm, testHOG);
		return traffic_sign;
	}
	else 
		return 0;
}

float testing_turn() {
	ros::NodeHandle n;

	// Initializations
	classifier visual;

	cv::HOGDescriptor hog(cv::Size(64, 64), 
					  cv::Size(32,32), 
					  cv::Size(16,16), 
					  cv::Size(32,32), 
					  9, 1,-1, 0, 0.2, 
					  1, 64, 1);

	std::vector<cv::Mat> trainImgs;
	std::vector<int> trainLabels;
	cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();
	double area;

	visual.trainStage(hog, svm, trainImgs, trainLabels);

	visual.imagen = cv::imread("/home/michi/catkin_ws/src/traffic_sign_recognition/test/turn.png");
	
	if (!visual.imagen.empty()) {
		cv::Mat img_denoise = visual.deNoise(visual.imagen);
		std::vector<cv::Mat> imgs_mser = visual.MSER_Features(visual.imagen, area);
		cv::Mat testHOG = visual.HOG_Features(hog, imgs_mser);
		float traffic_sign = visual.SVMTesting(svm, testHOG);
		return traffic_sign;
	}
	else 
		return 0;
}

float testing_forward() {
	ros::NodeHandle n;

	// Initializations
	classifier visual;

	cv::HOGDescriptor hog(cv::Size(64, 64), 
					  cv::Size(32,32), 
					  cv::Size(16,16), 
					  cv::Size(32,32), 
					  9, 1,-1, 0, 0.2, 
					  1, 64, 1);

	std::vector<cv::Mat> trainImgs;
	std::vector<int> trainLabels;
	cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();
	double area;

	visual.trainStage(hog, svm, trainImgs, trainLabels);

	visual.imagen = cv::imread("/home/michi/catkin_ws/src/traffic_sign_recognition/test/forward.png");
	
	if (!visual.imagen.empty()) {
		cv::Mat img_denoise = visual.deNoise(visual.imagen);
		std::vector<cv::Mat> imgs_mser = visual.MSER_Features(visual.imagen, area);
		cv::Mat testHOG = visual.HOG_Features(hog, imgs_mser);
		float traffic_sign = visual.SVMTesting(svm, testHOG);
		return traffic_sign;
	}
	else 
		return 0;
}


TEST(TestSigns, stopTest) {
	float type = testing_stop();
   
    EXPECT_EQ(3, type);
}

TEST(TestSigns, turnTest) {
	float type = testing_turn();
   
    EXPECT_EQ(2, type);
}

TEST(TestSigns, forwardTest) {
	float type = testing_forward();
   
    EXPECT_EQ(1, type);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "test_vision");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}