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
 *@brief Vision node
 */

#include <cstdlib>
#include <string>
// #include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "ros/console.h"
#include "classifier.hpp"
#include "traffic_sign_recognition/sign.h"


int main(int argc, char **argv) {
	// Node creation
	ros::init(argc, argv, "classification");
	ros::NodeHandle n;

	// Initializations
	classifier visual;

	cv::HOGDescriptor hog(cv::Size(64, 64), 
					  cv::Size(32,32), 
					  cv::Size(16,16), 
					  cv::Size(32,32), 
					  9, 1,-1, 0, 0.2, 
					  1, 64, 1);
	ROS_INFO_STREAM("HOG Descriptor created");

	std::vector<cv::Mat> trainImgs;
	std::vector<int> trainLabels;
	cv::Mat trainHOG;

	cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();
	ROS_INFO_STREAM("Support Vector Machine Created");
	double area;

	cv::Mat img_denoise;
	std::vector<cv::Mat> imgs_mser;
	cv::Mat testHOG;
	float traffic_sign;


	// Image Subscriber
	ros::Subscriber sub = n.subscribe("/camera/rgb/image_raw", 
		1, &classifier::imageCallback, &visual);

	// Msg Publisher
	ros::Publisher signPub = n.advertise<traffic_sign_recognition::sign>("traffic", 1);
	traffic_sign_recognition::sign msg;

	///////// TRAINING ////////////
	// ROS_INFO_STREAM("SVM Training Stage started...");
	// // Load training data and resize
	// visual.loadTrainingImgs(trainImgs, trainLabels);

	// // HOG features of training images
	// trainHOG = visual.HOG_Features(hog, trainImgs);

	// // Train SVM and save model
	// visual.SVMTraining(svm, trainHOG, trainLabels);
	// ROS_INFO_STREAM("SVM Training Stage completed");
	// ros::Duration(2).sleep();

	visual.trainStage(hog, svm, trainImgs, trainLabels);

	//////// CLASSIFICATION /////////
	ROS_INFO_STREAM("Detection and Classification started...");
	// cv::Mat testResponse;
	while (ros::ok()) {
		if (!visual.imagen.empty()) {
			
			// Get the detections
			img_denoise = visual.deNoise(visual.imagen);
			imgs_mser = visual.MSER_Features(visual.imagen, area);
			// msg.area = area;
			// std::cout << area << std::endl;

			// HOG features of detections
			if (imgs_mser.size() != 0) {
				testHOG = visual.HOG_Features(hog, imgs_mser);

				// Evaluate using the SVM
				traffic_sign = visual.SVMTesting(svm, testHOG);
				std::cout << "Label: " << traffic_sign << std::endl;
				// ROS_INFO_STREAM("Label: %f", traffic_sign);

				// Publish the type of sign through message
				msg.area = area;
				msg.sign_type = traffic_sign;
				signPub.publish(msg);
			}
		}
		ros::spinOnce();
	}
	cv::destroyWindow("view2");

}