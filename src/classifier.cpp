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
 *@file classifier.cpp
 *@author Miguel Maestre Trueba
 *@brief Class classifier definition code
 */

#include <cstdlib>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "ros/console.h"
#include "classifier.hpp"

void classifier::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
	try {
    	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    	imagen = cv_ptr->image;
    	// cv::imshow("view", imagen);
    	cv::waitKey(30);
  	}
  	catch (cv_bridge::Exception& e) {
    	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  	}
}

cv::Mat classifier::deNoise(cv::Mat inputImage) {
  	cv::Mat output;

  	cv::GaussianBlur(inputImage, output, cv::Size(3, 3), 0, 0);

  	return output;
}

std::vector<cv::Mat> classifier::MSER_Features(cv::Mat img, double &area) {
	cv::Mat bgr[3];
	cv::Mat red_blue;
	cv::Mat rb_binary;
	cv::Mat detection;
	cv::Size size(64, 64);
	std::vector<cv::Mat> detections;

	// Get normalized images with Red and Blue
	split(img, bgr);

	cv::Mat red_norm = 255*(bgr[2] / (bgr[0] + bgr[1] + bgr[2]));
	cv::Mat red;
	red_norm.convertTo(red, CV_8UC1);
	cv::Mat blue_norm = 255*(bgr[0] / (bgr[0] + bgr[1] + bgr[2]));
	cv::Mat blue;
	blue_norm.convertTo(blue, CV_8UC1);

	max(red, blue, red_blue);

	threshold(red_blue, rb_binary, 200, 255, cv::THRESH_BINARY);

	// MSER
	cv::Ptr<cv::MSER> ms = cv::MSER::create(50, 1000, 14400, 0.9, 0.1, 200, 1.01, 0.1, 1);
	std::vector<std::vector<cv::Point> > regions;
	std::vector<cv::Rect> mser_bbox;
	ms->detectRegions(rb_binary, regions, mser_bbox);

	for (int i = 0; i < regions.size(); i++) {
		// Ratio filter of detected regions
		double ratio = static_cast<double>(mser_bbox[i].height) / static_cast<double>(mser_bbox[i].width);

		if (ratio > 0.9 && ratio < 1.1) {
			// Crop bounding boxes to get new images
        	detection = img(mser_bbox[i]);

        	rectangle(img, mser_bbox[i], CV_RGB(255, 0, 0));

        	area = static_cast<double>(mser_bbox[i].height) * static_cast<double>(mser_bbox[i].width);
        	// std::cout << area << std::endl;

        	cv::namedWindow("view2");
			imshow("view2", img);

        	// Resize images  to fit the trained data
        	cv::resize(detection, detection, size);

        	// Output the vector of images
        	detections.push_back(detection);
		}
		else
			area = 0;
    }
	return detections;
}

cv::Mat classifier::HOG_Features(cv::HOGDescriptor hog, std::vector<cv::Mat> imgs) {

	// CHANGE TO IMGS INSTEAD OF SINGLE IMG
	// TRAINHOG MAT SHOULD HAVE ALL THE INFORMATION
	std::vector<std::vector<float> > HOG;

	for (int i = 0; i < imgs.size(); i++) {
		std::vector<float> descriptor;
		hog.compute(imgs[i], descriptor);
		HOG.push_back(descriptor);
	}

	// Convert HOG features vector to Matrix
	cv::Mat signMat(HOG.size(), HOG[0].size(), CV_32FC1);
	// std::cout << signMat.size() << std::endl;
	for (int i = 0; i < HOG.size(); i++) {
		for (int j = 0; j < HOG[0].size(); j++) {
			signMat.at<float>(i,j) = HOG[i][j]; 
		}
	}

	return signMat;
}

void classifier::loadTrainingImgs(std::vector<cv::Mat> &trainImgs, std::vector<int> &trainLabels) {

	cv::String pathname = "/home/michi/catkin_ws/src/traffic_sign_recognition/Training_Images/1";
	// cv::String pathname = "../../../src\\traffic_sign_recognition\\Training_Images\\1";
	std::vector<cv::String> filenames;
	cv::glob(pathname, filenames);
	cv::Size size(64, 64);

	// std::cout << filenames.size() << std::endl;
	for (int i = 0; i < filenames.size(); i++) {
		cv::Mat src = imread(filenames[i]);

		cv::resize(src, src, size);
		
		trainImgs.push_back(src);
		trainLabels.push_back(1);
	}

	cv::String pathname2 = "/home/michi/catkin_ws/src/traffic_sign_recognition/Training_Images/2";
	// cv::String pathname = "../../../src\\traffic_sign_recognition\\Training_Images\\1";
	std::vector<cv::String> filenames2;
	cv::glob(pathname2, filenames2);
	cv::Size size2(64, 64);

	// std::cout << filenames.size() << std::endl;
	for (int i = 0; i < filenames2.size(); i++) {
		cv::Mat src2 = imread(filenames2[i]);

		cv::resize(src2, src2, size2);
		
		trainImgs.push_back(src2);
		trainLabels.push_back(2);
	}

	cv::String pathname3 = "/home/michi/catkin_ws/src/traffic_sign_recognition/Training_Images/3";
	// cv::String pathname = "../../../src\\traffic_sign_recognition\\Training_Images\\1";
	std::vector<cv::String> filenames3;
	cv::glob(pathname3, filenames3);
	cv::Size size3(64, 64);

	// std::cout << filenames.size() << std::endl;
	for (int i = 0; i < filenames3.size(); i++) {
		cv::Mat src3 = imread(filenames3[i]);

		cv::resize(src3, src3, size3);
		
		trainImgs.push_back(src3);
		trainLabels.push_back(3);
	}
}

void classifier::SVMTraining(cv::Ptr<cv::ml::SVM> &svm, cv::Mat trainHOG, std::vector<int> trainLabels) {
	// Set parameters
	svm->setGamma(0.50625);
	svm->setC(12.5);
	svm->setKernel(cv::ml::SVM::RBF);
	svm->setType(cv::ml::SVM::C_SVC);
	cv::Ptr<cv::ml::TrainData> td = cv::ml::TrainData::create(trainHOG, cv::ml::ROW_SAMPLE, trainLabels);
	svm->train(td);
}

float classifier::SVMTesting(cv::Ptr<cv::ml::SVM> &svm, cv::Mat testHOG) {
	cv::Mat answer;

	svm->predict(testHOG, answer);

	std::cout << answer.rows << std::endl;
	for(int i = 0; i < answer.rows; i++) {
		// std::cout << "Label: " << answer.at<float>(i,0) << std::endl;
		return answer.at<float>(i,0);
	}
}
