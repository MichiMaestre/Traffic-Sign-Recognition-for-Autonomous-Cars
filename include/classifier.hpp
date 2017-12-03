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
 *@file classifier.hpp
 *@author Miguel Maestre Trueba
 *@brief Header file for class classifier
 */

#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"

class classifier {
public:
	cv::Mat imagen;

	void imageCallback(const sensor_msgs::ImageConstPtr& msg);

	cv::Mat deNoise(cv::Mat inputImage);

	std::vector<cv::Mat> MSER_Features(cv::Mat img);

	cv::Mat HOG_Features(cv::HOGDescriptor hog, std::vector<cv::Mat> imgs);

	void loadTrainingImgs(std::vector<cv::Mat> &trainImgs, std::vector<int> &trainLabels);

	void SVMTraining(cv::Ptr<cv::ml::SVM> &svm, cv::Mat trainHOG, std::vector<int> trainLabels);

	void SVMTesting(cv::Ptr<cv::ml::SVM> &svm, cv::Mat testHOG);
};