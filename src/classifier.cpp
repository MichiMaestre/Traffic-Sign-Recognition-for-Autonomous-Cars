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

std::vector<cv::Mat> classifier::MSER_Features(cv::Mat img) {
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

	// threshold(red_blue, rb_binary, 200, 255, cv::THRESH_BINARY);

	// MSER
	cv::Ptr<cv::MSER> ms = cv::MSER::create(8, 1000, 14400, 0.2, 0.7, 200, 1.01, 0.003, 5);
	std::vector<std::vector<cv::Point> > regions;
	std::vector<cv::Rect> mser_bbox;
	ms->detectRegions(red_blue, regions, mser_bbox);

	for (int i = 0; i < regions.size(); i++) {
		// Ratio filter of detected regions
		double ratio = static_cast<double>(mser_bbox[i].height) / static_cast<double>(mser_bbox[i].width);

		if (ratio > 0.8 && ratio < 1.2) {
        	rectangle(img, mser_bbox[i], CV_RGB(255, 0, 0));

        	// Crop bounding boxes to get new images
        	detection = img(mser_bbox[i]);

        	// Resize images  to fit the trained data
        	cv::resize(detection, detection, size);

        	// Output the vector of images
        	detections.push_back(detection);
		}
    }

	return detections;
}