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

	std::vector<cv::Mat> trainImgs;
	std::vector<int> trainLabels;
	cv::Mat trainHOG;

	cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();

	cv::Mat img_denoise;
	std::vector<cv::Mat> imgs_mser;
	cv::Mat testHOG;


	// Image Subscriber
	ros::Subscriber sub = n.subscribe("/camera/rgb/image_raw", 
		1, &classifier::imageCallback, &visual);


	///////// TRAINING ////////////
	// Load training data and resize
	visual.loadTrainingImgs(trainImgs, trainLabels);

	// HOG features of training images
	trainHOG = visual.HOG_Features(hog, trainImgs);

	// Train SVM and save model
	visual.SVMTraining(svm, trainHOG, trainLabels);


	//////// CLASSIFICATION /////////
	while (ros::ok()) {
		if (!visual.imagen.empty()) {
			
			// Get the detections
			img_denoise = visual.deNoise(visual.imagen);
			imgs_mser = visual.MSER_Features(visual.imagen);
			
			for (int i = 0; i < imgs_mser.size(); i++) {
				cv::namedWindow("view2");
				imshow("view2", imgs_mser[i]);

				// HOG features of detections
				if (imgs_mser.size() != 0) {
					testHOG = visual.HOG_Features(hog, imgs_mser[i]);

					// Evaluate using the SVM


				}
			}
		}

		ros::spinOnce();
	}
	cv::destroyWindow("view2");

}