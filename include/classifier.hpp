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
 *@brief Header file with definitions for class classifier.
 */

#pragma once

#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"

/**
 *@brief Definition of the classifier class. It contains all the functions and variables depicted in the
 *@brief UML Class diagram.
 *@brief It trains a SVM, reads images from topic, detects MSER and HOG features and classifies traffic sign images.
 */
class classifier {
 private:
    // DATASET LOADING
    /**
     *@brief Loads all the images from the dataset by type of sign and labels them
     *@param trainImgs is an output vector that contains all the training images
     *@param trainLabels is an output vector that contains all the labels of the training images
     *@return none
     */
    void loadTrainingImgs(std::vector<cv::Mat> &trainImgs,
        std::vector<int> &trainLabels);

    // TRAIN SVM
    /**
     *@brief Sets all the parameters of the SVM object and feeds it with the HOG features of the labeled training images. 
     *@param svm is the Support vector Machine object
     *@param trainHOG is the cv::Mat with the HOG features from the training images
     *@param trainLabels is the vector wuth all the labels for the training images
     *@return none
     */
    void SVMTraining(cv::Ptr<cv::ml::SVM> &svm,
        cv::Mat trainHOG, std::vector<int> trainLabels);

 public:
    cv::Mat imagen;  /// OpenCV Image from the camera given by the subscriber

    std::vector<cv::Rect> boxes;  /// Bounding boxes in the current frame

    float traffic_sign;  /// The label of the detections, outputed by the SVM

    // IMAGE CALLBACK
    /**
     *@brief Callback used in the subscriber for the camera topic. Gets the ROS Img msg and transforms it to an OpenCV img.
     *@param msg is the ROS Image msg
     *@return none
     */
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    /**
     *@brief Apply gaussian filter to the input image to denoise it
     *@param inputImage is the current image to be filtered
     *@return Blurred and denoised image
     */
    cv::Mat deNoise(cv::Mat inputImage);

    // MSER FEATURES
    /**
     *@brief Find MSER features in the image. First normalize image and binarize, then look for regions and finally, resize each regions image.
     *@param img is the current image where MSER features are detected
     *@param area is an output variable that stores the area of each region's bounding box 
     *@return Vector of all the images of detections 
     */
    std::vector<cv::Mat> MSER_Features(cv::Mat img, double &area);

    // HOG FEATURES
    /**
     *@brief Compute the HOG features in every image of a vector
     *@param hog is the HOG Descriptor object with all the parameters set up
     *@param imgs is the vector of images to get theHOG from
     *@return Matrix with the HOG Descriptor for all the images.
     */
    cv::Mat HOG_Features(cv::HOGDescriptor hog, std::vector<cv::Mat> imgs);

    // TRAINING
    /**
     *@brief Function that runs loadTrainImgs(), HOG_Features() and SVM Training, sequentially
     *@param hog is the HOG Descriptor object with all the parameters set up
     *@param svm is the Support vector Machine object
     *@param trainImgs is an output vector that contains all the training images
     *@param trainLabels is an output vector that contains all the labels of the training images
     *@return 1 if success
     */
    int trainStage(cv::HOGDescriptor &hog, cv::Ptr<cv::ml::SVM> &svm,
        std::vector<cv::Mat> &trainImgs, std::vector<int> &trainLabels);

    // CLASSIFICATION
    /**
     *@brief Feed SVM with the HOG Matrix of the current image and outputs its label.
     *@param testHOG is the HOG Descriptor object with all the parameters set up
     *@param svm is the Support vector Machine object
     *@return Labels of the tested set of features. Label of the sign being recognized.
     */
    float SVMTesting(cv::Ptr<cv::ml::SVM> &svm, cv::Mat testHOG);

    // VISUALIZATION
    /**
     *@brief Opens a vindow with the robot's view of the workspace. It outputs bouding box around detected signs and the name of the sign.
     *@param none
     *@return 1 if success
     */
    int visualization();
};
