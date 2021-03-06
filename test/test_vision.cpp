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
 *@brief Unit test code for classifier class
 */

#include <cv_bridge/cv_bridge.h>
#include <gtest/gtest.h>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "classifier.hpp"
#include "traffic_sign_recognition/sign.h"

/**
 *@brief Function to test if stop sign is classified correctly
 *@param none
 *@return Label of the classified sign
 */
float testing_stop() {
    ros::NodeHandle n;

    // Initializations
    classifier visual;

    cv::HOGDescriptor hog(cv::Size(64, 64),
                      cv::Size(32, 32),
                      cv::Size(16, 16),
                      cv::Size(32, 32),
                      9, 1, -1, 0, 0.2,
                      1, 64, 1);

    std::vector<cv::Mat> trainImgs;
    std::vector<int> trainLabels;
    cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();

    // Train
    visual.trainStage(hog, svm, trainImgs, trainLabels);

    // Read images from testing images directory
    cv::String path1 = "./test_imgs/stop";
    std::vector<cv::String> img_stop;
    cv::glob(path1, img_stop);
    visual.imagen = cv::imread(img_stop[0]);

    // Classify
    if (!visual.imagen.empty()) {
        double area;

        cv::Mat img_denoise = visual.deNoise(visual.imagen);
        std::vector<cv::Mat> imgs_mser = visual.MSER_Features(
            visual.imagen, area);
        cv::Mat testHOG = visual.HOG_Features(hog, imgs_mser);
        float traffic_sign = visual.SVMTesting(svm, testHOG);
        return traffic_sign;
    }
}

/**
 *@brief Function to test if turn sign is classified correctly
 *@param none
 *@return Label of the classified sign
 */
float testing_turn() {
    ros::NodeHandle n;

    // Initializations
    classifier visual;

    cv::HOGDescriptor hog(cv::Size(64, 64),
                      cv::Size(32, 32),
                      cv::Size(16, 16),
                      cv::Size(32, 32),
                      9, 1, -1, 0, 0.2,
                      1, 64, 1);

    std::vector<cv::Mat> trainImgs;
    std::vector<int> trainLabels;
    cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();

    // Train
    visual.trainStage(hog, svm, trainImgs, trainLabels);

    // Read images from testing images directory
    cv::String path1 = "./test_imgs/turn";
    std::vector<cv::String> img_turn;
    cv::glob(path1, img_turn);
    visual.imagen = cv::imread(img_turn[0]);

    // Classify
    if (!visual.imagen.empty()) {
        double area;

        cv::Mat img_denoise = visual.deNoise(visual.imagen);
        std::vector<cv::Mat> imgs_mser = visual.MSER_Features(
            visual.imagen, area);
        cv::Mat testHOG = visual.HOG_Features(hog, imgs_mser);
        float traffic_sign = visual.SVMTesting(svm, testHOG);
        return traffic_sign;
    }
}

/**
 *@brief Function to test if forward sign is classified correctly
 *@param none
 *@return Label of the classified sign
 */
float testing_forward() {
    ros::NodeHandle n;

    // Initializations
    classifier visual;

    cv::HOGDescriptor hog(cv::Size(64, 64),
                      cv::Size(32, 32),
                      cv::Size(16, 16),
                      cv::Size(32, 32),
                      9, 1, -1, 0, 0.2,
                      1, 64, 1);

    std::vector<cv::Mat> trainImgs;
    std::vector<int> trainLabels;
    cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();

    // Train
    visual.trainStage(hog, svm, trainImgs, trainLabels);

    // Read images from testing images directory
    cv::String path1 = "./test_imgs/forw";
    std::vector<cv::String> img_forw;
    cv::glob(path1, img_forw);
    visual.imagen = cv::imread(img_forw[0]);

    // Classify
    if (!visual.imagen.empty()) {
        double area;

        cv::Mat img_denoise = visual.deNoise(visual.imagen);
        std::vector<cv::Mat> imgs_mser = visual.MSER_Features(
            visual.imagen, area);
        cv::Mat testHOG = visual.HOG_Features(hog, imgs_mser);
        float traffic_sign = visual.SVMTesting(svm, testHOG);
        return traffic_sign;
    }
}


/**
 *@brief Function to test if the SVM is trained correctly
 *@param none
 *@return 1 if succeeds
 */
int test_training() {
    // Initializations
    classifier visual;

    cv::HOGDescriptor hog(cv::Size(64, 64),
                      cv::Size(32, 32),
                      cv::Size(16, 16),
                      cv::Size(32, 32),
                      9, 1, -1, 0, 0.2,
                      1, 64, 1);
    cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();
    std::vector<cv::Mat> trainImgs;
    std::vector<int> trainLabels;

    // Train
    int trainedSVM = visual.trainStage(hog, svm, trainImgs, trainLabels);

    return trainedSVM;
}

/**
 *@brief Function to test if MSER features are detected correctly in an image
 *@param none
 *@return Set of images with the detections
 */
std::vector<cv::Mat> test_MSERGood() {
    classifier visual;

    cv::String path1 = "./test_imgs/forw";
    std::vector<cv::String> img_forw;
    cv::glob(path1, img_forw);
    cv::Mat img = cv::imread(img_forw[0]);
    double area;
    std::vector<cv::Mat> img_mser = visual.MSER_Features(img, area);

    return img_mser;
}

/**
 *@brief Function to test if MSER features are not detected when not needed 
 *@param none
 *@return Empty vector of images
 */
std::vector<cv::Mat> test_MSERBad() {
    classifier visual;

    // Read images from testing images directory
    cv::String path1 = "./test_imgs/bad";
    std::vector<cv::String> img_bad;
    cv::glob(path1, img_bad);
    cv::Mat img = cv::imread(img_bad[0]);
    double area;
    std::vector<cv::Mat> img_mser = visual.MSER_Features(img, area);

    return img_mser;
}

/**
 *@brief Function to test if HOG features detected correctly
 *@param none
 *@return Matrix of 81x1 with the HOG descriptor
 */
cv::Mat test_HOG() {
    classifier visual;

    cv::HOGDescriptor hog(cv::Size(64, 64),
                      cv::Size(32, 32),
                      cv::Size(16, 16),
                      cv::Size(32, 32),
                      9, 1, -1, 0, 0.2,
                      1, 64, 1);

    std::vector<cv::Mat> imgs;

    // Read images from testing images directory
    cv::String path1 = "./test_imgs/forw";
    std::vector<cv::String> img_forw;
    cv::glob(path1, img_forw);
    cv::Mat img1 = cv::imread(img_forw[0]);
    cv::resize(img1, img1, cv::Size(64, 64));
    imgs.push_back(img1);

    // Read images from testing images directory
    cv::String path2 = "./test_imgs/turn";
    std::vector<cv::String> img_turn;
    cv::glob(path2, img_turn);
    cv::Mat img2 = cv::imread(img_turn[0]);
    cv::resize(img2, img2, cv::Size(64, 64));
    imgs.push_back(img2);

    cv::Mat HOG = visual.HOG_Features(hog, imgs);

    return HOG;
}

/**
 *@brief Function to test visualization conditions work
 *@param type is the type of sign detected
 *@return int indicating type of sign plotted
 */
int test_viz(int type) {
    classifier visual;

    // Read images from testing images directory
    cv::String path1 = "./test_imgs/forw";
    std::vector<cv::String> img_forw;
    cv::glob(path1, img_forw);
    visual.imagen = cv::imread(img_forw[0]);
    visual.traffic_sign = type;
    cv::Rect rect(1, 1, 10, 10);
    std::vector<cv::Rect> rects;
    rects.push_back(rect);
    visual.boxes = rects;

    int uno = visual.visualization();

    return uno;
}


/**
 *@brief Test if stop sign is being classified correctly
 *@param none
 *@return none
 */
TEST(TestVision, stopTest) {
    float type = testing_stop();

    EXPECT_EQ(3, type);
}

/**
 *@brief Test if turn sign is being classified correctly
 *@param none
 *@return none
 */
TEST(TestVision, turnTest) {
    float type = testing_turn();

    EXPECT_EQ(2, type);
}

/**
 *@brief Test if forward sign is being classified correctly
 *@param none
 *@return none
 */
TEST(TestVision, forwardTest) {
    float type = testing_forward();

    EXPECT_EQ(1, type);
}

/**
 *@brief Test if SVM training correctly
 *@param none
 *@return none
 */
TEST(TestVision, trainingTest) {
    int trained = test_training();

    EXPECT_EQ(1, trained);
}

/**
 *@brief Tests to see if MSER features are being detected and not detected properly
 *@param none
 *@return none
 */
TEST(TestVision, MSERTests) {
    // Good test
    std::vector<cv::Mat> mser = test_MSERGood();
    EXPECT_NE(0, mser.size());

    // Bad test
    std::vector<cv::Mat> mser2 = test_MSERBad();
    EXPECT_EQ(0, mser2.size());
}

/**
 *@brief Test to see that HOG features are being processed properly.
 *@param none
 *@return none
 */
TEST(TestVision, HOGTest) {
    cv::Mat image = test_HOG();

    EXPECT_EQ(81, image.cols);
}

/**
 *@brief Test to check if visualization is acting correctly
 *@param none
 *@return none
 */
TEST(TestVision, VizTest) {
    int uno = test_viz(1);
    EXPECT_EQ(1, uno);

    int dos = test_viz(2);
    EXPECT_EQ(1, dos);

    int tres = test_viz(3);
    EXPECT_EQ(1, tres);
}

/**
 *@brief Main function that runs all tests in the file
 *@param argc is the number of arguments
 *@param argv is the arguments characters array
 *@return Tests results
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "test_vision");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
