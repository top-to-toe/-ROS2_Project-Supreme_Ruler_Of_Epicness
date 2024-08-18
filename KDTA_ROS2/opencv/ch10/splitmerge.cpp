#include "opencv2/opencv.hpp"
#include <iostream>

using namespace cv;
using namespace std;
String folder = "/home/hansollee/Documents/KDTA_ROS2/opencv/data/";

int main() {
    Mat img = imread(folder + "candies.png");

    vector<Mat> bgr, blue_planes, green_planes, red_planes;
    split(img, bgr);

    Mat empty(img.rows, img.cols, CV_8UC1, Scalar(0));
    blue_planes = {bgr[0], empty, empty};
    green_planes = {empty, bgr[1], empty};
    red_planes = {empty, empty, bgr[2]};

    Mat bDst, gDst, rDst;
    merge(blue_planes, bDst);
    merge(green_planes, gDst);
    merge(red_planes, rDst);

    imshow("img", img);
    imshow("blue_img", bDst);
    imshow("green_img", gDst);
    imshow("red_img", rDst);
    waitKey(0);
    
    return 0;
}