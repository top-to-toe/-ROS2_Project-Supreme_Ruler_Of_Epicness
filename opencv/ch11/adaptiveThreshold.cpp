#include "opencv2/opencv.hpp"
#include <iostream>

using namespace cv;
using namespace std;
String folder = "/home/hansollee/Documents/KDTA_ROS2/opencv/data/";

int main() {
    // Mat img = imread(folder + "neutrophils.png", IMREAD_GRAYSCALE);
    Mat img = imread(folder + "sudoku.jpg", IMREAD_GRAYSCALE);
    Mat dst, dst2;
    threshold(img, dst2, 80, 255, THRESH_BINARY);
    adaptiveThreshold(img, dst, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 33, 5);     // block 사이즈도 커널의 개념이기때문에 홀수값이여야 한다.

    imshow("img", img);
    imshow("dst", dst);
    imshow("dst2", dst2);
    waitKey(0);
    
    return 0;
}