#include "opencv2/opencv.hpp"
#include <iostream>

using namespace cv;
using namespace std;
String folder = "/home/hansollee/Documents/KDTA_ROS2/opencv/data/";

int main() {
    Mat src = imread(folder + "rose.bmp", IMREAD_GRAYSCALE);
    // float data[] = {-1, -1, 0, -1, 0, 1, 0, 1, 1};
    float data[] = {1.0/9, 1.0/9, 1.0/9, 1.0/9, 1.0/9, 1.0/9, 1.0/9, 1.0/9, 1.0/9};

    Mat emboss(3, 3, CV_32F, data);                                 // 커널의 갯수는 항상 홀수개로 정해야 된다!
    Mat dst;

    // filter2D(src, dst, -1, emboss, Point(-1, -1), 120);          // delta값 120은 밝기를 + 120만큼 해준다는 의미 
    filter2D(src, dst, -1, emboss, Point(-1, -1), 0);

    imshow("src", src);
    imshow("dst", dst);
    waitKey(0);
    return 0;
}