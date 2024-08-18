#include "opencv2/opencv.hpp"
#include <iostream>

using namespace std;
using namespace cv;

string folder = "/home/hansollee/Documents/KDTA_ROS2/opencv/data/";

int main() {
    Mat img;
    img = imread(folder + "lenna.bmp");
    namedWindow("lenna");
    imshow("lenna", img);
    int key = waitKey(0);
    cout << key << endl;
    // Size size;
    // size.height = 100;
    // size.width = 100;
    destroyWindow("lenna");
    imshow("lenna", img);
    // resizeWindow("lenna", size);
    waitKey(0);

    return 0;
}