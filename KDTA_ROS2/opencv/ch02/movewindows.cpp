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
    while(waitKey(0) != -1) {
        // int key = waitKey(0);
        cout << "aaa" << endl;
        moveWindow("lenna", 50, 0);
    }
    return 0;
}