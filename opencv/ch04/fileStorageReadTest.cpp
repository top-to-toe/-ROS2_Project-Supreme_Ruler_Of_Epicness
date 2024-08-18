#include "opencv2/opencv.hpp"
#include <iostream>

using namespace cv;
using namespace std;
String folder = "/home/hansollee/Documents/KDTA_ROS2/opencv/data/";

int main() {
    FileStorage fs;
    String name;
    int age;
    Point pt1;
    vector<float> v;
    Mat img;

    fs.open(folder + "myData.json", FileStorage::READ);
    if(!fs.isOpened()) {
        cerr << "file open failed" << endl;
        return 1;
    }
    // 읽어오는 json 확장자 형태 파일의 내용 형식에 맞게 각 데이터 항목에 맞추어 변수 매칭 및 접근을 위한 정의
    fs["name"] >> name;
    fs["age"] >> age;
    fs["point"] >> pt1;
    fs["vector"] >> v;
    fs["mat1"] >> img;

    fs.release();

    cout << name << endl;
    cout << age << endl;
    cout << pt1 << endl;
    for(auto& val: v) {
        cout << val << " ";
    }
    cout << endl;
    imshow("img", img);
    waitKey(0);

    return 0;
}