#include "opencv2/opencv.hpp"
#include <iostream>

using namespace cv;
using namespace std;
String folder = "/home/hansollee/Documents/KDTA_ROS2/opencv/data/";

int main() {
    Mat frame, doubleFrame, reshapeFrame;
    VideoCapture cap(folder + "vtest.avi");
    // VideoCapture cap(0, CAP_V4L2);               // 0값을 주면 기본적으로 연결된 디바이스의 카메라에 접근
    double fps = cap.get(CAP_PROP_FPS);
    int delay = cvRound(1000/fps);
    Size sz1(cap.get(CAP_PROP_FRAME_WIDTH), cap.get(CAP_PROP_FRAME_HEIGHT));
    std::vector<int> shape = {sz1.height * 2, sz1.width / 2};
    
    // open check 
    if(!cap.isOpened()) {
        cerr << "Video open failed" << endl;
        return -1;
    }
    cout << "Video open" << endl;
    
    int fourcc = VideoWriter::fourcc('D', 'I', 'V', 'X');
    VideoWriter outputVideo(folder + "output10.avi", fourcc, fps, sz1);
    
    // frame image show
    while(true) {
        cap >> frame;
        if(frame.empty())               // 빈 프레임이 있는 경우 break.
            break;
        resize(frame, doubleFrame, sz1 * 2);
        reshapeFrame = frame.reshape(3, shape);        // reshape

        imshow("frame", frame);
        imshow("doubleframe", doubleFrame);
        imshow("reshape", reshapeFrame);

        outputVideo << frame;

        if(waitKey(delay) == 27)                       // 27 --> ESC key
            break;
    }
    cap.release();
    outputVideo.release();
    return 0;
}

// VMware 상단 메뉴 Player - Removable Devices - 하위 메뉴중 카메라와 관련된 디바이스 항목 선택(Connect)