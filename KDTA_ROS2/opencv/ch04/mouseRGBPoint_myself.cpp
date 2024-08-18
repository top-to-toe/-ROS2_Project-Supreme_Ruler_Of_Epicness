/* 과제 내용 */
// 1. lenna 이미지 window 출력
// 2. 마우스 따라다니는 사각형(size: [50, 50])
// 3. 마우스 클릭시 마우스 위치의 RGB 값을 cout -> 터미널 화면에 출력
// 4. RGB vector에 넣고 FileStorage 저장 "mouseRGBvector.json"
// 5. 최적화 tickmeter

// OpenCV 라이브러리 헤더 파일을 포함
#include "opencv2/opencv.hpp"

// C++ 표준 라이브러리 헤더 파일 포함
#include <iostream>
#include <chrono>  // 시간 측정을 위한 헤더
#include <thread>  // 스레드 관리를 위한 헤더

// OpenCV와 C++ 네임스페이스 사용 설정
using namespace cv;
using namespace std;

// 이미지 파일이 저장된 폴더 경로
String folder = "/home/hansollee/Documents/KDTA_ROS2/opencv/data/";

// 사각형의 초기 위치 및 크기
int rect_x = 100;
int rect_y = 100;
int rect_width = 50;
int rect_height = 50;

// 마우스 이벤트 핸들러 함수 정의
void onMouse(int event, int x, int y, int flags, void *data);

// 사용할 데이터 구조체 정의
struct Mydata {
    Point ptOld;                            // 이전 마우스 위치
    Point ptNew;                            // 현재 마우스 위치
    Mat img;                                // 이미지 데이터
    Scalar black = Scalar(0, 0, 0);         // 색상 정의
    Scalar white = Scalar(255, 255, 255);
    Scalar yellow = Scalar(0, 255, 255);
    Scalar blue = Scalar(255, 0, 0);
    Scalar green = Scalar(0, 255, 0);
    Scalar red = Scalar(0, 0, 255);
    vector<Vec3b> rgbValues;                // 클릭한 픽셀의 RGB 값 저장
    bool flag = false;                      // 마우스 버튼 눌림 상태
};

int main() {
    // 데이터 구조체 인스턴스 생성
    Mydata mydata;
    // 이미지 파일 읽기
    mydata.img = imread(folder + "lenna.bmp");

    // 이미지 파일 읽기 실패 시 처리
    if (mydata.img.empty()) {
        cerr << "Error: Image not found!" << endl;
        return -1;
    }

    // 윈도우 생성 및 마우스 이벤트 콜백 함수 설정
    namedWindow("img");
    setMouseCallback("img", onMouse, (void *)&mydata);
    float fps = 30.0;  // 목표 FPS 설정
    int delay = cvRound(1000 / fps);  // 프레임 간 대기 시간 계산

    // FPS 측정을 위한 TickMeter 객체 생성
    TickMeter tm1;
    TickMeter tm2;

    // 메인 루프
    while (true) {
        tm1.start();  // 주요 작업 시간 측정 시작
        tm2.start();  // 부가적인 시간 측정 시작

        // 이미지 복사본 생성
        Mat img_display = mydata.img.clone();

        // 현재 사각형 그리기
        rectangle(img_display, Point(rect_x, rect_y), Point(rect_x + rect_width, rect_y + rect_height), mydata.black, 2);

        // 이미지 출력
        imshow("img", img_display);

        // ESC 키 입력 시 종료
        if (waitKey(10) == 27) {
            // JSON 파일로 RGB 값 저장
            FileStorage fs(folder + "mouseRGBvector.json", FileStorage::WRITE);
            fs << "RGB_Values" << "[";  // JSON 배열 시작
            for (const auto& rgb : mydata.rgbValues) {
                fs << "{" << "R" << (int)rgb[2] << "G" << (int)rgb[1] << "B" << (int)rgb[0] << "}";  // 각 RGB 값 저장
            }
            fs << "]";  // JSON 배열 종료
            fs.release();  // 파일 저장 종료

            break;  // 메인 루프 종료
        }

        tm1.stop();  // 주요 작업 시간 측정 종료
        cout << "tm1.getFPS(): " << tm1.getFPS() << endl;  // 주요 작업 FPS 출력
        // 목표 FPS보다 높으면 대기 시간을 계산하여 CPU 소비를 줄임
        if (tm1.getFPS() > fps) {
            auto sleep_duration = static_cast<int>(1000.0 * (tm1.getFPS() - fps) / tm1.getFPS() / fps);
            this_thread::sleep_for(chrono::milliseconds(sleep_duration));  // 계산된 시간만큼 스레드 대기
        }

        tm2.stop();  // 부가적인 시간 측정 종료
        cout << "tm2.getFPS(): " << tm2.getFPS() << endl;  // 부가적인 시간 FPS 출력
    }

    return 0;  // 프로그램 종료
}

// 마우스 이벤트 핸들러 함수 구현
void onMouse(int event, int x, int y, int flags, void *data) {
    // 데이터 구조체 포인터로 변환
    Mydata *ptr = (Mydata *)data;
    
    switch (event) {
        case EVENT_MOUSEMOVE:
            // 마우스 이동 시 사각형 위치 업데이트
            rect_x = x;
            rect_y = y;
            break;
        case EVENT_LBUTTONDOWN:
            cout << "Mouse left button down" << endl;
            // 왼쪽 마우스 버튼 클릭 시 처리
            if (x >= 0 && x < ptr->img.cols && y >= 0 && y < ptr->img.rows) {
                // 클릭한 픽셀의 RGB 값 추출
                Vec3b intensity = ptr->img.at<Vec3b>(y, x);
                cout << "RGB values at (x=" << x << ", y=" << y << "): "
                     << "R=" << (int)intensity[2] << ", "
                     << "G=" << (int)intensity[1] << ", "
                     << "B=" << (int)intensity[0] << endl;

                // RGB 값을 벡터에 저장
                ptr->rgbValues.push_back(intensity);
            }
            ptr->flag = true;  // 마우스 버튼 눌림 상태 설정
            break;
        case EVENT_LBUTTONUP:
            cout << "Mouse left button up" << endl;
            ptr->flag = false;  // 마우스 버튼 눌림 상태 해제
            break;
    }
}

/*
    1. 마우스 이벤트 핸들러 함수 (onMouse):
        onMouse 함수는 마우스 동작을 감지하고 처리하는 역할을 합니다.
        event: 발생한 마우스 이벤트를 나타내는 변수입니다.
        x, y: 마우스 클릭 또는 이동 시의 좌표입니다.
        flags: 마우스 이벤트에 대한 추가 정보입니다.
        data: 사용자 데이터(여기서는 Mydata 구조체)에 대한 포인터입니다.

    2.마우스 이벤트 처리:
        EVENT_MOUSEMOVE: 마우스가 움직일 때 사각형의 위치를 마우스 위치로 업데이트합니다.
        EVENT_LBUTTONDOWN: 왼쪽 마우스 버튼을 클릭했을 때 처리합니다.
            픽셀 좌표가 유효한 범위 내에 있으면 해당 좌표의 RGB 값을 추출하여 출력합니다.
            추출한 RGB 값을 rgbValues 벡터에 저장합니다.
            flag를 true로 설정하여 마우스 버튼이 눌린 상태임을 나타냅니다.
        EVENT_LBUTTONUP: 왼쪽 마우스 버튼을 뗐을 때 처리합니다.
            flag를 false로 설정하여 마우스 버튼이 눌리지 않은 상태임을 나타냅니다.
*/

/*
    tm1와 tm2는 TickMeter 객체를 사용하여 프레임 속도(FPS)를 측정하고, 일정 FPS를 유지하기 위한 최적화 기능을 구현하기 위해 추가된 것.
    1. FPS 측정:
        tm1은 이미지 출력과 관련된 주요 작업 시간을 측정합니다. tm1.start()와 tm1.stop() 사이의 경과 시간을 기반으로 FPS를 계산합니다. 이를 통해 프로그램이 목표 프레임 속도에 도달하는지를 모니터링할 수 있습니다.
    2. FPS 조절을 위한 대기 시간 계산:
        프로그램은 목표 FPS와 실제 FPS를 비교하여, 만약 실제 FPS가 목표 FPS보다 높다면, 추가적인 대기 시간을 계산하여 CPU 소비를 줄이고 시스템 자원을 효율적으로 관리하려고 합니다. 이를 위해 tm1.getFPS()를 사용하여 현재 FPS를 가져오고, 이 값을 기반으로 대기 시간을 계산하여 스레드를 통해 적용합니다.
    3. tm2의 역할:
        tm2는 보조적인 시간 측정을 담당합니다. 주로 추가적인 디버깅이나 성능 모니터링 목적으로 사용될 수 있습니다. 코드 실행 중에 추가적인 시간 측정이 필요할 때 유용하게 사용될 수 있습니다.
*/