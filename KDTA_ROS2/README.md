# KDTA ROS2
## [OPENCV]

- 공유 슬라이드 [[링크](https://docs.google.com/presentation/d/1453nx14DVMk0nBLW7jpt0g6x7a7z2wuNaJKmcVQi4rw/edit?usp=sharing)]

- ubuntu 설치 후 OpenCV 설치

```bash
sudo apt update 
sudo apt upgrade
sudo apt install libopencv-dev python3-opencv
sudo apt install build-essential cmake git pkg-config
```

- OpenCV 설치 후 추가 작업
  - 각 패키지에 대한 추가 설명
  - libjpeg-dev, libtiff5-dev, libpng-dev: 이미지 파일 포맷을 읽고 쓰기 위한 라이브러리
  - libavcodec-dev, libavformat-dev, libswscale-dev, libv4l-dev: 비디오 파일 포맷을 읽고 쓰기 위한 라이브러리
  - libxvidcore-dev, libx264-dev: 비디오 코덱 라이브러리
  - libgtk-3-dev: GUI 툴킷 라이브러리
  - libatlas-base-dev, gfortran: 선형대수 라이브러리
  - python3-dev: 파이썬 개발 헤더 파일
  - python3-pip: 파이썬 패키지 설치 도구
  - numpy: 수치 계산 라이브러리
  - libopencv-dev: OpenCV 라이브러리
  - python3-opencv: OpenCV 파이썬 바인딩 라이브러리
  - build-essential: 빌드 도구
  - cmake: 빌드 자동화 도구
  - git: 소스 코드 버전 관리 도구
  - pkg-config: 라이브러리 정보 검색 도구

```bash
sudo apt-get install libjpeg-dev libtiff5-dev libpng-dev
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt-get install libxvidcore-dev libx264-dev
sudo apt-get install libgtk-3-dev
sudo apt-get install libatlas-base-dev gfortran
sudo apt-get install python3-dev
sudo apt-get install python3-pip
pip3 install numpy
```

---

### 2024-07-23

---

- 1 차시
  - VMware worksation 17 설치
  - ubuntu 22.04 설치
  - terminator 설치 (sudo apt install terminator)
  - git 설치 (sudo apt install git)
- 2 차시
  - vscode 설치 (sudo apt install .code~)
  - github 연동 (git clone [github URL])
  - opencv 환경 설정 (sudo apt install libopencv-dev python3-opencv, sudo apt update, sudo apt upgrade)
- 3 차시
  - cmake, make 설치 및 컴파일 용도 사용 (sudo apt cmake, sudo apt make)
  - CMakeLists.txt 및 c_cpp_properties.json 작성 (환경설정 관련)
  - imread 및 imshow, namedWindow 등 활용하여 간단한 출력화면 구성(first / helloworld.cpp, ch02 / windows.cpp & .. / movewindows.cpp)
  - 주요 클래스 Point, Size, Rect, Scalar 멤버 객체 활용 및 출력 예제 화면 크기 변경 등

---

### 2024-07-24

---

- 주요 클래스 마무리
  - Mat 클래스
- matrix 연산자 사용 예시 구현
  - Fixed Pixel Type: matrix 클래스의 element가 어떤 타입의 데이터인지 지정.
    - CV_8UC1 : 8 bit unsigned integer : uchar(0..255)
      - 8 : bit 단위로서 8bit를 의미
      - U : unsigend의 약자, ( U : Unsinged, S : Signed, F : Floating )
      - C1 : Channel의 약자이며 1채널을 의미한다. ( 1채널의 경우 C1 을 생략 가능하여 CV_8UC1 = CV_8U 와 같은 값을 지니게 된다. )
    - CV_8SC1 : 8 - bit signed integer : schar(-128..127)
    - CV_16UC1 : 16 - bit unsigned integer : ushort(0..65535)
    - CV_16SC1 : 16 - bit signed integer : short(-32768..32767)
    - CV_32SC1 : 32 - bit signed integer : int(-2147483648..2147483647)
    - CV_32FC1 : 32 - bit floating - point number : float(-FLT_MAX..FLT_MAX, INF, NAN)
    - CV_64FC1 : 64 - bit floating - point number : double(-DBL_MAX..DBL_MAX, INF, NAN)
  - Scalar 멤버함수 속성 활용(색상 출력)
  - matrix 연산자 멤버 객체(Clone, setTo)를 활용한 복제 [[matOp2.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch03/matOp2.cpp)]
  - ROI(Region of Interest: 이미지 내 관심영역)을 활용한 이미지 내 특정 영역 추출 및 출력 / Rect 클래스 변수 활용(특정 초기 입력 좌표값을 통한 관심영역 지정)
    [[matOp3.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch03/matOp3.cpp)]
  - 행렬 조작 기능 학습, 다양한 방법으로 행렬 접근 및 조작 - matrix 클래스(cv::Mat) 정적 멤버 함수 zeros(지정된 크기와 데이터 타입으로 모든 요소를 0으로 초기화하는 역할)사용, 이중 반복문 사용하여 각 픽셀을 반복하면서 해당 픽셀의 값을 1씩 증가('.at<Template>' method를 사용하여 요소에 접근, point 및 iteraotr 각각 활용)
    [[matOp4.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch03/matOp4.cpp)]
  - 외부 저장 공간에 데이터 저장 및 출력 [[matOp5.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch03/matOp5.cpp)]
  - 벡터(Vec3b 사용) 및 스칼라(Scalar) 행렬 객체를 활용한 색상 출력 [[vecOp.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch03/vecOp.cpp)]


---
#### chapter 04

- VideoCapture / VideoWriter 클래스
  - 비디오 캡쳐 및 파일 저장
  - 디바이스의 카메라를 통한 비디오 영상 출력 / videoWriter 클래스 객체를 활용하여 동영상 파일 생성 [[video.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch04/video.cpp)]  
    (실습 노트북의 내장 카메라 접근 이슈로 인해 저장된 동영상을 출력)
  - line 그리기 [[line.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch04/line.cpp)]
  - rectangle [[rectangleTest.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch04/rectangleTest.cpp)]
  - circle, ellipse, polylines, fillPoly
  - putText [[drawText.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch04/drawText.cpp)]
  - keyboard 이벤트 처리
    [[keyboard.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch04/keyboard.cpp)]
    [[keyboard2.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch04/keyboard2.cpp)]

---

### 2024-07-25

---

- 1 차시
  - mouse 기능 동작에 따른 관련 이벤트 표현(callback 함수 작성) [[mouseTest.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch04/mouseTest.cpp)]
- 2 차시  
  - trackBar 조작 기능 구현 [[trackBarTest.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch04/trackBarTest.cpp)] / [[trackBarTest2.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch04/trackBarTest2.cpp)]
  - 이미지 합성 [[maskSetTo.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch04/maskSetTo.cpp)]
    - setTo함수로 합성될 이미지에 대해 색상 변경
    - copyTo함수로 합성
    - Masking 되어있는 이미지에 대해 읽어올때(imread()) 속성값으로 인자에 IMREAD_GRAYSCALE를 기재
- 3 차시
  - 파일스토리지 사용법
    - file 저장 [[fileStorageTest.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch04/filestorageTest.cpp)]
    - FileStorage 클래스 객체의 open함수로 저장될 위치 및 FileStorage::WRITE 속성을 통해 저장에 대해 명시
    - 저장된 파일 내용 불러오기 [[fileStorageReadTest.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch04/fileStorageReadTest.cpp)]
    - FileStorage::READ로 저장된 파일을 불러옴에 대해 명시
  - 마스크 연산
- 4 차시
  - chapter 마무리 종합 과제 수행 [[mouseRGBPoint_myself.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch04/mouseRGBPoint_myself.cpp)] - 마우스 따라 다니는 사각형
    1. TickMeter 객체 사용: TickMeter를 이용하여 tm1과 tm2를 선언하고 시작(tm1.start(), tm2.start()) 및 종료(tm1.stop(), tm2.stop()) 시간을 측정합니다. 이는 FPS 측정과 최적화를 위한 핵심 요소입니다.
    2. 이미지 처리 및 출력: imshow() 함수를 이용하여 img_display 이미지를 출력하고, ESC 키 입력 시 프로그램을 종료합니다. 또한, 현재 마우스 위치에 따라 사각형을 그립니다.
    3. FPS 관리 및 대기 시간 계산: tm1.getFPS()를 이용하여 현재 FPS를 계산하고, 목표 FPS보다 높으면 추가적인 슬립을 통해 CPU 소비를 조정합니다.
    4. 마우스 이벤트 처리: onMouse() 함수를 이용하여 마우스 이벤트를 처리하고, 왼쪽 마우스 클릭 시 클릭한 픽셀의 RGB 값을 출력하고 rgbValues 벡터에 저장합니다.
    5. 파일 저장: 마지막으로, ESC 키 입력 시 rgbValues에 저장된 RGB 값을 JSON 파일로 저장합니다.
  
  - 강사님 구현 코드 [[mouseRGBPoint.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch04/mouseRGBPoint.cpp)]

#### chapter 05
- 명암비 조절 [[brightness.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch05/brightness.cpp)]
  - cvtColor 함수를 사용하여 변환.
- 히스토그램 분석 [[histogram.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch05/histogram.cpp)]
  - 영상의 밝기 또는 색상 분포를 시각적으로 나타내는 도구.
    히스토그램은 영상 내 각 픽셀 값의 빈도를 나타내는 그래프이며, 픽셀 값의 범위를 구간으로 나누어 각 구간에 속하는 픽셀의 수를 히스토그램의 세로축 값으로 표시함.
  - calcHist() 함수를 사용하여 영상의 히스토그램을 계산.
- 히스토그램 평활화(histogram equalization) [[histogramEqualization.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch05/histogramEqualization.cpp)]
  - 영상의 대비를 개선하기 위해 사용, 영상의 히스토그램을 조정하여 전체적인 밝기 분포를 균일하게 함.

#### chapter 06
- 이미지 / 영상 논리 연산 [[logical.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch06/logical.cpp)]
  - bitwise 논리식 함수를 활용하여 연산 결과물 출력

#### chapter 07
- 필터링
  - 커널(윈도우): Mat 사용. 작은 크기. 홀수만 사용.  
    Convolution(회선)

---

### 2024-07-26

---

#### chapter 08 (영상의 기하학적 변환)
- 1차시
  - affine transformation [[affine.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch08/affine.cpp)]
- 2차시
  - warpaffine
  - affineRotation [[affineRotation.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch08/affineRotation.cpp)] - 트랙바 연결 예제 작성
- 3차시
  - 투시변환 [[perspectiveTransform.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch08/perspectiveTransform.cpp)]
    - 카드 예제에서 거리 및 각도 측정 코드 추가 [[perspectiveCard.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch08/perspectiveCard.cpp)]
	
#### chapter 09
- edge
  - 캐니 엣지 검출 [[canny.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch09/canny.cpp)]
- 허프 변환 [[houghline.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch09/houghline.cpp)] / [[houghlineP.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch09/houghlineP.cpp)]

#### chapter 10
- Color 영상 처리
  - 색 공간 변환
    - color [[color.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch10/color.cpp)]
	- inRange [[inRange.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch10/inRange.cpp)]
	- split & merge [[splitmerge.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch10/splitmerge.cpp)]

#### chapter 11
- 영상의 이진화(Binarization)
  - threshold 함수 [[threshold.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch11/threshold.cpp)]
    - src: 입력 이미지. 이 이미지의 각 픽셀에 대해 임계값 처리가 수행됩니다.
    - dst: 출력 이미지. 임계값 처리된 결과가 저장됩니다.
    - thresh: 임계값. 이 값보다 큰 픽셀은 maxval로 설정되고, 그렇지 않은 픽셀은 0으로 설정됩니다.
    - maxval: 임계값을 초과하는 픽셀에 할당된 값입니다.
    - type: 임계값 처리 유형을 지정하는 정수입니다. OpenCV는 다양한 임계값 처리 유형을 제공합니다. 예를 들어, THRESH_BINARY는 기본적인 이진 임계값 처리를, THRESH_BINARY_INV는 이진 임계값 처리의 반전 버전을 나타냅니다.
- 적응형 이진화
  - adaptiveThreshold 함수[[adaptiveThreshold.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch11/adaptiveThreshold.cpp)]
    - src: 입력 이미지. 이 이미지의 각 픽셀에 대해 적응형 임계값 처리가 수행됩니다.
	- dst: 출력 이미지. 적응형 임계값 처리된 결과가 저장됩니다.
	- maxValue: 임계값을 초과하는 픽셀에 할당된 값입니다.
	- adaptiveMethod: 적응형 임계값 결정 방법을 지정하는 정수입니다. OpenCV는 ADAPTIVE_THRESH_MEAN_C와 ADAPTIVE_THRESH_GAUSSIAN_C 두 가지 방법을 제공합니다.
	- thresholdType: 임계값 처리 유형을 지정하는 정수입니다. OpenCV는 THRESH_BINARY와 THRESH_BINARY_INV 두 가지 유형을 제공합니다.
	- blockSize: 임계값을 계산하는 데 사용되는 이웃 영역의 크기입니다. 이 값은 홀수여야 합니다.
	- C: 계산된 평균 또는 가중 평균에서 뺄 상수입니다. 일반적으로 이 값은 양수지만, 0 또는 음수일 수도 있습니다.
- 이진 영상의 침식과 팽창
  - 모폴로지 (Morphology) - [[morphologyEx.cpp](https://github.com/top-to-toe/KDTA_ROS2/blob/main/opencv/ch11/morphologyEx.cpp)]
    - 영상을 형태학적인 측면으로 접근하는 것
    - 구조 요소(structuring element)는 모폴로지 연산의 결과를 결정하는 커널, 마스크, 윈도우
	- src: 입력 이미지. 이 이미지에 대해 팽창 연산이 수행됩니다.
	- dst: 출력 이미지. 팽창 연산된 결과가 저장됩니다.
	- kernel: 팽창 연산에 사용되는 구조 요소입니다. 이 요소는 팽창의 정도와 모양을 결정합니다.
	- anchor: 구조 요소의 앵커 포인트입니다. 기본값은 구조 요소의 중심입니다.
	- iterations: 팽창 연산이 적용되는 횟수입니다. 이 값이 클수록 이미지는 더욱 팽창됩니다.
	- borderType: 픽셀 외삽법을 지정하는 정수입니다. OpenCV는 다양한 외삽법을 제공합니다.
	- borderValue: 상수 경계의 경우의 경계 값입니다.
	
#### chapter 13
- cascade
- HogDescriptor

---

## 2024-07-30

---

## [ROS2]

- ROS2 개념
  - 노드
  - 토픽
  - 메시지
  - 런처
  - 노드 통신
  - 노드 실행
- ROS2 설치
  - ROS2 humble 설치
  - ros2 humble documentation debian installation
  - 환경 설정 파일
    - .bashrc
      - source /opt/ros/humble/setup.bash
      - source ~/xxx_Workspace/install/local_setup.bash
      - 여러가지 alias 설정
      - export ROS_DOMAIN_ID 설정
- 패키지 작성
  - kdta_ROS2_workspace
    - simple_pkg_cpp
      - xml 수정
      - CMakeLists.txt 수정
      - 노드 추가
        - helloword.cpp : 기본 main 코드
        - helloOpenCV.cpp : OpenCV 사용 코드(외부 라이브러리 적용)
        - hellow_publisher : Node 및 spin timer 적용
        - hellow_publisher_class : Node 상속 클래스 적용

---

## 2024-07-31

---

- 패키지 작성 (이어서)
  - kdta_ROS2_workspace
    - simple_pkg_cpp 패키지
      - 노드 추가
        - hellow_publisher_class2 : 분할컴파일 (해더, 메인, CPP)
        - hellow_subscriber : subscriber 노드
        - time_publisher_class : timer 노드
        - time_subscriber : timer subscriber 노드
    - move_t_sim 패키지
      - 노드 추가
        - moveTurtleSim.cpp : 터틀심 이동 노드( turtle1, turtle2 동시 이동)
- 최종 실습(Topic) [[ROS2/src/msg](https://github.com/top-to-toe/KDTA_ROS2/tree/main/ROS2/src/msg)] - 과제 구현 완료
  - 5개 노드 3개 Topic 사용
    - 노드 : mpub, msub, msub2, tpub, mtsub
    - Topic :
      - /message1: String ( mpub -> msub, mtsub )
      - /message2: String ( mpub -> msub2 )
      - /time: Header ( tpub -> mtsub )
  - 강사님 구현코드 [[topic_final](https://github.com/freshmea/kdta_ROS2/tree/main/topic_final)]

---

## 2024-08-01

---

- 터틀봇3 네트워크 설정
  - 공유기 설치
    - ssid: turtle
    - password: turtlebot3
  - 가상환경 bridge 설정
    - 네트워크 브릿지 클리 후 네트워크 카드 하나만 사용
- 터틀봇3 개요
  - 터틀봇3 소개
  - 터틀봇3 구성
  - 터틀봇3 시스템 구성
- 터틀봇3 ssh 연결
  - 터미널에서 연결
    - `ssh ubuntu@192.169.14.XX`
      - userid: ubuntu
      - password: aa
    - vscode remote ssh 연결
      - vscode remote ssh 설정
- 터틀봇3 브링업
  - 터틀봇3 브링업
    - ROS_DOMAIN_ID 확인
    - `export TURTLEBOT3_MODEL=burger`
    - 브링업 런치 실행 `ros2 launch turtlebot3_bringup robot.launch.py`
- 텔레옵키 노드 실행(노트북)
  - 텔레옵키 노드 실행
    - `ros2 run turtlebot3_teleop teleop_keyboard`
- gazebo
  - gazebo 설치
    - `sudo apt install ros-humble-gazebo-*`
  - turtlebot3 simulation 설치(git clone)
    - `cd ~/kdta_ROS2_workspace/src`
    - `git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git`
    - `cd ~/turtlebot3_ws && colcon build --symlink-install`
  - turtlebot3 simulation 실행
    - `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`
- 터틀봇3에서 빌드 후 실행
  - move_t_sim 이동
    - vscode 로컬과 원격을 동시 실행 후 이동
    - 빌드 후 실행(터틀봇3)
      - `cd ~/robot_ws`
      - `colcon build --symlink-install`
      - `ros2 run move_t_sim moveTurtleBot`
- 터틀심 사각형 움직임 구현
  - moveTurtleSim2.cpp 작성
  - 빌드 후 실행(터틀봇3)
    - `ros2 run move_t_sim moveTurtleSim2`

---

## 2024-08-02

---

- 터틀심 사각형 움직임 구현 -> 터틀봇 용 코드로 변경
  - moveTurtleBot2.cpp 작성
  - 세부 알고리즘 변경
- 사용자 인터페이스 작성
  - srv/AddTwoInt.srv 파일 작성
- Service 노드 추가
  - Service server 노드
  - Service client 노드1
    - spin_until_done 사용
  - Service client 노드2
    - spin 사용 후 callback 사용
- 파라미터 
  - 파라미터 기초
    - ros2 param get /node_name param_name
    - ros2 param set /node_name param_name param_value
    - ros2 param list /node_name
    - ros2 param dump /node_name
    - ros2 param load /node_name
- Launch 파일 작성
  - msg에 추가

---

## 2024-08-05

---

- 파라미터 코드 적용
  - param_events topic 활용.
  - 파라미터 변경시 이벤트 발생
  - set_parameters_callback 함수 사용 <- 적용 안됨
- action 코드 작성
  - interface_example 에서 fibonacci action 추가
  - simple_pkg_cpp 에서 action server, client 추가
- 토픽, 서비스, 액션 통합 예제 작성
  - arithmetic 패키지 작성
  - interface_example 에 토픽, 서비스 액션 추가
    - ArithmeticArgument.msg
    - ArithmeticOperator.srv
    - ArithmeticChecker.action
  - 토픽 노드 작성
    - argument.cpp

---

## 2024-08-06

---

- Arithmetic 패키지 작성
  - 분할 컴파일
    - argument.cpp -> argument.cpp, argument.hpp
    - 폴더 나누기
  - 서비스 노드 작성
    - operator.cpp
  - `실습` argument.cpp 기능 추가하기
    - 램덤하게 operator 발행 하기
    - 파라미터 설정해서 램덤 혹은 스테틱 설정하기
    - 파라미터를 yaml 로 저장해서 launch 파일로 불러오기