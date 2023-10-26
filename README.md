# 자율주행 데브코스 C++ 예제 코드
1. 허프변환과 PID 제어를 사용한 Lane Keeping 알고리즘 사용
2. ROS package 사용을 위해 폴더 명과 프로젝트 명을 Cmakelist.txt, package.xml 파일에서 알맞게 바꾼 후 `catkin_make`로 빌드 후 실행
3. 이슈를 반영한 C++ 코드 수정
4. Xycar와 연결된 디바이스를 사용하기 위해 내장 설치된 Xycar_device 패키지를 같이 가져와 저장함

주행 코스
![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/4257f10e-0d08-4abc-b680-77a1933b21a3/Untitled.png)

---

https://www.notion.so/Team1-a6e51ca53ed44ea7b429b7814c15cca6?pvs=4#8aeeea3c3eae4a00aded142709aa6924


## 차선 검출

- usb_cam 토픽을 구독하여 데이터를 받고 이 데이터로 이미지 전처리를 진행
- 차선 인식을 위해 Hough변환을 사용
- 여러 파라미터들은 Yaml 포맷을 활용해 관리 및 수정했으며 토픽에서 받아온 이미지 파일의 크기, ROI 영역, Canny edge의 Threshold, Hough변환을 위한 변수 등이 이에 해당

1. ROI영역 설정

전체 이미지를 활용하면 많은 연산이 필요하고 불필요한 부분도 포함되기 때문에 ROI영역을 설정
차량의 속도가 빠르면 멀리 보는 것이, 느리면 가까이 보는 것이 좋다는 점을 고려해 맞는 영역을 지정
![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/f30aee94-ba79-4947-bb7f-f32bbf7d8a9f/Untitled.png)

2. GrayScale 변환

카메라의 이미지는 RGB 형태로 표현되며 R, G, B는 각각 0~255의 값을 가지기 때문에 이를 연산하는데는 오랜 시간이 필요
하지만 Grayscale은 0~255의 1차원 값만 사용하기에 연산량이 줄어드는 장점이 있다. 도로의 차선은 흰색, 주황색이 대부분이기에 빠른 연산이 중요한 자율주행에서 필수적인 전처리에 해당
이번 프로젝트에서 사용하는 차선도 검정색이기 때문에 Grayscale로 변환해 처리하는 것이 효율적이라 판단되어 변환

![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/fef9f54e-1ee3-41ff-8052-69f034bff56d/Untitled.png)

3. Canny edge 검출

Canny edge는 Gradient의 변화가 큰 부분을 검출하는 방식이다. Gradient의 변화가 큰 부분은 밝기의 변화가 큰 부분으로 즉, 경계면을 검출한다는 의미를 뜻함
이를 활용하면 차선의 경계를 쉽게 찾아낼 수 있고 여기서 찾은 경계면의 직선들을 활용해 허프 변환에 사용하기 용이하기 때문에 Canny edge 검출 방법을 사용

![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/e7185965-841b-40ce-a779-7873ed2f63cc/Untitled.png)

                     [Threshold 200, 300]                                           [Threshold 50, 150]

4. Hough 변환

3단계를 거쳐 전처리 과정을 거치고 나온 데이터를 통해 허프변환을 수행한다. 허프변환은 직선의 방정식을 활용하여 이미지에서 모양을 찾는 방법
하나의 점을 지나는 직선의 방정식은 y=mx+b로 표현할 수 있다. 이를 삼각함수로 변환하면 r = xcosθ + ysinθ 로 변환
여러 개의 점을 지나는 하나의 직선을 구하는 과정에서 각 점의 θ값을 1~180까지 변화하며 r의 값을 구하고 (θ, r)로 구성된 180개의 2차원 배열이 구해진다. 모든 점에서 동일한 과정을 수행한 뒤 이 점들을 지나는 직선일 확률이 높은 곳을 검출해내는 방식을 사용

![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/28fd9c22-42ac-471d-9f75-860965e9e4ff/Untitled.png)

---


## 차선 유지

- 차선 검출을 통해 인식된 차선을 벗어나지 않게 유지하는 방법이 중요
- 검출된 직선의 시작과 끝에 해당하는 x좌표의 중심들로 좌, 우 차선의 위치를 저장
- 위에서 저장한 값을 더해 2로 나눈 값을 중앙으로 인식하도록 함
- 차량의 중심은 카메라의 가운데 이미지 가로 크기의 중간지점으로 설정

![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/f20c5d3f-97f0-43cc-80d5-a41b1fdcf9c9/Untitled.png)

차선의 중심(노란 원)이 차량의 중심(빨간 원)에서 벗어나면 제어를 통해 차선을 유지하도록 설계. 두 점의 차이를 Error 값으로 설정해 PID 제어를 통해 중앙으로 올 수 있도록 함

---

## 필터

- 왼쪽 차선과 오른쪽 차선의 x좌표 중앙값에 필터를 사용해 부드럽게 조향할 수 있도록 설정
- 저주파 통과 필터(LPF), 평균 필터, 이동평균필터, 고주파 통과 필터(HPF) 여러 필터 중 적합한 필터를 찾기 위해 여러 테스트 주행을 실시
- 급 커브 구간에서 빠른 속도를 감속하고 방향각 제어를 해야하기 때문에 최근 값에 높은 가중치를 둬 민첩한 조향 제어를 할 수 있는 이동평균필터(Moving Average Filter)를 사용
- Sample(n)의 개수는 20으로 설정하고 Yaml파일에서 수정할 수 있도록 설정

---

## 제어

- 차선의 중앙을 어떤 상황에서도 잘 찾을 수 있도록 PID 제어기를 사용했다

1. P_Gain

랩타임을 줄이기 위해 직선 구간과 곡선 구간의 P_Gain 값을 다르게 설정했다. 직선 구간에서는 약 0.4 곡선 구간에서는 약 0.8 정도를 설정해줬다.  직선 구간에서도 완벽하게 차선을 인식해서 주행하는 것이 아니기 때문에 높은 P_Gain 값을 가지고 있으면 크게 흔들리고 감속하게 된다
곡선 구간을 판단하는 척도는 차선의 중앙 좌표와 차의 중앙 좌표가 일정 값 이상 차이가 나면 곡선 구간으로 인지하도록 코드를 설계했다. 하드웨어 적으로 최대 조향각이 -50~+50으로 한정되어있기 때문에 오버슈트가 발생할 만큼 큰 값으로 설정했다

2. D_Gain

D_Gain 값은 P_Gain 값에 맞춰 직진 구간에서 오버 슈트가 발생하지 않을 값으로 설정

3. I_Gain

I_Gain 값은 자이카를 고속 주행 컨셉에 맞춰 프로그래밍 했기 때문에 최소 속도를 높여서 정착시간까지 반응이 일어나지 않기 때문에 0에 근사하는 값으로 설정해서 PD 제어기에 가까운 역할을 수행하도록 했다

