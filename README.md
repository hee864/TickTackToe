# 🤖 틱택토 로봇 게임 (Tic-Tac-Toe Robot Game)

이 프로젝트는 **두산 로봇 API (DR SDK)** 를 활용하여, 사용자가 GUI로 틱택토를 진행하고 로봇이 실제로 동작하며 'O' 표시를 그리는 시스템입니다.  
Tkinter 기반의 GUI, Python 소켓을 이용한 서버-클라이언트 구조, 그리고 실제 로봇 제어가 결합된 프로젝트입니다.


## 🔧 사전 준비 사항

### ✅ 시스템 요구사항

- Python 3.8 이상
- ROS 2 Foxy 또는 이후 버전
- 두산 로봇 제어 패키지 (DR SDK)
- 로봇이 실제 연결된 상태 또는 시뮬레이터 환경

# 🤖 틱택토 로봇 게임 (Tic-Tac-Toe Robot Game)

...

## 🎥 데모 영상

[![틱택토 로봇 데모](https://img.youtube.com/vi/VpZkLcmdUkc/0.jpg)](https://youtu.be/VpZkLcmdUkc)

> 영상 클릭 시 실제 로봇이 보드에 'O'를 그리는 틱택토 플레이를 확인할 수 있습니다.


```bash
pip install rclpy tkinter

    tkinter는 OS에 따라 별도 설치가 필요할 수 있습니다. (예: Ubuntu → sudo apt install python3-tk)

🚀 실행 방법
1. 로봇 서버 실행

ros2 run rokey ttt_number

    ROS 2 노드가 생성되고 로봇이 초기 위치로 이동합니다.

    이후 포트 1024에서 GUI 클라이언트를 기다립니다.

2. GUI 클라이언트 실행

python3 gui_client.py

    사용자 인터페이스가 열리고, 난이도를 먼저 선택합니다.

    사용자가 먼저 'X'로 수를 두고, 로봇이 'O'를 그립니다.

🎮 GUI 기능

    난이도 선택: 하드(전략적) / 이지(랜덤)

    게임 판정: 승, 패, 무승부 자동 감지

    로봇 반응: 승리 시 댄스, 보드 리셋 시 실제 '지우개 동작'

    게임 리셋: 언제든지 클릭 한 번으로 초기화 가능

    자동 종료: GUI 닫기 시 서버에 종료 메시지 전송

🔌 소켓 통신 프로토콜
클라이언트 → 서버
명령	설명
-1\n	하드 모드 선택
-2\n	이지 모드 선택
0 ~ 8\n	사용자 수 (틱택토 보드 위치)
9\n	게임 리셋 요청
exit\n	서버 종료 요청
서버 → 클라이언트
응답	의미
0~8	로봇이 둔 위치
10	로봇 승리
11	무승부
12	사용자 승리
-99	잘못된 명령 or 예외 발생
📐 로봇 좌표 개요

    각 위치는 posx([x, y, z, rx, ry, rz]) 형식

    3개의 원 좌표 (circle1, circle2, circle3)를 이용해 movec() 로 원 그리기

    수직 압력 감지를 통해 표면 접촉 후 그리기 수행

    draw_line() 함수는 틱택토 보드 선을 자동으로 그림

    ereasing() 함수는 실제 보드를 지우는 움직임을 수행

💡 주요 함수 요약 (robot_control.py)
함수명	역할
get_best_move()	하드 모드용 AI
get_easy_move()	랜덤 수 선택
draw_circle(i)	지정 위치에 원 그리기
draw_line()	보드의 선 긋기
ereasing()	보드 지우기 동작
robot_dance()	승리 시 로봇 댄스
🖥️ GUI 흐름 (gui_client.py)

    소켓 연결 후 난이도 선택

    사용자 수 클릭 → 서버 전송

    서버 응답에 따라 로봇 수 표시

    게임 종료 시 결과 표시 및 자동 리셋

📌 주의사항

    로봇 제어에는 위험이 따르므로 안전 확보 후 실행할 것

    DR_init.py 내 IP, 모델 정보는 본인의 로봇 환경에 맞게 수정 필요

    실제 하드웨어 사용 시, 각 좌표의 안전성은 별도로 보장해야 함

✨ 향후 확장 가능성

    📷 카메라 비전 기반 사용자 수 인식

    🔊 음성 명령 기반 게임 진행

    🌐 Flask + WebSocket 기반 웹 게임 인터페이스

    🧠 강화학습 기반 로봇 학습형 틱택토 AI
