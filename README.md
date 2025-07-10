# 🤖 틱택토 로봇 게임 (Tic-Tac-Toe Robot Game)

본 프로젝트는 **두산 로봇 API (DR SDK)**와 **ROS 2**를 활용한 다양한 로봇 제어 예제를 포함하고 있으며, 특히 **틱택토 로봇 게임**, **힘 제어 기반 블록 조작**, **GUI 기반 로봇 위치 제어** 등의 실험을 포함합니다.
틱택토 로봇 게임은 사용자가 GUI로 틱택토를 진행하고 로봇이 실제로 동작하며 'O' 표시를 그리는 시스템입니다.  
Tkinter 기반의 GUI, Python 소켓을 이용한 서버-클라이언트 구조, 그리고 실제 로봇 제어가 결합된 프로젝트입니다.


---

## 📁 프로젝트 구조

```
.
├── README.md
└── src
    ├── LICENSE
    └── rokey
        ├── package.xml
        ├── resource
        │   └── rokey
        ├── rokey
        │   ├── basic
        │   │   ├── block.py              # 블록 순응제어 및 진동 후 파지
        │   │   ├── data/
        │   │   │   └── calibrate_data.json
        │   │   ├── force_control.py      # 힘 조건 충족 시 멈추는 제어
        │   │   ├── get_current_pos.py    # GUI 기반 현재 위치 시각화
        │   │   ├── getting_position.py   # 이동 후 위치 출력
        │   │   ├── grip.py               # 그리퍼 제어 반복
        │   │   ├── jog_complete.py       # Jog UI로 조인트/직교 이동 테스트
        │   │   ├── move.py               # 단순 위치 이동 시퀀스
        │   │   ├── move_periodic.py      # 주기 진동 기반 테스트
        │   │   ├── movesx_test.py        # 사인 곡선 궤적 따라 이동
        │   │   ├── ros2topictic.py       # ROS2 기반 틱택토 로직
        │   │   ├── server_example.py     # 틱택토 서버 예제 (랜덤 응답)
        │   │   ├── test.py               # 틱택토 최적 수 계산 및 movec 동작
        │   │   └── ttt_sim.py            # 가상 모드 기반 틱택토 서버
        │   ├── __init__.py
        │   └── ticktacktoe
        │       ├── gui_ex.py             # GUI 클라이언트
        │       └── ttt_number.py         # 실제 실행용 틱택토 로봇 제어 서버
        ├── setup.cfg
        └── setup.py
```

---
## 🔧 사전 준비 사항

### ✅ 시스템 요구사항

- Python 3.8 이상
- ROS 2 Foxy 또는 이후 버전
- 두산 로봇 제어 패키지 (DR SDK)
- 로봇이 실제 연결된 상태 또는 시뮬레이터 환경
## 🕹 주요 실행 예제

### 🎮 Tic-Tac-Toe Robot (틱택토 게임)

- **실행 명령**
    ```bash
    ros2 run rokey ttt_number
    python3 src/rokey/rokey/ticktacktoe/gui_ex.py
    ```

- **기능**
    - 사용자 GUI 클릭 → 서버로 수 전송
    - 로봇이 최적 위치에 원 그리기
    - 하드모드(AI) / 이지모드(랜덤) 지원
    - 게임 리셋, 승리 시 댄스, 보드 지우기 동작
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

python3 gui_ex.py

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
🖥️ GUI 흐름 (gui_ex.py)

    소켓 연결 후 난이도 선택

    사용자 수 클릭 → 서버 전송

    서버 응답에 따라 로봇 수 표시

    게임 종료 시 결과 표시 및 자동 리셋

📌 주의사항

    로봇 제어에는 위험이 따르므로 안전 확보 후 실행할 것

    DR_init.py 내 IP, 모델 정보는 본인의 로봇 환경에 맞게 수정 필요

    실제 하드웨어 사용 시, 각 좌표의 안전성은 별도로 보장해야 함

✨ 향후 확장 가능성

    🔊 음성 명령 기반 게임 진행

    🌐 Flask + WebSocket 기반 웹 게임 인터페이스

    🧠 강화학습 기반 로봇 학습형 틱택토 AI
---

### 📦 블록 진동 및 파지 제어 (`block.py`)

- `순응제어 + 주기적 진동`으로 블록을 위로 뽑아냄
- `task_compliance_ctrl`, `set_desired_force`, `move_periodic` 활용

---

### 🖱 Jog UI 테스트 (`jog_complete.py`)

- Tkinter 기반의 포지션 제어 툴
- 조인트/직교 좌표 이동, 증분 버튼, z-axis 정렬, grip/release 버튼 제공

---

## 📡 ROS2 통신 기반

- `/dsr01/msg/current_posx`, `/dsr01/msg/joint_state` 등 토픽 구독
- `/tictactoe_input` 토픽을 통한 사용자 수 입력 처리
- `SetRobotMode` 서비스 사용 예제 포함

---

## 📂 test/ 디렉터리 요약

| 파일명 | 설명 |
|--------|------|
| `test.py` | 틱택토 로직 + 최적 수 계산 + movec 원 그리기 |
| `ttt_sim.py` | 가상 모드 기반 틱택토 서버 (movel만 수행) |
| `server_example.py` | 랜덤 수 전송 테스트용 서버 |
| `move.py`, `grip.py` 등 | 이동, 그리퍼, 진동 등 개별 기능 단위 테스트 |

---



