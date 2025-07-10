# 🤖 틱택토 로봇 게임 (Tic-Tac-Toe Robot Game)

본 프로젝트는 **두산 로봇 API (DR SDK)**와 **ROS 2**를 활용한 다양한 로봇 제어 예제를 포함하고 있으며,  
특히 다음과 같은 실험을 포함합니다:

- 틱택토 로봇 게임  
- 힘 제어 기반 블록 조작  
- GUI 기반 로봇 위치 제어

틱택토 게임은 사용자가 GUI로 수를 두고, 로봇이 실제로 'O' 표시를 그리는 인터랙티브 시스템입니다.

---

## 📁 프로젝트 구조

```
.
├── README.md
│   └─ 프로젝트 개요 및 실행 방법 안내
└── src
    ├── LICENSE
    │   └─ 프로젝트 라이선스 (예: MIT, Apache 2.0)
    └── rokey
        ├── package.xml
        │   └─ ROS2 패키지 메타 정보
        ├── resource/
        │   └─ rokey 관련 리소스 파일 (필수 구조)
        ├── rokey/
        │   ├── basic/
        │   │   ├── block.py              # 순응제어 + 진동 기반 블록 파지
        │   │   ├── force_control.py      # 힘 조건 만족 시 정지
        │   │   ├── get_current_pos.py    # 현재 위치 실시간 GUI 출력 및 복사
        │   │   ├── getting_position.py   # 이동 후 위치 출력 및 기록
        │   │   ├── grip.py               # 그리퍼 개폐 반복 동작 테스트
        │   │   ├── jog_complete.py       # 조인트/직교 이동 GUI + 그리퍼 + 복사 기능
        │   │   ├── move.py               # 기본 위치 간 MoveJ / MoveL 테스트
        │   │   ├── move_periodic.py      # amove_periodic을 활용한 진동 테스트
        │   │   ├── movesx_test.py        # 사인 곡선을 따라 연속 이동 (movesx 사용)
        │   │   ├── ros2topictic.py       # ROS2 기반 틱택토 게임 노드
        │   │   ├── heewoo.py             # 힘 감지로 블록 높이를 측정하여 크기별로 정렬
        │   │   └── data_recording.py     # 로봇 위치 + 카메라 이미지 저장 (캘리브레이션용)
        │
        │   └── ticktacktoe/
        │       ├── gui_ex.py             # 사용자 입력용 Tkinter GUI 클라이언트
        │       ├── ttt_number.py         # 틱택토 상태 판별 및 서버 로직
        │       ├── test.py               # movec 기반 O그리기 + 가상 틱택토 실행
        │       └── ttt_sim.py            # 가상 모드 틱택토 서버 실행 스크립트
        ├── setup.cfg                    # Python 패키지 설정 정보
        └── setup.py                     # 패키지 설치 및 배포용 스크립트

```

---

## 🔧 사전 준비 사항

- Python 3.8 이상  
- ROS 2 Foxy 또는 이후 버전  
- 두산 로봇 제어 패키지 (DR SDK)  
- 실제 로봇 또는 가상 제어 환경  
- `tkinter` (GUI용: Ubuntu → `sudo apt install python3-tk`)

---

## 🕹 주요 실행 예제

### 🎮 틱택토 로봇 게임 실행

```bash
# 로봇 서버 실행
ros2 run rokey ttt_number

# GUI 실행
python3 src/rokey/rokey/ticktacktoe/gui_ex.py
```

- 하드 모드 (AI) / 이지 모드 (랜덤)
- GUI에서 클릭 → 로봇이 최적 위치에 원 그림
- 게임 리셋 / 보드 지우기 / 승리 댄스 동작 포함

---

## 🎥 데모 영상

[![틱택토 로봇 데모](https://img.youtube.com/vi/VpZkLcmdUkc/0.jpg)](https://youtu.be/VpZkLcmdUkc)

> 실제 로봇이 보드에 'O'를 그리고 반응하는 모습을 확인할 수 있습니다.

---

## 🔌 소켓 통신 프로토콜

### 📤 클라이언트 → 서버
| 명령어 | 설명 |
|--------|------|
| `-1\n` | 하드 모드 선택 |
| `-2\n` | 이지 모드 선택 |
| `0~8\n` | 사용자 수 입력 |
| `9\n` | 게임 리셋 요청 |
| `exit\n` | 서버 종료 요청 |

### 📥 서버 → 클라이언트
| 응답값 | 의미 |
|--------|------|
| `0~8` | 로봇이 둔 위치 |
| `10` | 로봇 승리 |
| `11` | 무승부 |
| `12` | 사용자 승리 |
| `-99` | 예외 or 잘못된 명령 |

---

## 📐 로봇 제어 구조 (틱택토)

- 각 위치는 `posx([x, y, z, rx, ry, rz])` 형식
- `movec()`로 3점 원 그리기 수행
- `draw_line()`, `ereasing()`으로 보드선 및 리셋 동작 수행
- `robot_dance()`로 승리 시 세레모니 동작

---

## 🧩 basic 파일 미니 프로젝트  


🎮 시뮬레이션 상의 틱택토 게임 구현 (test.py, ttt_sim.py)

    GUI를 통해 사용자가 틱택토 게임을 플레이하면
    로봇이 movec 명령으로 실제 보드에 'O'를 그리고 승패를 판정합니다.

    Gazebo 환경에서 실행되는 시뮬레이션 기반 서버 구현
![sim](image/simulationver.gif)

📦 힘 제어 & 순응 제어 기반 블록 정렬 (heewoo.py)
[![블록정렬]](https://youtu.be/pemCm9gFjXA)

    Z축 방향 힘 감지로 블록 길이를 측정하고,
    이를 기준으로 짧은/중간/긴 블록을 서로 다른 위치로 분류

    2FG 그리퍼를 사용해 그립/릴리즈 동작 포함

    총 9개 블록에 대해 자동 분류 실행

🦾 움직임 테스트 파일들
	파일명	설명
	move.py	MoveJ → MoveL 명령을 이용해 기본 위치 이동을 반복 수행
	move_periodic.py	amove_periodic 명령으로 툴 기준 회전 진동 테스트 (Rx, Rz 등)
	movesx_test.py	X-Y 평면 상의 사인 궤적을 따라 연속적으로 이동 (곡선 경로 테스트)
	block.py	순응 제어 + Z축 진동(move_periodic)으로 블록 감지 및 파지
	force_control.py	원하는 Z축 방향 힘을 설정한 뒤, 힘이 감지될 때까지 이동 후 정지
	getting_position.py	정해진 여러 위치로 이동하며, 각 위치에서의 실제 posx 값을 출력
	get_current_pos.py	현재 위치(posx, posj)를 GUI 상에 실시간으로 표시하고 복사
	grip.py	디지털 출력 기반 그리퍼 개폐 동작 반복 테스트
	jog_complete.py	GUI 기반으로 조인트/직교 이동 + 그리퍼 조작 + 위치 복사 기능 통합

## 📡 ROS2 통신 기능

- `/dsr01/msg/current_posx`, `/dsr01/msg/joint_state` 구독
- `/tictactoe_input` 구독을 통한 틱택토 입력 처리
- `SetRobotMode` 서비스 호출 예제 포함

---


## ✨ 향후 확장 가능성

- 🔊 음성 명령 기반 게임 진행  
- 🌐 Flask + WebSocket 기반 웹 게임 서비스  
- 🧠 강화학습 기반 로봇 학습형 틱택토 AI

---

## 📄 라이선스

본 프로젝트는 **Apache 2.0 License** 하에 배포됩니다.
