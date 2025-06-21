import rclpy
import socket
import time
import DR_init
import select
import json

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 100, 100

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
DR_init.__dsr__host = "127.0.0.1"
DR_init.__dsr__port = 12345  # virtual 모드 포트

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("ticktacktoe", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import movej, movel, movec, get_current_posx, set_tool, set_tcp
    from DR_common2 import posx, posj

    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind(('', 1024))
    server_sock.listen(1)
    print("[INFO] Server listening on port 1024")
    conn, addr = server_sock.accept()
    print(f"[INFO] Client connected: {addr}")
    conn_file = conn.makefile('r')

    board_state = [0] * 9

    def print_board_state():
        symbols = {0: ' ', 1: 'X', 2: 'O'}
        print("\nCurrent Board:")
        for i in range(3):
            row = [symbols[board_state[3 * i + j]] for j in range(3)]
            print(' ' + ' | '.join(row))
            if i < 2:
                print("---+---+---")
        print()

    def is_winner(player):
        win_cases = [
            [0, 1, 2], [3, 4, 5], [6, 7, 8],
            [0, 3, 6], [1, 4, 7], [2, 5, 8],
            [0, 4, 8], [2, 4, 6]
        ]
        return any(all(board_state[i] == player for i in case) for case in win_cases)

    def get_best_move():
        # 1. 로봇이 이길 수 있는 자리 찾기
        for i in range(9):
            if board_state[i] == 0:
                board_state[i] = 2
                if is_winner(2):
                    board_state[i] = 0
                    return i
                board_state[i] = 0

        # 2. 사용자가 이길 수 있는 자리 막기
        for i in range(9):
            if board_state[i] == 0:
                board_state[i] = 1
                if is_winner(1):
                    board_state[i] = 0
                    return i
                board_state[i] = 0

        # 3. 그 외 우선순위에 따라 놓기
        priority = [4, 0, 2, 6, 8, 1, 3, 5, 7]
        for i in priority:
            if board_state[i] == 0:
                return i

        return -1

    positions = [
        posx([635,-50,90,130,180,130]), posx([615,-70,90,130,180,130]), posx([595,-50,90,130,180,130]),
        posx([585,-50,90,130,180,130]), posx([565,-70,90,130,180,130]), posx([545,-50,90,130,180,130]),
        posx([535,-50,90,130,180,130]), posx([515,-70,90,130,180,130]), posx([495,-50,90,130,180,130]),
    ]
    circle2 = [posx([x[0], x[1], x[2] - 5, *x[3:]]) for x in positions]
    circle3 = [posx([x[0], x[1], x[2] - 10, *x[3:]]) for x in positions]

    def draw_circle(i):
        try:
            print(f"[DRAW] Drawing O at {i}")
            movel(positions[i], vel=VELOCITY, acc=ACC)
            movec(circle2[i], circle3[i], vel=VELOCITY, acc=ACC, radius=0.0)
            time.sleep(0.2)
        except Exception as e:
            print(f"[ERROR] 모션 수행 중 예외 발생: {e}")

    def process_move(rx_data):
        if rx_data < 0 or rx_data > 8 or board_state[rx_data] != 0:
            return {"move": -1, "status": "Invalid"}

        board_state[rx_data] = 1
        if is_winner(1):
            return {"move": -1, "status": "User Wins"}
        if all(cell != 0 for cell in board_state):
            return {"move": -1, "status": "Draw"}

        robot_move = get_best_move()
        board_state[robot_move] = 2
        print_board_state()

        # 모션은 나중에 수행
        result = {"move": robot_move, "status": "Continue"}

        if is_winner(2):
            result["status"] = "Robot Wins"
        elif all(cell != 0 for cell in board_state):
            result["status"] = "Draw"

        return result

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            ready, _, _ = select.select([conn], [], [], 0.1)
            if ready:
                try:
                    msg = conn_file.readline()
                except Exception as e:
                    print(f"[ERROR] 수신 중 예외 발생: {e}")
                    break

                if not msg:
                    print("[INFO] Client disconnected")
                    break

                msg = msg.strip()
                print(f"[DEBUG] Received: {msg}")

                if msg.isdigit() and 0 <= int(msg) <= 8:
                    response = process_move(int(msg))

                    try:
                        conn.sendall((json.dumps(response) + "\n").encode())
                    except Exception as e:
                        print(f"[ERROR] 전송 실패: {e}")
                        break

                    if response["status"] == "Continue":
                        draw_circle(response["move"])

                    if response["status"] in ["User Wins", "Robot Wins", "Draw"]:
                        try:
                            conn.sendall(b"{\"move\": -1, \"status\": \"Game Over\"}\n")
                        except:
                            pass
                        break
                else:
                    try:
                        conn.sendall(b"{\"move\": -1, \"status\": \"Invalid\"}\n")
                    except:
                        break
    finally:
        print("[INFO] Shutting down server")
        try:
            conn.close()
        except:
            pass
        server_sock.close()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
