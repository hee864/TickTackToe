import rclpy
import socket
import time
import DR_init
import select
import random

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 100, 100

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
DR_init.__dsr__host = "127.0.0.1"
DR_init.__dsr__port = 12345  # virtual mode

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("ticktacktoe", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import movej, movel
    from DR_common2 import posx

    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind(('', 1024))
    server_sock.listen(1)
    print("[INFO] Server listening on port 1024")

    conn, addr = server_sock.accept()
    print(f"[INFO] Client connected: {addr}")
    conn_file = conn.makefile('r')

    board = [0] * 9

    def is_winner(p):
        return any(all(board[i] == p for i in line) for line in [
            [0,1,2],[3,4,5],[6,7,8],[0,3,6],[1,4,7],[2,5,8],[0,4,8],[2,4,6]
        ])

    def get_best_move():
        for i in range(9):
            if board[i] == 0:
                board[i] = 2
                if is_winner(2):
                    board[i] = 0
                    return i
                board[i] = 0
        for i in range(9):
            if board[i] == 0:
                board[i] = 1
                if is_winner(1):
                    board[i] = 0
                    return i
                board[i] = 0
        for i in [4,0,2,6,8,1,3,5,7]:
            if board[i] == 0:
                return i
        return -1

    def get_easy_move():
        choices = [i for i in range(9) if board[i] == 0]
        return random.choice(choices) if choices else -1

    def get_robot_move():
        return get_best_move() if mode == "hard" else get_easy_move()

    pos_table = [posx([635 - 20*i, -50, 90, 130, 180, 130]) for i in range(9)]

    mode = None  # 초기 난이도

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            ready, _, _ = select.select([conn], [], [], 0.1)
            if not ready:
                continue

            msg = conn_file.readline().strip()
            print(f"[USER] {msg}")

            # 난이도 설정
            if mode is None:
                if msg == "-1":
                    mode = "hard"
                    conn.sendall(b"ok\n")
                elif msg == "-2":
                    mode = "easy"
                    conn.sendall(b"ok\n")
                else:
                    conn.sendall(b"-99\n")

            elif msg == "9":
                board = [0] * 9
                conn.sendall(b"reset\n")

            else:
                try:
                    user_move = int(msg)
                    if 0 <= user_move <= 8 and board[user_move] == 0:
                        board[user_move] = 1
                        if is_winner(1):
                            conn.sendall(b"12\n")
                            continue
                        if all(v != 0 for v in board):
                            conn.sendall(b"11\n")
                            continue

                        bot_move = get_robot_move()
                        board[bot_move] = 2
                        movel(pos_table[bot_move], vel=VELOCITY, acc=ACC)
                        time.sleep(0.3)
                        conn.sendall(f"{bot_move}\n".encode())

                        # 승리 체크 추가 응답
                        if is_winner(2):
                            conn.sendall(b"10\n")
                        elif all(v != 0 for v in board):
                            conn.sendall(b"11\n")
                    else:
                        conn.sendall(b"-99\n")
                except:
                    conn.sendall(b"-99\n")
    finally:
        conn.close()
        server_sock.close()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
