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
DR_init.__dsr__port = 12345

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("ticktacktoe", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import (
        movej, movel, movec,
        get_current_posx,
        set_tool, set_tcp,
        set_digital_output, get_digital_input,
        task_compliance_ctrl, set_desired_force,
        release_force, release_compliance_ctrl, check_force_condition,
        move_periodic, wait,amove_periodic,
        DR_MV_MOD_REL, DR_FC_MOD_REL, DR_AXIS_Z, DR_TOOL
    )
    from DR_common2 import posx
    ON, OFF = 1, 0

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    board = [0] * 9
    mode = None  # 초기 난이도

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

    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            print("Wait for digital input")


    def justGrip():

        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait_digital_input(1)
        time.sleep(0.3)

    def justRelease():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        # wait_digital_input(2)

    def down():
        pos_down = posx([0,0,-70,0,0,0])
        movel(pos_down, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)

    def up():
        pos_up = posx([0,0,40,0,0,0])
        movel(pos_up, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)

    JReady = [0, 0, 90, 0, 90, 0]

    pos11 = posx([635,-50, 80, 130, 180.0, 130])
    pos12 = posx([615,-70, 80, 130, 180.0, 130])
    pos13 = posx([595,-50, 80, 130, 180.0, 130])
    pos21 = posx([585,-50, 80, 130, 180.0, 130])
    pos22 = posx([565,-70, 80, 130, 180.0, 130])
    pos23 = posx([545,-50, 80, 130, 180.0, 130])
    pos31 = posx([535,-50, 80, 130, 180.0, 130])
    pos32 = posx([515,-70, 80, 130, 180.0, 130])
    pos33 = posx([495,-50, 80, 130, 180.0, 130])
    pos41 = posx([635, 0, 80, 130, 180.0, 130])
    pos42 = posx([615,-20, 80, 130, 180.0, 130])
    pos43 = posx([595, 0, 80, 130, 180.0, 130])
    pos51 = posx([585, 0, 80, 130, 180.0, 130])
    pos52 = posx([565,-20, 80, 130, 180.0, 130])
    pos53 = posx([545, 0, 80, 130, 180.0, 130])
    pos61 = posx([535, 0, 80, 130, 180.0, 130])
    pos62 = posx([515,-20, 80, 130, 180.0, 130])
    pos63 = posx([495, 0, 80, 130, 180.0, 130])
    pos71 = posx([635, 50, 80, 130, 180.0, 130])
    pos72 = posx([615,30, 80, 130, 180.0, 130])
    pos73 = posx([595, 50, 80, 130, 180.0, 130])
    pos81 = posx([585, 50, 80, 130, 180.0, 130])
    pos82 = posx([565,30, 80, 130, 180.0, 130])
    pos83 = posx([545, 50, 80, 130, 180.0, 130])
    pos91 = posx([535, 50, 80, 130, 180.0, 130])
    pos92 = posx([515,30, 80, 130, 180.0, 130])
    pos93 = posx([495, 50, 80, 130, 180.0, 130])

    circle1 = [pos11,pos21,pos31,pos41,pos51,pos61,pos71,pos81,pos91]
    circle2 = [pos12,pos22,pos32,pos42,pos52,pos62,pos72,pos82,pos92]
    circle3 = [pos13,pos23,pos33,pos43,pos53,pos63,pos73,pos83,pos93]

    line1 = posx([640, -25, 80, 130, 180, 130])
    line2 = posx([640,  25, 80, 130, 180, 130])
    line3 = posx([590,  75, 80, 130, 180, 130])
    line4 = posx([540,  75, 80, 130, 180, 130])

    lines = [line1, line2, line3, line4]

    def draw_line():
        rel1 = posx([-150, 0, 0, 0, 0, 0])
        rel2 = posx([0, -150, 0, 0, 0, 0])
        rels = [rel1, rel1, rel2, rel2]

        for i in range(0,4):
            movel(lines[i], vel=VELOCITY, acc=ACC)
            task_compliance_ctrl(stx=[100, 100, 100, 100, 100, 100])
            time.sleep(0.3)
            set_desired_force(fd=[0, 0, -5, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            print("내려가는중")
            while True:
                if not check_force_condition(DR_AXIS_Z, max=5) == 0:
                    release_force()
                    release_compliance_ctrl()
                    print("정지")
                    time.sleep(0.3)
                    movel(rels[i], vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
                    up()
                    break
            time.sleep(0.2)
    ereaser=posx([370, -150, 40, 130, 180, 130])
    def ereasing():
        movel(ereaser, vel=VELOCITY, acc=ACC)
        pos_down = posx([0,0,-30,0,0,0])
        movel(pos_down, vel=VELOCITY, acc=ACC, mod = DR_MV_MOD_REL)
        justGrip()
        time.sleep(0.3)
        up()
        erase1=posx([565, -40, 30, 130, 180.0, 130])
        erase2=posx([565, 40, 30, 130, 180.0, 130])
        erase_list=[erase1, erase2]
        e = [100, 0, 0, 0, 0, 10]

        for i in range (2):
            movel(erase_list[i], vel=VELOCITY, acc=ACC)
            task_compliance_ctrl(stx=[100, 100, 100, 100, 100, 100])
            time.sleep(0.3)
            set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            print("내려가는중")
            while True :
                
                if not check_force_condition(DR_AXIS_Z, max=5) ==0:


                    print("정지")
                    time.sleep(0.3)

                    amove_periodic(amp=e, period=5.0, repeat=2, ref=DR_TOOL)
                    time.sleep(12)
                    # amove_periodic(amp=[0, 80, 0, 0, 0, 30], period=4.0, repeat=2, ref=DR_TOOL)
                    # time.sleep(5)
                    release_force()
                    time.sleep(0.3)
                    release_compliance_ctrl()
                    time.sleep(0.3)
                    up()
                    break
        movel(ereaser, vel=VELOCITY, acc=ACC)
        movel(pos_down, vel=VELOCITY, acc=ACC, mod = DR_MV_MOD_REL)
        justRelease()
        time.sleep(0.3)
        up()
        movej(JReady, vel=VELOCITY, acc=ACC)
        movel(posx([310, 36, 130, 130, 180, 130]), vel=VELOCITY, acc=ACC)
        movel(posx([310, 36, 72, 130, 180, 130]), vel=VELOCITY, acc=ACC)
        justGrip()
        time.sleep(0.3)
        up()


    # movej(JReady, vel=VELOCITY, acc=ACC)
    # draw_line()


    def draw_circle(i):
        movel(circle1[i], vel=VELOCITY, acc=ACC)
        print("force_control_start")
        task_compliance_ctrl(stx=[100]*6)
        time.sleep(0.3)
        set_desired_force(fd=[0, 0, -5, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        print("내려가는중")
        while True:
            if not check_force_condition(DR_AXIS_Z, max=5) == 0:
                release_force()
                release_compliance_ctrl()
                print("정지")
                time.sleep(0.3)
                z = get_current_posx()[0][2]
                circle2[i][2] = z
                circle3[i][2] = z
                movec(circle2[i], circle3[i], vel=VELOCITY, acc=ACC, radius=0.0, ref=0, angle=[360.0, 0.0])
                up()
                break
        time.sleep(0.3)

    def robot_dance():
        print("[INFO] Robot is dancing!")
        for _ in range(2):
            movej([0, 0, 90, 0, 60, 0], vel=80, acc=80)
            movej([0, 0, 90, 0, 120, 0], vel=80, acc=80)
        movej(JReady, vel=VELOCITY, acc=ACC)
    
    
    justRelease()
    movej(JReady, vel=VELOCITY, acc=ACC)
    movel(posx([310, 36, 130, 130, 180, 130]), vel=VELOCITY, acc=ACC)
    movel(posx([310, 36, 72, 130, 180, 130]), vel=VELOCITY, acc=ACC)
    justGrip()
    time.sleep(0.3)
    up()
    movej(JReady, vel=VELOCITY, acc=ACC)
    draw_line()

    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind(('', 1024))
    server_sock.listen(1)
    print("[INFO] Server listening on port 1024")

    conn, addr = server_sock.accept()
    print(f"[INFO] Client connected: {addr}")
    conn_file = conn.makefile('r')

    try:
        while rclpy.ok():
        # while True:
           
            rclpy.spin_once(node, timeout_sec=0.01)  #spin_once
            ready, _, _ = select.select([conn], [], [], 0.1)
            if not ready:
                continue

            msg = conn_file.readline().strip()
            print(f"[USER] {msg}")

            if mode is None:
                if msg == "-1":
                    mode = "hard"
                    conn.sendall(b"ok\n")
                elif msg == "-2":
                    mode = "easy"
                    conn.sendall(b"ok\n")
                else:
                    conn.sendall(b"-99\n")
            elif msg == "9": #reset
                board=[0]*9
                # conn.sendall(b"reset\n")
                movej(JReady, vel=VELOCITY, acc=ACC)
                movel(posx([310, 36, 130, 130, 180, 130]), vel=VELOCITY, acc=ACC)
                movel(posx([310, 36, 72, 130, 180, 130]), vel=VELOCITY, acc=ACC)
                justRelease()
                up()
                ereasing()
                draw_line()
                #pass
            #종료 추가 (클라이언트에서 창을 닫으면 exit을 보내도록 함)
            if msg == "exit":
                print("[INFO] 클라이언트가 종료했습니다.")
                break

                
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
                        # conn.sendall(f"{bot_move}\n".encode())
                        draw_circle(bot_move)
                        response=f"{bot_move}\n"
                        if is_winner(2):
                            
                            response+="10\n"
                            robot_dance()
                            time.sleep(1)
                            # conn.sendall(b"10\n")
                        elif all(v != 0 for v in board):
                            # conn.sendall(b"11\n")
                            response+="10\n"
                        conn.sendall(response.encode())
                    else:
                        conn.sendall(b"-99\n")
                except:
                    conn.sendall(b"-99\n")

    finally:
        # movej(JReady, vel=VELOCITY, acc=ACC)
        # movel(posx([310, 36, 130, 130, 180, 130]), vel=VELOCITY, acc=ACC)
        # movel(posx([310, 36, 72, 130, 180, 130]), vel=VELOCITY, acc=ACC)
        # justRelease()
        # up()
        # ereasing()
        # movej(JReady, vel=VELOCITY, acc=ACC)
        conn.close()
        server_sock.close()
        rclpy.shutdown()

if __name__ == "__main__":
    main()