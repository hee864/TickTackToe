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
        move_periodic, wait,
        DR_MV_MOD_REL, DR_FC_MOD_REL, DR_AXIS_Z, DR_TOOL
    )
    from DR_common2 import posx

    ON, OFF = 1, 0

    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            print("Wait for digital input")

    def release():
        down()
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait_digital_input(2)
        up()

    def grip():
        down()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait_digital_input(1)
        up()

    def justGrip():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait_digital_input(1)

    def justRelease():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait_digital_input(2)

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
            while True :
                
                if not check_force_condition(DR_AXIS_Z, max=5) ==0:

                    release_force()
                    release_compliance_ctrl()
                    print("정지")
                    time.sleep(0.3)
                    movel(rels[i], vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
                    up()
                    break
            time.sleep(0.2)


    circle1 = [pos11,pos21,pos31,pos41,pos51,pos61,pos71,pos81,pos91]
    circle2 = [pos12,pos22,pos32,pos42,pos52,pos62,pos72,pos82,pos92]
    circle3 = [pos13,pos23,pos33,pos43,pos53,pos63,pos73,pos83,pos93]
    ereaser=posx([370, -150, 10, 130, 180, 130])

    def ereasing():
        movel(ereaser, vel=VELOCITY, acc=ACC)
        justGrip()
        up()



        movel(pos51, vel=VELOCITY, acc=ACC)

        task_compliance_ctrl(stx=[100, 100, 100, 100, 100, 100])
        time.sleep(0.3)
        set_desired_force(fd=[0, 0, -5, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        print("내려가는중")
        while True :
            
            if not check_force_condition(DR_AXIS_Z, max=5) ==0:


                print("정지")
                time.sleep(0.3)

                move_periodic(amp=[80, 0, 0, 0, 0, 30], period=4.0, repeat=2, ref=DR_TOOL)
                time.sleep(0.3)
                release_force()
                release_compliance_ctrl()
                up()
                break

    def draw_circle(i):
        #펜 위치로 가서 잡고 올라가서
        # movel(posx([310, 36, 130, 130, 180, 130]), vel=VELOCITY, acc=ACC)
        # movel(posx([310, 36, 72, 130, 180, 130]), vel=VELOCITY, acc=ACC)
        # justGrip()
        # up()
        #원 위치로 가서 힘제어 순응제어 시작
        movel(circle1[i], vel=VELOCITY, acc=ACC)
        print("force_control_start")
        task_compliance_ctrl(stx=[100, 100, 100, 100, 100, 100])
        time.sleep(0.3)
        set_desired_force(fd=[0, 0, -5, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        print("내려가는중")
        while True :
            #힘을 받게 되면
            if not check_force_condition(DR_AXIS_Z, max=5) ==0:
                #바로 힘순응제어 풀고 
                release_force()
                release_compliance_ctrl()
                print("정지")
                time.sleep(0.3)
                print(get_current_posx())
                z= get_current_posx()[0][2]
                print(z)
                circle2[i][2] = z
                print(circle2[i])
                circle3[i][2] = z
                print(circle3)
                movec(circle2[i], circle3[i], vel=VELOCITY, acc=ACC, radius=0.00, ref=0, angle=[360.00, 0.00])
                up()            
                break
                
        time.sleep(0.3)

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

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
        win_cases = [[0,1,2],[3,4,5],[6,7,8],[0,3,6],[1,4,7],[2,5,8],[0,4,8],[2,4,6]]
        return any(all(board_state[i] == player for i in case) for case in win_cases)

    def get_best_move():
        for i in range(9):
            if board_state[i] == 0:
                board_state[i] = 2
                if is_winner(2):
                    board_state[i] = 0
                    return i
                board_state[i] = 0
        for i in range(9):
            if board_state[i] == 0:
                board_state[i] = 1
                if is_winner(1):
                    board_state[i] = 0
                    return i
                board_state[i] = 0
        for i in [4, 0, 2, 6, 8, 1, 3, 5, 7]:
            if board_state[i] == 0:
                return i
        return -1

    def process_move(rx_data):
        if rx_data < 0 or rx_data > 8 or board_state[rx_data] != 0:
            return "Invalid"
        board_state[rx_data] = 1
        if is_winner(1): return "User Wins"
        if all(cell != 0 for cell in board_state): return "Draw"
        robot_move = get_best_move()
        board_state[robot_move] = 2
        print_board_state() #터미널 창에도 그리기 
        draw_circle(robot_move) #로봇이 그릴 차례에 draw_circle 가서 그리기
        if is_winner(2): return f"{robot_move}:Robot Wins"
        if all(cell != 0 for cell in board_state): return f"{robot_move}:Draw"
        return {"move": robot_move} #클라이언트에게 위치 전송 
    
    #게임시작전 원래 포즈로 이동
    movej(JReady, vel=VELOCITY, acc=ACC)
    # 게임 시작 준비 (펜위치로 가서 펜잡고 선그리고)
    movel(posx([310, 36, 130, 130, 180, 130]), vel=VELOCITY, acc=ACC)
    movel(posx([310, 36, 72, 130, 180, 130]), vel=VELOCITY, acc=ACC)
    justGrip()
    up()
    draw_line()
    #펜위치로 가서 팬 놓고 
    # movel(posx([310, 36, 130, 130, 180, 130]), vel=VELOCITY, acc=ACC)
    # movel(posx([310, 36, 72, 130, 180, 130]), vel=VELOCITY, acc=ACC)
    # justRelease()
    # up()

    #다시 원래 포즈로 이동
    movej(JReady, vel=VELOCITY, acc=ACC)
    #소켓 통신 대기

    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind(('', 1024))
    server_sock.listen(1)
    print("[INFO] Server listening on port 1024")
    conn, addr = server_sock.accept()
    print(f"[INFO] Client connected: {addr}")
    conn_file = conn.makefile('r')
    
    game_over=False #종료 플래그 
    try: #소켓통신 으로 값 받아와서 process_move에 넣어주기
        while rclpy.ok() and not game_over:
            rclpy.spin_once(node, timeout_sec=0.01)
            ready, _, _ = select.select([conn], [], [], 0.1)
            if ready:
                msg = conn_file.readline().strip()
                if not msg:
                    print("[INFO] Client disconnected")
                    break
                if msg==9:
                    print('reset received from client')
                    print_board_state()

                    justRelease()
                    movej(JReady,vel=VELOCITY,acc=ACC)
                    
                    conn.sendall(b'reset DOne')
                    

                    board_state[:]=[0]*9
                    continue
                if msg.isdigit() and 0 <= int(msg) <= 8:
                    result = process_move(int(msg))
                    print(f"[DEBUG] Sending result: {result}")
                    conn.sendall((json.dumps(result) + "\n").encode())
                    if "status" in result and ("Wins" in result["status"] or "Draw" in result["status"]):
                        conn.sendall(b"Game Over\n")
                        game_over=True
                        break
               
    finally:
        #게임 종료 처리 =지우기+로봇 복귀
        justRelease()
        movej(JReady, vel=VELOCITY, acc=ACC)
        ereasing()
        movej(JReady, vel=VELOCITY, acc=ACC)
        conn.close()
        server_sock.close()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
