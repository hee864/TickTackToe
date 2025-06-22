import rclpy
import DR_init
import time
# Robot 기본 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            wait,
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            set_digital_output,
            get_digital_input,
            set_tool,
            set_tcp,
            movej,
            movel,
            DR_MV_MOD_REL,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            check_position_condition,
            DR_SSTOP,
            DR_BASE,
            release_force,
            get_current_posx,
            
            
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            print("Wait for digital input")

    def release():
        
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait_digital_input(2)

    def grip():
        
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait_digital_input(1)

    def up():
        pos_up = posx([0, 0, 30, 0, 0, 0])
        movel(pos_up, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)

    # 초기 자세
    JReady = [0, 0, 90, 0, 90, 0]

    # A 위치들 (검사 대상)
    pos_a = [
        0,
        posx([601.98, -3.99, 94.2, 151.17, -180.0, 148.85]),
        posx([550.44, -4.44, 94.2, 122.11, -179.91, 119.75]),
        posx([499.93, -3.36, 94.2, 96.24, -179.93, 93.87]),
        posx([601.44, 46.94, 94.2, 31.77, -179.71, 29.09]),
        posx([550.36, 47.54, 94.2, 17.4, -179.4, 14.45]),
        posx([499.48, 47.95, 94.2, 18.63, -179.21, 15.26]),
        posx([601.78, 98.37, 94.2, 19.94, -179.14, 16.37]),
        posx([550.48, 98.66, 94.2, 23.14, -179.23, 19.42]),
        posx([499.25, 99.09, 94.2, 20.67, -179.12, 16.86]),
    ]

    # B 위치들 (분류 대상)
    pos_b = [
        0,
        posx([600.63, -154.13, 94.2, 5.12, -178.91, 1.34]),
        posx([549.58, -153.99, 94.2, 176.65, 178.43, 173.07]),
        posx([498.57, -153.31, 94.2, 176.63, 178.56, 173.29]),
        posx([600.49, -102.33, 94.2, 33.84, -179.78, 31.26]),
        posx([549.73, -102.73, 94.2, 0.07, -178.47, -3.21]),
        posx([498.24, -103.54, 94.2, 173.96, -180.0, 173.39]),
        posx([600.5, -51.41, 94.2, 2.15, -178.0, -1.36]),
        posx([549.47, -51.78, 94.2, 1.07, -177.91, -2.37]),
        posx([498.39, -52.18, 94.2, 179.11, 177.79, 175.91]),
    ]

    small_idx = 1
    middle_idx = 4
    long_idx = 7

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    # set_tool("TCP208mm")
    # set_tcp("Tool Weight_3_24")
    movej(JReady, vel=VELOCITY, acc=ACC)

    for i in range(1, 10):
        
        movel(pos_a[i], vel=VELOCITY, acc=ACC)
        # 순응 제어 및 힘 설정
        task_compliance_ctrl(stx=[100] * 6)
        time.sleep(0.2)
        set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

        # 힘이 감지될 때까지 아래로 내림
        while True:

            if not check_force_condition(DR_AXIS_Z, max=10) == 0:
                z_pos = get_current_posx()[0][2]
                release_force()
                release_compliance_ctrl()
                break

        
        if 42 < z_pos <= 46:
            print("짧은 막대 감지")
            up()
            grip()
            movel(pos_b[small_idx], vel=VELOCITY, acc=ACC)
            release()
            small_idx += 1
        elif 52 < z_pos <= 56:
            print(" 중간 막대 감지")
            up()
            grip()
            movel(pos_b[middle_idx], vel=VELOCITY, acc=ACC)
            release()
            middle_idx += 1
        else:
            print(" 긴 막대 감지")
            
            up()
            grip()
            movel(pos_b[long_idx], vel=VELOCITY, acc=ACC)
            release()
            long_idx += 1
                   
        

       
       

       

       
    rclpy.shutdown()

if __name__ == "__main__":
    main()