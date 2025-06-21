# [383.25, 31.61, 6.97, 28.85 ,-180.00, 26.79 ]-block1
#[ 383., -65.29 ,10.42, 170.12, -179.74 ,167.70] -block2
#up block [471.00,-63.28,36.69,163.03,-179.59,160.34] 순응제어 전 좌표1
#up block 2 [471.16,20.03,35.21,149.51,-179.64,146.98] 순응제어 전 좌표 2
import rclpy
import DR_init
import time
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60# for[]DR_init.__dsr__id = ROBOT_ID
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
            DR_BASE,
            DR_FC_MOD_ABS,
            release_force,
            get_current_posx,
            set_stiffnessx,
            amove_periodic,
            check_position_condition,
            mwait,
            move_periodic,
            DR_SSTOP,        )
        from DR_common2 import posx, posj    
    except ImportError as e:
            print(f"Error importing DSR_ROBOT2 : {e}")
            return
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            print("Wait for digital input")
            pass
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
        movel(pos_down, vel=VELOCITY, acc=ACC, mod = DR_MV_MOD_REL)
    def up():
        pos_up=posx([0,0,70,0,0,0])
        movel(pos_up,vel=VELOCITY,acc=ACC,mod=DR_MV_MOD_REL)
    block1=posx([383.25, 31.61, 6.97, 28.85 ,-180.00, 26.79 ])
    block2=posx([ 383., -65.29 ,10.42, 170.12, -179.74 ,167.70])
    block1_up=posx([471.00,-63.28,36.69,163.03,-179.59,160.34])
    block2_up=posx([471.16,20.03,35.21,149.51,-179.64,146.98])    
     
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")
    while rclpy.ok():
        print("순응제어 진입")
        task_compliance_ctrl(stx=[100]*6)
        time.sleep(0.3)
        set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        time.sleep(0.3)

        print("진동 시작")
        move_periodic(
            amp=[0.0, 0.0, 5.0, 0.0, 3.0, 0.0],       # Z축 5mm + Pitch 3도 진동
            period=[0.0, 0.0, 1.0, 0.0, 0.8, 0.0],    # 각각의 주기 설정
            atime=0.5,
            repeat=3,
            ref=DR_BASE
        )
        wait(3.0)  # 충분히 기다리기

        print("힘 해제 및 위로 뽑기")
        release_force()
        release_compliance_ctrl()
        up()
    rclpy.shutdown()
    if __name__ == "__main__":
        main()