# pick and place in 1 method. from pos1 to pos2 @20241104
import rclpy
import DR_init
import time

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60,60
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
ON, OFF = 1, 0
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)    
    DR_init.__dsr__node = node    
    try:
        from DSR_ROBOT2 import (
            # set_digital_output,
            # get_digital_input,
            # set_tool,
            # set_tcp,
            # movej,
            # movel,
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
            release_force,
            get_current_posx
        )        
        from DR_common2 import posx, posj    


    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return    
    


    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            print("Wait for digital input")
            pass    
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
        movel(pos_down, vel=VELOCITY, acc=ACC, mod = DR_MV_MOD_REL)
    def up():
        pos_up=posx([0,0,70,0,0,0])
        movel(pos_up,vel=VELOCITY,acc=ACC,mod=DR_MV_MOD_REL)
    # 44 54 64



    def check(index):
        print("force_control_start")
        task_compliance_ctrl(stx=[100, 100, 100, 100, 100, 100])
        time.sleep(0.3)
        set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        print("내려가는중")
        while True :
            
            if not check_force_condition(DR_AXIS_Z, max=10) ==0:

                release_force()
                release_compliance_ctrl()
                print("정지")
                break


        time.sleep(0.3)
        z= get_current_posx()[0][2]
        if z>60:
            large.append(posa[index])
            print('큰거')
        elif z>50:
            medium.append(posa[index])
            print('중간거')
        else:
            small.append(posa[index])
            print('작은거')



    JReady = [0, 0, 90, 0, 90, 0]    
    pos1 = posx([601.98, -3.99, 94.2, 151.17, -180.0, 148.85])
    pos2 = posx([550.44, -4.44, 94.2, 122.11, -179.91, 119.75])
    pos3 = posx([499.93, -3.36, 94.2, 96.24, -179.93, 93.87])
    pos4 = posx([601.44, 46.94, 94.2, 31.77, -179.71, 29.09])
    pos5 = posx([550.36, 47.54, 94.2, 17.4, -179.4, 14.45])
    pos6 = posx([499.48, 47.95, 94.2, 18.63, -179.21, 15.26])
    pos7 = posx([601.78, 98.37, 94.2, 19.94, -179.14, 16.37])
    pos8 = posx([550.48, 98.66, 94.2, 23.14, -179.23, 19.42])
    pos9 = posx([499.25, 99.09, 94.2, 20.67, -179.12, 16.86])    
    posa=[pos1,pos2,pos3,pos4,pos5,pos6,pos7,pos8,pos9]    
    posb1 = posx([600.63, -154.13, 94.2, 5.12, -178.91, 1.34])
    posb2 = posx([549.58, -153.99, 94.2, 176.65, 178.43, 173.07])
    posb3 = posx([498.57, -153.31, 94.2, 176.63, 178.56, 173.29])
    posb4 = posx([600.49, -102.33, 94.2, 33.84, -179.78, 31.26])
    posb5 = posx([549.73, -102.73, 94.2, 0.07, -178.47, -3.21])
    posb6 = posx([498.24, -103.54, 94.2, 173.96, -180.0, 173.39])
    posb7 = posx([600.5, -51.41, 94.2, 2.15, -178.0, -1.36])
    posb8 = posx([549.47, -51.78, 94.2, 1.07, -177.91, -2.37])
    posb9 = posx([498.39, -52.18, 94.2, 179.11, 177.79, 175.91])    
    posb=[posb1,posb2,posb3,posb4,posb5,posb6,posb7,posb8,posb9] 


    small=[]
    medium=[]
    large=[]

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    while rclpy.ok():        
        movej(JReady, vel=VELOCITY, acc=ACC)
        justGrip()  


        for i in range(0,9):
            movel(posa[i], vel=VELOCITY, acc=ACC)
            check(i)
            movel(posa[i], vel=VELOCITY, acc=ACC)

        combined= small + medium + large
        justRelease()

        for i in range(0,9):           
            movel(combined[i], vel=VELOCITY, acc=ACC)
            grip()
            movel(posb[i], vel=VELOCITY, acc=ACC)
            release()        
                
        rclpy.shutdown()

if __name__ == "__main__":
    main()