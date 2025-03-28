# cup ver 1.0

# 소스코드
# 웹로직그리퍼
# 계획서
# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 800, 800

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("dsr_example_demo_py", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            set_tool,
            set_tcp,
            movej,
            movel,
            amovel,
            get_current_posx,
            set_digital_output,
            wait,
            release_force,
            align_axis,
            parallel_axis,
            DR_FC_MOD_REL,
            DR_FC_MOD_ABS,
            DR_AXIS_Z,
            DR_BASE,
            DR_MV_MOD_ABS,
            DR_MV_MOD_REL,
        )

        from DR_common2 import posx

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    def grip():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)

    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        
    def compliance(force):
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, -1, 0, 0, 0], mod=DR_FC_MOD_ABS)
        
        while not check_force_condition(DR_AXIS_Z, max=force):
            pass

        release_compliance_ctrl()

    #pos_home = posx([189.97, -73.98, 250.48, 106.71, 179.97, 107.21,])  
    pos_home = posx([394.059, -95.22, 249.754, 171.143, 178.525, 171.697])
    pos_final=[]

    set_tool("Tool Weight_D_5")
    set_tcp("GripperDA_v155")

    def pick_compliance(): #순응제어, 열기, 잡기
        compliance(10)
        
        movel(posx([0, 0, 10, 0, 0, 0]), vel=VELOCITY, acc=ACC,ref=DR_BASE, mod=DR_MV_MOD_REL)
        release()
        wait(1.0)
        movel(posx([0, 0, -22, 0, 0, 0]),vel=VELOCITY,acc=ACC,ref=DR_BASE,mod=DR_MV_MOD_REL,)

        grip()
        wait(1.5)

    def put_compliance(i,height_x,step_x,step_y): #좌표계산, 이동, 순응제어, 놓기
        #놓아야하는 좌표를 정해준다
        move_x = 180 + 67.5 * height_x + 39*step_x
        move_y = 39*step_y
        xy_move = posx([move_x,move_y,0,0,0,0])
        #내린거 다시 올리기
        pos_z = posx([0, 0, -225 + 12*i,0,0,0])

        #이동위치
        movel(xy_move, vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        #z정렬
        parallel_axis([0,0,-1],DR_AXIS_Z,DR_AXIS_Z)
        movel(pos_z, vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)

        compliance(5)
        release() 

    def movement_repeat(i,height_x,step_x,step_y,floor_z):
        
        movel(pos_home, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        grip()
        wait(0.5)
        #컵을 하나 없애면 해당 높이만큼 탑이 낮아지기 때문에 더 하강하여 순응제어
        movel(posx([0,0,-11*i,0,0,0]), vel=VELOCITY, acc=ACC, ref=DR_BASE, mod = DR_MV_MOD_REL)

        pick_compliance()
        go_up_after_pick = posx([0,0,120 + 100*floor_z,0,0,0])
        movel(go_up_after_pick, vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)

        put_compliance(i, height_x,step_x,step_y)
        go_up_after_put = posx([0,0,120,0,0,0])
        movel(go_up_after_put, vel=VELOCITY, acc=ACC, ref=DR_BASE, mod = DR_MV_MOD_REL)

    def flip_and_drop():
        
        movel(pos_home, vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_ABS)

        release()
        wait(0.5)
        movel(
            posx([370, 165.74, 211.3, 172.91, -178.07, 174.46]), 
            vel=VELOCITY,
            acc=ACC,
            mod=DR_MV_MOD_ABS,
        )  # 몸통 돌리기
        movel(
            posx([389.8, -111.43, 90, 87.13, -90.02, 90.35]),
            vel=VELOCITY,
            acc=ACC,
            mod=DR_MV_MOD_ABS,
        )  # 그립 직전 위치
        
        grip()
        wait(1)
        movel(
            posx([369.91, 165.74, 209.3, 172.91, -178.07, 174.46]),
            vel=VELOCITY,
            acc=ACC,
            mod=DR_MV_MOD_ABS,
        )  # 몸통 돌리기
        movej([0, 0, 0, 0, 0, 0], vel=60, acc=60)
        movej([0.51, -39.16, 133.51, 1.21, -4.69, 88.30], vel=60, acc=60)
        movej([-48.70, 10.22, 121.52, -92.34, -108.56, 140.11], vel=60, acc=60)
        movej([-49.45, 5.46, 116.69, -97.76, -104.82, 146.79], vel=60, acc=60) 
        movel(
            posx([589.47, -88.8, 324.89, 56.68, 88.57, 89.73]),
            vel=VELOCITY,
            acc=ACC,
            mod=DR_MV_MOD_ABS,
        )
        print(f"pos_final : {pos_final}")
        compliance(5)
        wait(0.5)
        release()
        wait(0.5)
        movel(
        posx([0, 0, 50, 0, 0, 0]),
        vel=VELOCITY,
        acc=ACC,
        ref=DR_BASE,
        mod=DR_MV_MOD_REL,
        )
        wait(1.0)

    def coordinates(i):
        coordinate_list = [(i,0,0,0,0),
                           (i,0,0,2,0),
                           (i,0,0,4,0),
                           (i,1,0,1,0),
                           (i,1,0,3,0),
                           (i,2,0,2,0),
                           (i,0.33,0,1,1),
                           (i,0.33,0,3,1),
                           (i,1.33,0,2,1),
                           (i,0.66,0,2,2)
                           ]
        return coordinate_list[i-1]

    if rclpy.ok():

        for i in range(1,11,1):
            height_x, step_x, step_y, floor_z = coordinates(i)[1:]  # i 제외하고 언패킹
            movement_repeat(i, height_x, step_x, step_y, floor_z)
        
        gcp, sol = get_current_posx()
        for k in gcp:
            pos_final.append(k)

        print(f"gcp : {gcp}")
        flip_and_drop()

    rclpy.shutdown()


if __name__ == "__main__":
    main()