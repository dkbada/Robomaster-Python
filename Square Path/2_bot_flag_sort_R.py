import cv2
import robomaster
from robomaster import robot
from robomaster import vision
import time

list_marker_info = RmList()
list_sort_list = RmList()
list_sight_list = RmList()
variable_index = 0
variable_yaw = 0
variable_y_coor = 0
variable_angle = 0
variable_trans1 = 0
variable_direction = 0
variable_trans2 = 0
variable_x_coor = 0
variable_num = 0
variable_count = 0
variable_back = 0
variable_v1 = 0
variable_v2 = 0
variable_turn = 0
variable_dist = 0
def user_defined_normal_arrange():
    variable_dist = 0.67
    chassis_ctrl.move_with_distance(-180,variable_dist)
    time.sleep(2)
    chassis_ctrl.move_with_distance(-90,variable_num)
    chassis_ctrl.move_with_distance(0,0.7)
    user_defined_grab()

    user_defined_angle()

    variable_dist = ir_distance_sensor_ctrl.get_distance_info(1)
    while not variable_dist >= 150:
        time.sleep(0.1)
        variable_dist = ir_distance_sensor_ctrl.get_distance_info(1)
    time.sleep(0.5)
    chassis_ctrl.move_with_distance(90,variable_num)
    user_defined_angle()

    robotic_arm_ctrl.moveto(197, -45, wait_for_complete=True)
    gripper_ctrl.open()
    time.sleep(1)
    user_defined_angle()

    chassis_ctrl.move_with_distance(-180,0.2)
    user_defined_get_close()

    user_defined_angle()

def user_defined_turned_arrange():
    chassis_ctrl.move_with_distance(-180,0.6)
    time.sleep(3)
    chassis_ctrl.stop()
    time.sleep(0.2)
    chassis_ctrl.set_wheel_speed(-40,-40,60,-100)
    time.sleep(1.6)
    user_defined_angle()

    variable_num = 1.65
    chassis_ctrl.move_with_distance(-180,0.3)
    chassis_ctrl.move_with_distance(-90,variable_num)
    chassis_ctrl.move_with_distance(0,0.2)
    user_defined_grab()

    user_defined_angle()

    time.sleep(1.8)
    variable_dist = ir_distance_sensor_ctrl.get_distance_info(1)
    while not variable_dist >= 100:
        time.sleep(0.1)
        variable_dist = ir_distance_sensor_ctrl.get_distance_info(1)
    chassis_ctrl.move_with_distance(90,0.5)
    chassis_ctrl.stop()
    time.sleep(0.2)
    chassis_ctrl.set_wheel_speed(-40,-40,-100,60)
    time.sleep(1.6)
    chassis_ctrl.stop()
    time.sleep(0.2)
    user_defined_angle()

    variable_num = 0.32
    chassis_ctrl.move_with_distance(90,variable_num)
    user_defined_angle()

    robotic_arm_ctrl.moveto(197, -45, wait_for_complete=True)
    gripper_ctrl.open()
    time.sleep(1)
    user_defined_angle()

    chassis_ctrl.move_with_distance(-180,0.2)
    user_defined_get_close()

    user_defined_angle()

    time.sleep(8.5)
def user_defined_compare():
    time.sleep(0.5)
    variable_count = variable_count + 1
    list_marker_info=RmList(vision_ctrl.get_marker_detection_info())
    list_sort_list[2] = list_marker_info[2]
    if list_sort_list[2] <= list_sort_list[1]:
        variable_count = 0
        if variable_turn == 0:
            user_defined_normal_arrange()
        else:
            user_defined_turned_arrange()

            variable_turn = 0
def user_defined_angle():
    time.sleep(0.1)
    variable_angle = chassis_ctrl.get_position_based_power_on(rm_define.chassis_rotate)
    if variable_angle > -50 and variable_angle < 0:
        chassis_ctrl.rotate_with_speed(rm_define.clockwise,abs(variable_angle))
        time.sleep(1)
    else:
        if variable_angle > 0 and variable_angle < 50:
            chassis_ctrl.rotate_with_speed(rm_define.anticlockwise,abs(variable_angle))
            time.sleep(1)
        else:
            if variable_angle > -90 and variable_angle < -50:
                chassis_ctrl.rotate_with_speed(rm_define.anticlockwise,90 - abs(variable_angle))
                time.sleep(1)
            else:
                if variable_angle > -140 and variable_angle < -90:
                    chassis_ctrl.rotate_with_speed(rm_define.clockwise,abs(variable_angle) - 90)
                    time.sleep(1)
                else:
                    if variable_angle > 50 and variable_angle < 90:
                        chassis_ctrl.rotate_with_speed(rm_define.clockwise,90 - abs(variable_angle))
                        time.sleep(1)
                    else:
                        if variable_angle > 90 and variable_angle < 140:
                            chassis_ctrl.rotate_with_speed(rm_define.anticlockwise,abs(variable_angle) - 90)
                            time.sleep(1)
    chassis_ctrl.stop()
    time.sleep(0.1)
    
def user_defined_get_close():
    time.sleep(0.1)
    list_marker_info=RmList(vision_ctrl.get_marker_detection_info())
    while not list_marker_info[4] >= 0.38:
        list_marker_info=RmList(vision_ctrl.get_marker_detection_info())
        chassis_ctrl.set_trans_speed(0.1)
        chassis_ctrl.move_with_time(0,0.1)
    chassis_ctrl.stop()
    chassis_ctrl.set_trans_speed(0.5)
    user_defined_angle()

def user_defined_center():
    variable_yaw = 96 * (list_marker_info[3] - list_sight_list[1])
    chassis_ctrl.move_with_speed(0,0,variable_yaw)
    time.sleep(1)
    
def user_defined_grab():
    time.sleep(0.1)
    list_marker_info=RmList(vision_ctrl.get_marker_detection_info())
    time.sleep(0.1)
    user_defined_center()

    while not list_marker_info[4] >= 0.53:
        list_marker_info=RmList(vision_ctrl.get_marker_detection_info())
        chassis_ctrl.set_trans_speed(0.1)
        chassis_ctrl.move_with_time(0,0.2)
        user_defined_center()

    gripper_ctrl.close()
    time.sleep(2)
    robotic_arm_ctrl.moveto(145, 62, wait_for_complete=True)
    time.sleep(0.5)
    chassis_ctrl.set_trans_speed(0.5)

def user_defined_path():
    time.sleep(1)
    list_marker_info=RmList(vision_ctrl.get_marker_detection_info())
    list_sort_list[1] = list_marker_info[2]
    time.sleep(3)
    variable_num = 0.55
    variable_turn = 0
    chassis_ctrl.move_with_distance(90,variable_num)
    user_defined_angle()

    time.sleep(0.3)
    list_marker_info=RmList(vision_ctrl.get_marker_detection_info())
    if len(list_marker_info) >= 6:
        variable_turn = 0
        user_defined_compare()
    else:
        variable_turn = variable_turn + 1
        user_defined_angle()

        chassis_ctrl.set_wheel_speed(-40,-40,-100,60)
        time.sleep(1.6)
        chassis_ctrl.stop()
        time.sleep(0.5)
        user_defined_angle()

        chassis_ctrl.move_with_distance(-180,0.1)
        chassis_ctrl.move_with_distance(90,0.5)
        user_defined_angle()

        if chassis_ctrl.get_position_based_power_on(rm_define.chassis_rotate) >= -30 and chassis_ctrl.get_position_based_power_on(rm_define.chassis_rotate) <= 30:
            time.sleep(1)
            list_sort_list[1] = 0
        else:
            list_marker_info=RmList(vision_ctrl.get_marker_detection_info())
            if len(list_marker_info) >= 6:
                user_defined_compare()

def start():
    led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 255, 0, 0, rm_define.effect_always_on)
    robotic_arm_ctrl.moveto(197, -45, wait_for_complete=True)
    gripper_ctrl.open()
    time.sleep(1)
    vision_ctrl.enable_detection(rm_define.vision_detection_marker)
    ir_distance_sensor_ctrl.enable_measure(1)
    list_sort_list=RmList()
    list_sort_list.append(0)
    list_sort_list.append(0)
    list_sight_list=RmList(media_ctrl.get_sight_bead_position())
    variable_turn = 0
    variable_count = 0
    variable_x_coor = chassis_ctrl.get_position_based_power_on(rm_define.chassis_forward)
    variable_y_coor = chassis_ctrl.get_position_based_power_on(rm_define.chassis_translation)
    variable_angle = chassis_ctrl.get_position_based_power_on(rm_define.chassis_rotate)
    while not variable_count >= 8:
        user_defined_path()
