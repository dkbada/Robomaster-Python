import time
import cv2
import math

import robomaster
from robomaster import robot

# import multiprocessing
import threading
from threading import Thread, Lock

def sub_data_handler(sub_info):
    global distance 
    distance = sub_info

class RobotDJI(robot.Robot):
    def __init__(self, name = None, *args, **kwargs):
        super().__init__(*args, **kwargs)
        """Initializes the data."""
        self.name = name
        print("Creating robot {}".format(self.name))

        self.number = -1
        
        self.isCamStreaming = False
        self.isCamRendering = False

        self.isMoving = False
        self.markerDetect = False
        self.searchComplete = False

        self.markers = []

    def startup(self, conn_type="sta", proto_type="udp", sn=None, vel_sub = False):
        self.initialize(conn_type=conn_type, proto_type=proto_type, sn=sn)
        if vel_sub:
            print("Signing up for velocity updates")
            self.chassis.sub_velocity(freq=5, callback=self.sub_velocity_handler)

    def on_detect_marker(self, marker_info):
        # print("the thread ID is ", self.thread.ident)
        self.markers.clear()
        number = len(marker_info)
        if (number>0):
            # print("marker detect callback function")
            for i in range(0, number):
                info = marker_info[i]
                self.markers.append((info,self.name))

    def start_CamStream(self, startRender = True, x0 = 50, y0 = 50):
        self.isCamStreaming = True
        self.camera.start_video_stream(display=False)
        if self.markerDetect:
            print("Turning on marker detection on {}".format(self.name))
            result = self.vision.sub_detect_info(name="marker", color="red", callback=self.on_detect_marker)
            print("with the result: {}".format(result))
        if startRender:
            self.read_lock = Lock()
            self.thread = Thread(target = self.start_CamRender, args=(x0, y0))
            self.thread.start()
            # print("the thread ID is ", self.thread.ident)

    def stop_CamStream(self):
        self.isCamStreaming = False
        result = self.vision.unsub_detect_info(name="marker")
        self.camera.stop_video_stream()

    def start_CamRender(self, x0 = 50, y0 = 50):
        self.isCamRendering = True
        cv2.namedWindow(self.name, cv2.WINDOW_NORMAL) 
        cv2.moveWindow(self.name, x0, y0)
        cv2.resizeWindow(self.name, 640, 360)

        while self.isCamRendering:
            # print("the thread ID is ", self.thread.ident)
            img = self.camera.read_cv2_image(strategy="newest", timeout=1.0)
            cv2.imshow(self.name, img)
            self.read_lock.acquire()
            if (self.markerDetect and (len(self.markers) > 0)):
                # print("marker was detected")
                # print(self.markers[0][0])
                self.searchComplete = True
            self.read_lock.release()
            if cv2.waitKey(1) == ord('q'):
                break
            time.sleep(0.01)

    def stop_CamRender(self):
        self.isCamRendering = False
        if self.thread.is_alive():
            # print(time.thread_time_ns())
            self.thread.join()
        cv2.destroyWindow(self.name)
    
    def marker_search(self):
        read_lock = Lock()
        read_lock.acquire()
        if (not self.isCamStreaming or not self.markerDetect):
            print("Make sure the camera is running")
            return
        print("Starting marker search")
        while (not self.searchComplete):
            # self.chassis.move(x=0.0, y=0.0, z=90, xy_speed=0, z_speed=30)
            #self.chassis.drive_speed(x = 0.0, y = 0, z = 30, timeout = 0.2)
            time.sleep(0.4)
        self.markerDetect = False
        print("Marker was found!")
        read_lock.release()
        self.play_audio(filename="examples/01_robot/" + "demo1.wav").wait_for_completed()
        self.number = int(self.markers[0][0][4])

    def marker_grab(self):
        self.gripper.open(power=50)
        time.sleep(2)
        self.robotic_arm.move(x=221, y=0).wait_for_completed()
        self.marker_search

        done = False
        while (done == False):
            #center bot with marker
            x_val = float(self.markers[0][0][0]) #the screen is from 0 - 1
           
            #change values
            yaw = 96 * ( 0.5 - x_val)
            height = float (self.markers[0][0][2])
            self.chassis.move(x=0, y=0, z=yaw).wait_for_completed()
            
            #move toward marker
            if (height < 0.16):
                self.chassis.move(x=0.1, y=0, z=0).wait_for_completed()
                time.sleep(0.1)
                self.chassis.drive_speed(x=0, y=0, z=0, timeout=5)
                time.sleep(0.5)
                
            #grab marker
            if (height > 0.17):
                self.gripper.close(power=50)
                time.sleep(2)
                done = True

    def sub_velocity_handler(self, velocity_info):
        sum_vel = abs(velocity_info[0]) + abs(velocity_info[1]) + abs(velocity_info[2])
        # print("sum of abs vel {}".format(sum_vel))
        if (sum_vel < 0.03):
            self.isMoving = False
        else:
            self.isMoving = True
            
    def base_stop(self):
        print("Stopping the base of robot {}".format(self.name))
        self.chassis.stop()

    def move_forward(self, amt):
        self.chassis.move(amt, 0, 0, 1, 180).wait_for_completed()
    
    def move_backward(self, amt):
        self.chassis.move(-amt, 0, 0, 1, 180).wait_for_completed()       

    def move_left(self, amt):
        self.chassis.move(0, -amt, 0, 1, 180).wait_for_completed()

    def move_right(self, amt):
        self.chassis.move(0, amt, 0, 1, 180).wait_for_completed()
    
    def rotate_right(self, amt):
        self.chassis.move(x=0, y=0, z=-amt).wait_for_completed()

    def rotate_left(self, amt):
        self.chassis.move(x=0, y=0, z=amt).wait_for_completed()

    def stop_moving(self):
        self.chassis.drive_speed(x=0, y=0, z=0, timeout=0.3)
        time.sleep(1)

    def align_to_box(self): # align to box using two IR sensors

        self.sensor.sub_distance(freq=5, callback=sub_data_handler)
        time.sleep(1)
        ir_l = distance[0]
        ir_r = distance[1]
        print(ir_l)
        print(ir_r)

        #while ir_l >= 265 or ir_r >= 265:
        while ir_l >= 330 and ir_r >= 330:
            if ir_l >= 330:
                self.chassis.drive_wheels(w1=23, w2=0, w3=0, w4=0)
                ir_l = distance[0]
                print(ir_l)
                time.sleep(0.6)
                self.chassis.drive_speed(x=0, y=0, z=0, timeout=5)
                time.sleep(0.1)
            if ir_r >= 330:
                self.chassis.drive_wheels(w1=0, w2=23, w3=0, w4=0)
                ir_r = distance[1]
                print(ir_r)
                time.sleep(0.6)
                self.chassis.drive_speed(x=0, y=0, z=0, timeout=5)
                time.sleep(0.1)

        self.sensor.unsub_distance()

        self.stop_moving()

    def move_to_goal(self): # rotate around until goal identified, move toward it
        height = 0
        r = 0.4 # box radius
        x = 0.3 # robot length
        #theta = 20
        theta = 30
        theta_rad = theta * (math.pi / 180) # rotation amt 
        d2 = (r+x+0.2) * math.tan(theta_rad) - 0.1
        d3 = (((r+x+0.2) / math.cos(theta_rad)) - (r+x+0.2)) + 0.1

        #while height < 0.5: #change value as needed; repeat until goal height is greater than 0.5
        done = False
        time.sleep(0.1)
        # no goal detected, do counter-clockwise around box routine
        while (done == False):
            self.move_backward(0.2)
            self.move_right(d2)
            self.rotate_left(theta)
            self.move_forward(d3)
            self.align_to_box()
            #print("waiting...")
            time.sleep(2)
            if self.searchComplete == True:
                self.markerDetect = True
                print("marker detected!")
                print(self.markers)
                # goal detected, push box until close enough to goal
                self.move_forward(0.2)
                self.stop_moving()
                #print(self.markerDetect)
                if len(self.markers) > 2:
                    height = float (self.markers[0][0][2])
                    print(height)
                time.sleep(0.5)
            #self.markerDetect = False
            self.searchComplete = False
            if height > 0.3:
                done = True
    
    
    def go_to_box(self): # identifies box and move toward it
        self.robotic_arm.move(x=-30, y=10).wait_for_completed()
        self.gripper.open(power=50)
        self.marker_search
        time.sleep(0.2)
        
        height = 0
        time.sleep(1)

        # rotate around until marker detected
        read_lock = Lock()
        read_lock.acquire()
        while (not self.searchComplete):
            #self.chassis.move(x=0.0, y=0.0, z=90, xy_speed=0, z_speed=30)
            self.chassis.drive_speed(x = 0.0, y = 0, z = 30, timeout = 0.2)
            time.sleep(0.4)
        self.markerDetect = False
        print("Marker was found!")
        read_lock.release()

        # align to marker and move forward until close enough
        while height < 0.13:
            if len(self.markers[0][0]) >= 5:
                x_val = float(self.markers[0][0][0])
                height = float (self.markers[0][0][2])
                print(height)
                yaw = 96 * ( 0.5 - x_val)
                yaw = 150 * (x_val-0.5)
                vel = 2*(0.34 - height)
                self.chassis.drive_speed(x=vel, y=0, z=yaw)
            else:
                self.chassis.drive_speed(x=0, y=0, z=0, timeout=0.3)
            time.sleep(0.1)
        self.stop_moving()

        # point camera toward goal angle
        #self.align_to_box
        self.robotic_arm.move(x=144, y=100).wait_for_completed()
        self.robotic_arm.move(x=-70, y=100).wait_for_completed()
        self.stop_moving()
        self.markerDetect = False
        #self.searchComplete = False


if __name__ == '__main__':
    
    print("We are starting")
    robomaster1 = RobotDJI("robot1")
    robomaster2 = RobotDJI("robot2")

    print("Trying to connect")
    robomaster1.startup(conn_type="sta", sn="3JKCH8800100XU") #019
    robomaster2.startup(conn_type="sta", sn="3JKCH8800100TQ") #016

    robomaster1.play_sound(robot.SOUND_ID_ATTACK).wait_for_completed()
    robomaster2.play_sound(robot.SOUND_ID_RECOGNIZED).wait_for_completed()

    robomaster1.markerDetect = True
    robomaster2.markerDetect = True

    robomaster1.start_CamStream(startRender = True, x0 = 50, y0 = 50)
    robomaster2.start_CamStream(startRender = True, x0 = 700, y0 = 50)
    
    #detect marker and stream
    time.sleep(3)
    t1 = Thread(target = robomaster1.go_to_box, args=())
    t1.start()
    t2 = Thread(target = robomaster2.go_to_box, args=())
    t2.start()
    t1.join()
    t2.join()

    t1 = Thread(target = robomaster1.align_to_box, args=())
    t1.start()
    t2 = Thread(target = robomaster2.align_to_box, args=())
    t2.start()
    t1.join()
    t2.join()

    time.sleep(4)

    t1 = Thread(target = robomaster1.move_to_goal, args=())
    t1.start()
    t2 = Thread(target = robomaster2.move_to_goal, args=())
    t2.start()
    t1.join()
    t2.join()

    robomaster1.stop_CamRender()
    robomaster2.stop_CamRender()

    robomaster1.stop_CamStream()
    robomaster2.stop_CamStream()

    robomaster1.close()
    robomaster2.close()
