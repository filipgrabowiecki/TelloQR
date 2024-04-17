from threading import Thread, Event
import time
from djitellopy import Tello
import csv
import matplotlib.pyplot as plt
import cv2
from pyzbar.pyzbar import decode
import numpy as np
import keyboard
import os

class TelloDrone:

    class TelloKillSwitch(Thread):

        tc_handler = None

        def __init__(self, tc_handler):
            Thread.__init__(self)
            self.tc_handler = tc_handler

        def run(self):
            keyboard.wait('space')
            self.tc_handler.force_emergency_stop()


    class Threading(Thread):
        interval = 1.0
        running = None
        func = None

        def __init__(self, interval, event, func):
            Thread.__init__(self)
            self.running = event
            self.interval = interval
            self.func = func

        def run(self):
            while not self.running.wait(self.interval):
                self.func()


    drone = None
    stop_controller = None

    def force_emergency_stop(self):
        self.drone.emergency()
        self.stop_controller.set()

    def battery_temp_check(self):
        print(f"\nBATTERY: {self.drone.get_battery()}")
        print(f"TEMPERATURE: {self.drone.get_temperature()}\n")

    def show_camera(self):
        cap = self.drone.get_frame_read().frame
        cap = cv2.resize(cap, (1280, 720))
        cap = cv2.cvtColor(cap, cv2.COLOR_BGR2RGB)
        for barcode in decode(cap):
            self.barcode_rect = barcode.rect
            self.barcode_polygon = barcode.polygon
            self.barcode_data = barcode.data.decode('utf-8')
            self.height_14 = (barcode.polygon[0][1] - barcode.polygon[3][1])
            self.height_23 = (barcode.polygon[1][1] - barcode.polygon[2][1])

            # print(f"Height 1-4: {self.height_14}")
            # print(f"Height 2-3: {self.height_23}")
            # print(f"Height difference: {self.height_14 - self.height_23}")

            pts = np.array([barcode.polygon], np.int32)
            pts = pts.reshape((-1,1,2))
            cv2.polylines(cap,[pts],True,(255,0,255),5)
            # print(barcode.polygon)
        cv2.imshow('frame', cap)
        cv2.waitKey(1)

    def follow_qr(self):

        if self.barcode_data == 'right':
            self.right_90 = True
            self.left_90 = False
        elif self.barcode_data == 'left':
            self.left_90 = True
            self.right_90 = False
        # print("działa w chuj")
        left = self.barcode_rect[0]
        top = self.barcode_rect[1]
        width = self.barcode_rect[2]
        height = self.barcode_rect[3]
        width_dif = (1280 - width) / 2
        height_dif = (720 - height) / 2

        height_difference = self.height_14 - self.height_23
        # print(f"Height difference: {height_difference}")
        if height_difference < -6:
            self.roll_left = True
            self.roll_right = False
            self.roll_ok = False
            print("Turlaj w lewo")
        elif height_difference > 6:
            self.roll_left = False
            self.roll_right = True
            self.roll_ok = False
            print("Turlaj w prawo")
        elif height_difference > -6 and height_difference < 6:
            self.roll_left = False
            self.roll_right = False
            self.roll_ok = True
            print("Roll ok")

        # print(self.barcode_rect)
        if left != 0:
            if left > (width_dif + 50 + width):
                # print("DRONE RIGHT")
                self.drone_left = False
                self.drone_right = True
                self.drone_rotate_stop = False
                self.drone_up = False
                self.drone_down = False
            elif left < width_dif - 50:
                # print("DRONE LEFT")
                self.drone_left = True
                self.drone_right = False
                self.drone_rotate_stop = False
                self.drone_up = False
                self.drone_down = False
            elif left > width_dif - 50 and left < (width_dif + 50 + width):
                # print("DRONE ROTATION STOP")
                self.drone_left = False
                self.drone_right = False
                self.drone_rotate_stop = True

        if top != 0:
            if top > (height_dif + 100 + height):
                self.drone_down = True
                self.drone_up = False
                self.drone_straight = False
            if top < height_dif - 100:
                self.drone_down = False
                self.drone_up = True
                self.drone_straight = False
            if top > (height_dif - 100) and top < (height_dif + 100 + height):
                self.drone_down = False
                self.drone_up = False
                self.drone_straight = True

    def rc_control(self, lr, fb, up, y):
        self.drone.send_rc_control(left_right_velocity=lr, forward_backward_velocity=fb,
                                   up_down_velocity=up, yaw_velocity=y)
    def mission_func(self):
        self.drone.takeoff()
        self.rc_control(0,0,0,0)
        time.sleep(5)
        cnt = 0

        while cnt < 450:
            # print(self.roll_left, self.roll_right, self.roll_ok)
            if self.barcode_rect == [0, 0, 0, 0]:
                self.rc_control(0, 0, 0, 20)
                print("noqr")

            else:
                if self.drone_left == True:
                    self.rc_control(0, 0, 0, -10)
                    print("left")
                elif self.drone_right == True:
                    print("right")
                    self.rc_control(0, 0, 0, 10)
                elif self.drone_rotate_stop == True:
                    print("straight")
                    if self.roll_left == True:
                        self.rc_control(-15, 0, 0, 0)
                        print("Turlaj sie w lewo")
                    elif self.roll_right == True:
                        self.rc_control(15, 0, 0, 0)
                        print("Turlaj sie w prawo")
                    elif self.roll_ok == True:
                        print("Turlaj dropsa")
                        if self.drone_up == True:
                            print("up")
                            self.rc_control(0, 0, 15, 0)
                        elif self.drone_down == True:
                            print("Hubert")
                            self.rc_control(0, 0, (-15), 0)
                        elif self.drone_straight == True:
                            print("straight as an arrow")
                            width = self.barcode_rect[2]
                            height = self.barcode_rect[3]
                            if width > 250 or height > 210:
                                self.rc_control(0, 0, 0, 0)
                                print("Jesteśmy totalnie zajebiści ale chóbert nie")
                                print(self.barcode_data)
                                if self.left_90 == True:
                                    self.drone.rotate_counter_clockwise(90)
                                    print("90 left")
                                    self.left_90 = False
                                elif self.right_90 == True:
                                    self.drone.rotate_clockwise(90)
                                    print("90 right")
                                    self.right_90 = False
                            else:
                                self.rc_control(0, 20, 0, 0)

            time.sleep(0.2)
            cnt += 1
            # print(cnt)

        self.rc_control(0, 0, 0, 0)
        time.sleep(5)
        self.drone.land()

    def main(self):

        #KILLSWITCH
        self.kill_switch = self.TelloKillSwitch(self)
        self.kill_switch.start()
        self.stop_controller = Event()

        #START TELLO
        self.drone = Tello()
        self.drone.connect()

        #CAMERA
        self.drone.streamon()
        camera_show = self.Threading(0.001, self.stop_controller, self.show_camera)
        camera_show.start()

        #BATTERY
        battery_check = self.Threading(25, self.stop_controller, self.battery_temp_check)
        battery_check.start()

        #FOLLOWQR
        follow_qr = self.Threading(0.3, self.stop_controller, self.follow_qr)
        follow_qr.start()






    def __init__(self):
        self.is_flying = False
        self.has_landed = False
        self.barcode_rect = [0,0,0,0]
        self.drone_left = False
        self.drone_right = False
        self.drone_straight = False
        self.drone_up = False
        self.drone_down = False
        self.drone_rotate_stop = False
        self.barcode_data = ''
        self.height_14 = 0
        self.height_23 = 0
        self.right_90 = False
        self.left_90 = False
        self.roll_left = False
        self.roll_right = False
        self.roll_ok = False

        self.main()
        self.mission_func()

        self.drone.streamoff()
        self.drone.end()

if _name_ == "_main_":
    td = TelloDrone()