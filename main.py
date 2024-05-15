from threading import Thread, Event
from ultralytics import YOLO
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

    def attitude_indicator(self):
        image = np.zeros((400, 750, 3), dtype=np.uint8)
        pitch = self.drone.get_pitch()
        y1 = int(round((-5.6 * pitch + 200), 0))
        roll = self.drone.get_roll()
        y2 = int(round((8.6 * roll + 375), 0))
        angle = self.drone.get_roll()
        yaw = self.drone.get_yaw()
        speed = round((((self.drone.get_speed_x()) ** (2)) + ((self.drone.get_speed_y()) ** (2))) ** (1 / 2), 0)
        alt = self.drone.get_height()
        vs = self.drone.get_speed_z()
        bat = self.drone.get_battery()

        # yaw = 20
        # speed = 35
        # alt = 3.4
        # vs = 20
        # bat = 15

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness = 2

        yaw_text_pos = (10, 30)
        yaw_text = f'YAW: {yaw}'

        speed_text_pos = (10, 197)
        speed_text = f'SPEED: {speed}'

        alt_text_pos = (660, 197)
        alt_text = f'ALT: {alt}'

        vs_text_pos = (660, 167)
        vs_text = f'VS: {vs}'

        bat_text_pos = (620, 30)
        bat_text = f'BATTERY: {bat}'

        brown = (5, 129, 223)
        blue = (255, 229, 50)
        yellow = (0, 255, 255)
        white = (255, 255, 255)
        purple = (255, 0, 255)
        red = (0, 0, 255)

        # SKY/GROUND
        cv2.rectangle(image, (0, 0), (750, 200), blue, -1)
        cv2.rectangle(image, (0, 200), (750, 400), brown, -1)

        # PITCH/UP
        cv2.rectangle(image, (275, 171), (475, 173), white, -1)
        cv2.rectangle(image, (225, 143), (525, 145), white, -1)
        cv2.rectangle(image, (275, 115), (475, 117), white, -1)
        cv2.rectangle(image, (225, 87), (525, 89), white, -1)
        cv2.rectangle(image, (275, 59), (475, 61), white, -1)

        # PITCH/DOWN
        cv2.rectangle(image, (275, 227), (475, 229), white, -1)
        cv2.rectangle(image, (225, 255), (525, 257), white, -1)
        cv2.rectangle(image, (275, 283), (475, 285), white, -1)
        cv2.rectangle(image, (225, 311), (525, 313), white, -1)
        cv2.rectangle(image, (275, 339), (475, 341), white, -1)

        # ROLL/MID
        cv2.rectangle(image, (374, 190), (376, 210), white, -1)

        # ROLL/LEFT
        cv2.rectangle(image, (331, 190), (333, 210), white, -1)
        cv2.rectangle(image, (288, 190), (290, 210), white, -1)
        cv2.rectangle(image, (245, 190), (247, 210), white, -1)
        cv2.rectangle(image, (202, 190), (204, 210), white, -1)
        cv2.rectangle(image, (159, 190), (161, 210), white, -1)

        # ROLL/RIGHT
        cv2.rectangle(image, (417, 190), (419, 210), white, -1)
        cv2.rectangle(image, (460, 190), (462, 210), white, -1)
        cv2.rectangle(image, (503, 190), (505, 210), white, -1)
        cv2.rectangle(image, (546, 190), (548, 210), white, -1)
        cv2.rectangle(image, (589, 190), (591, 210), white, -1)

        cv2.rectangle(image, (300, y1 - 5), (450, y1 + 5), yellow, -1)
        cv2.rectangle(image, (y2 - 5, 125), (y2 + 5, 275), yellow, -1)

        # TEXT/YAW
        cv2.putText(image, yaw_text, yaw_text_pos, font, font_scale, purple, thickness)

        # SPEED/TEXT
        cv2.putText(image, speed_text, speed_text_pos, font, font_scale, purple, thickness)

        # ALT/TEXT
        cv2.putText(image, alt_text, alt_text_pos, font, font_scale, purple, thickness)

        # VS/TEXT
        cv2.putText(image, vs_text, vs_text_pos, font, font_scale, purple, thickness)

        # BAT/TEXT
        if bat > 20:
            cv2.putText(image, bat_text, bat_text_pos, font, font_scale, purple, thickness)
        else:
            cv2.putText(image, bat_text, bat_text_pos, font, font_scale, red, thickness)

        cv2.imshow('attitude indicator', image)
        cv2.waitKey(1)

    def object_detection(self):
        img = self.drone.get_frame_read().frame
        img = cv2.resize(img, (1280, 720))
        results = self.model(img)

        list_of_objects = []
        for i in results:
            list_of_objects.append(i.boxes.cls)

        values_list_float = list_of_objects[0].tolist()
        object_values_list = []
        for i in values_list_float:
            object_values_list.append(int(i))

        xyxy = []

        for i in results:
            xyxy.append(i.boxes.xyxy)
        list_of_xyxy_float = xyxy[0].tolist()
        list_of_xyxy = []

        for i in list_of_xyxy_float:
            new_list = []
            for j in i:
                new_list.append(int(j))
            list_of_xyxy.append(new_list)
        # print(f"list_of_xyxy: {list_of_xyxy}")

        person_indx = -1

        for i in object_values_list:
            if i == 39:
                person_indx = object_values_list.index(i)

        if person_indx != -1:
            person_xyxy = list_of_xyxy[person_indx]
            # print(person_xyxy)
            left = person_xyxy[0]
            top = person_xyxy[1]
            width = person_xyxy[2] - person_xyxy[0]
            height = person_xyxy[3] - person_xyxy[1]
            self.barcode_rect = [left, top, width, height]
            print(f"top: {top} left: {left} width: {width} height: {height}")

        image_ready = results[0].plot()
        cv2.imshow('image', image_ready)
        cv2.waitKey(1)

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

        # if self.barcode_data == 'right':
        #     self.right_90 = True
        #     self.left_90 = False
        # elif self.barcode_data == 'left':
        #     self.left_90 = True
        #     self.right_90 = False
        # print("działa w chuj")
        print(f"barcode_rect: {self.barcode_rect}")
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
        print(cnt)

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
                    # if self.roll_left == True:
                    #     self.rc_control(-15, 0, 0, 0)
                    #     print("Turlaj sie w lewo")
                    # elif self.roll_right == True:
                    #     self.rc_control(15, 0, 0, 0)
                    #     print("Turlaj sie w prawo")
                    # elif self.roll_ok == True:
                    #     print("Turlaj dropsa")
                    #     if self.drone_up == True:
                    #         print("up")
                    #         self.rc_control(0, 0, 15, 0)
                    #     elif self.drone_down == True:
                    #         print("Hubert")
                    #         self.rc_control(0, 0, (-15), 0)
                    #     elif self.drone_straight == True:
                    #         print("straight as an arrow")
                    #         width = self.barcode_rect[2]
                    #         height = self.barcode_rect[3]
                    #         if width > 250 or height > 210:
                    #             self.drone.land()

                                # self.rc_control(0, 0, 0, 0)
                                # print("Jesteśmy totalnie zajebiści ale chóbert nie")
                                # print(self.barcode_data)
                                # if self.left_90 == True:
                                #     self.drone.rotate_counter_clockwise(90)
                                #     print("90 left")
                                #     self.left_90 = False
                                # elif self.right_90 == True:
                                #     self.drone.rotate_clockwise(90)
                                #     print("90 right")
                                #     self.right_90 = False
                            # else:
                            #     self.rc_control(0, 20, 0, 0)

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
        # camera_show = self.Threading(0.001, self.stop_controller, self.show_camera)
        # camera_show.start()
        object_detection = self.Threading(0.001, self.stop_controller, self.object_detection)
        object_detection.start()

        #ATTITUDE INDICATOR
        attitude_indicator = self.Threading(0.001, self.stop_controller, self.attitude_indicator)
        attitude_indicator.start()

        #BATTERY
        battery_check = self.Threading(25, self.stop_controller, self.battery_temp_check)
        battery_check.start()

        #FOLLOWQR
        follow_qr = self.Threading(0.3, self.stop_controller, self.follow_qr)
        follow_qr.start()

        attitude_indicator = self.Threading(0.001, self.stop_controller, self.attitude_indicator)
        attitude_indicator.start()





    def __init__(self):
        self.model = YOLO('yolov8n.pt')
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
        time.sleep(20)
        self.drone.streamoff()
        self.drone.end()

if __name__ == "__main__":
    td = TelloDrone()
