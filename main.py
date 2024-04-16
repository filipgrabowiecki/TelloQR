import threading
import time
from djitellopy import Tello
import csv
import matplotlib.pyplot as plt
import cv2
from pyzbar.pyzbar import decode
import numpy as np

global czy_hubert_to_debil
czy_hubert_to_debil = True
class TelloDrone():
    drone = Tello()
    start = time.time()
    global is_flying
    global has_landed
    global barcode_rect
    global drone_left
    global drone_right
    global drone_straight
    global drone_up
    global drone_down
    global drone_rotate_stop

    global roll_right
    global roll_left
    global roll_ok
    global barcode_polygon

    global height_14
    global height_23

    def __init__(self):
        self.drone.connect()
        self.is_flying = False
        self.has_landed = False
        self.barcode_rect = [0,0,0,0]
        self.drone_left = False
        self.drone_right = False
        self.drone_straight = False
        self.drone_up = False
        self.drone_down = False
        self.drone_rotate_stop = False

        self.height_14 = 0
        self.height_23 = 0

        self.roll_left = False
        self.roll_right = False
        self.roll_ok = False

    def killswitch(self):
        while not self.has_landed:
            if self.is_flying:
                input()
                self.drone.emergency()

    def battery_temp_check(self):
        print(f"\nBATTERY: {self.drone.get_battery()}")
        print(f"TEMPERATURE: {self.drone.get_temperature()}\n")

    def graphs(self):

        x = []
        pitch = []
        yaw = []
        roll = []

        plt.plot(x, pitch, label='pitch')
        plt.plot(x, yaw, label='yaw')
        plt.plot(x, roll, label='roll')

        cnt = 0
        is_alive = True

        while not self.has_landed:

            x.append(cnt)
            pitch.append(self.drone.get_pitch())
            yaw.append(self.drone.get_yaw())
            roll.append(self.drone.get_roll())

            plt.gca().lines[0].set_xdata(x)
            plt.gca().lines[0].set_ydata(pitch)
            plt.gca().relim()
            plt.gca().autoscale_view()
            plt.pause(0.1)

            plt.gca().lines[1].set_xdata(x)
            plt.gca().lines[1].set_ydata(yaw)
            plt.gca().relim()
            plt.gca().autoscale_view()
            plt.pause(0.1)

            plt.gca().lines[2].set_xdata(x)
            plt.gca().lines[2].set_ydata(roll)
            plt.gca().relim()
            plt.gca().autoscale_view()
            plt.pause(0.1)

            cnt += 1

            if cnt > 20:
                # cnt = 0
                pitch = pitch[1:]
                yaw = yaw[1:]
                roll = roll[1:]
                x = x[1:]
                # plt.pause(2.0)
                # is_alive = False

    def flight_data(self):
        while not self.has_landed:
            # print(self.is_flying)
            if self.is_flying == True:
                file = open(f"flight_data.csv", "w")
                r = csv.writer(file, delimiter=";")
                r.writerow(["BATTERY",'FLIGHT_TIME',"ROLL","PITCH","YAW","VEL_X","VEL_Y","VEL_Z","ACC_X","ACC_Y","ACC_Z"])
                while self.is_flying:
                    battery = self.drone.get_battery()
                    flight_time = self.drone.get_flight_time()
                    roll = self.drone.get_roll()
                    pitch = self.drone.get_pitch()
                    yaw = self.drone.get_yaw()
                    vel_x = self.drone.get_speed_x()
                    vel_y = self.drone.get_speed_y()
                    vel_z = self.drone.get_speed_z()
                    acc_x = self.drone.get_acceleration_x()
                    acc_y = self.drone.get_acceleration_y()
                    acc_z = self.drone.get_acceleration_z()
                    r.writerow([battery, flight_time, roll, pitch,yaw,vel_x,vel_y,vel_z,acc_x,acc_y,acc_z])
                    time.sleep(0.1)
                file.close()

    def show_camera(self):
        self.drone.streamon()

        while not self.has_landed:
            cap = self.drone.get_frame_read().frame
            cap = cv2.resize(cap, (1280, 720))
            cap = cv2.cvtColor(cap, cv2.COLOR_BGR2RGB)
            for barcode in decode(cap):
                self.barcode_rect = barcode.rect
                self.barcode_polygon = barcode.polygon


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
        self.drone.streamoff()

    def follow_qr(self):
        while not self.has_landed:
            #print(self.barcode_rect)
            if self.is_flying:
                # print("działa w chuj")
                left = self.barcode_rect[0]
                top = self.barcode_rect[1]
                width = self.barcode_rect[2]
                height = self.barcode_rect[3]
                width_dif = (1280-width)/2
                height_dif = (720-height)/2

                height_difference = self.height_14 - self.height_23
                #print(f"Height difference: {height_difference}")
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
            #print(self.barcode_rect)
            time.sleep(0.3)

    def rc_control(self, lr, fb, up, y):
        self.drone.send_rc_control(left_right_velocity=lr, forward_backward_velocity=fb,
                                   up_down_velocity=up, yaw_velocity=y)

    def takeoff_func(self):
        self.is_flying = True
        self.drone.takeoff()

    def landing_func(self):
        self.drone.land()
        time.sleep(3)
        self.is_flying = False
        self.has_landed = True

    def mission_func(self):
        self.takeoff_func()
        # self.is_flying = True
        # self.has_landed = False
        print(f"IS FLYING: {self.is_flying} ")
        print(f"HAS LANDED: {self.has_landed} ")
        time.sleep(5)
        cnt = 0
        while cnt < 250:
            #print(self.roll_left, self.roll_right, self.roll_ok)
            if self.barcode_rect == [0, 0, 0, 0]:
                self.rc_control(0,0,0,20)
                print("noqr")


            else:
                if self.roll_left == True:
                    self.rc_control(10, 0, 0, 0)
                    print("Turlaj sie w lewo")
                elif self.roll_right == True:
                    self.rc_control(-10, 0, 0, 0)
                    print("Turlaj sie w prawo")
                elif self.roll_ok == True:
                    print("Turlaj dropsa")

                    if self.drone_left == True:
                        self.rc_control(0,0,0,-10)
                        print("left")
                    elif self.drone_right == True:
                        print("right")
                        self.rc_control(0,0,0,10)
                    elif self.drone_rotate_stop == True:
                        print("straight")
                        if self.drone_up == True:
                            print("up")
                            self.rc_control(0,0,15,0)
                        elif self.drone_down == True:
                            print("Hubert")
                            self.rc_control(0,0,(-15),0)
                        elif self.drone_straight == True:
                            print("straight as an arrow")
                            width = self.barcode_rect[2]
                            height = self.barcode_rect[3]
                            if width > 250 or height > 210:
                                self.rc_control(0, 0, 0, 0)
                                print("Jesteśmy totalnie zajebiści ale chóbert nie")
                            else:
                                self.rc_control(0,20,0,0)




            time.sleep(0.2)
            cnt += 1
            #print(cnt)

        self.rc_control(0,0,0,0)
        time.sleep(5)
        self.landing_func()
        # self.is_flying = False
        # self.has_landed = True
        print(f"IS FLYING: {self.is_flying} ")
        print(f"HAS LANDED: {self.has_landed} ")
        self.drone.end()

def main():
    drone = TelloDrone()
    drone.battery_temp_check()
    threading.Thread(target=drone.killswitch).start()
    # threading.Thread(target=drone.graphs).start()
    threading.Thread(target=drone.show_camera).start()
    # threading.Thread(target=drone.flight_data).start()
    threading.Thread(target=drone.follow_qr).start()
    #threading.Thread (target=drone.mission_func).start()
    print("\nBateria po misji: \n")
    drone.battery_temp_check()

if __name__ == '__main__':
    main()
