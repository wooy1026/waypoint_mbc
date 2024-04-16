#!/usr/bin/env python3
# -*- coding:utf-8 -*-  
import rospy
import math
import numpy as np 
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import pandas as pd



class move_limo:
    def __init__(self):
        rospy.init_node('control', anonymous=True)
        self.manual = 0
        self.auto = 0
        self.speed = 0
        self.steering = 0
        self.angle = 0
        self.yaw = 0
        self.utm_x = 0.0
        self.utm_y = 0.0
        self.lx = []  # 또는 기본 웨이포인트 값으로 초기화
        self.ly = []  # 또는 기본 웨이포인트 값으로 초기화

        self.drive_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        rospy.Subscriber("/heading_topic", Float64, self.heading_callback)
        rospy.Subscriber("/utm_x_topic", Float64, self.utm_x_callback)
        rospy.Subscriber("/utm_y_topic", Float64, self.utm_y_callback)
        rospy.Subscriber("/joy", Joy, self.send_joy)
        rospy.Timer(rospy.Duration(0.1), self.drive_control)


        file_path = '/home/wooyeong/csv_folder/waypoint_list/way_dir_h.csv'
        reading_csv = pd.read_csv(file_path)
        self.lx = reading_csv.iloc[:, 0].tolist() #first row
        self.ly = reading_csv.iloc[:, 1].tolist() #second row


    def utm_x_callback(self, data):
        self.utm_x = data.data

    def utm_y_callback(self, data):
        self.utm_y = data.data 

    def send_joy(self, data):
        self.joy_btn = data.buttons
        self.joy_axes = data.axes
        drive = Twist()
		
        self.auto = self.joy_btn[4]
        self.manual = self.joy_btn[5]
        
    def heading_callback(self, data): #헤딩 좌표계를 UTM 좌표계로 변환
        self.yaw_raw = data.data
        if self.yaw_raw > 0 and self.yaw_raw < 90:
            self.yaw = 90 - self.yaw_raw
        else:
            self.yaw = 450 - self.yaw_raw
        
    def stanley_control_angle(self): #스탠리 컨트롤
        min_dist = float('inf')
        target_i = 0

        for i in range(len(self.lx)):
            dx = self.lx[i] - self.utm_x
            dy = self.ly[i] - self.utm_y
            dist = math.sqrt(dx ** 2 + dy ** 2)

            if dist < min_dist:
                min_dist = dist
                target_i = i
        #print('dx:',dx)
        #print('dy:',dy)
        # print('utm_x:',self.utm_x)
        # print('utm_y:',self.utm_y)
        print('min_dist:',min_dist)

        #경로 끝남
        if target_i+1 >= len(self.lx):
            print("finish, idx= ", target_i)
            return None
        
        #경로 진행중 => map_yaw계산
        else:
            map_yaw = math.atan2(self.ly[target_i+1] - self.ly[target_i],self.lx[target_i+1] - self.lx[target_i]) * 180 / math.pi

            #atan 값의 범위(-180에서 180)을 0에서 360으로 변환
            if map_yaw < 0:
                map_yaw = map_yaw + 360

			#에러각 산출
            cte = map_yaw - self.yaw
            if cte < -180:
                cte = cte + 360
            elif cte > 180:
                cte = cte - 360

            Kp = 0.028 # 게인값 조정필요


###############################################################################

            ##경로의 왼쪽인지 오른쪽인지 판별파트 (min_dist)###
            angle = (math.atan2(self.ly[target_i] - self.utm_y,self.lx[target_i] - self.utm_x))*180/math.pi

            if angle < 0:
                angle = angle + 360

            if min_dist > 0.01:
                if map_yaw - angle > 0:
                    min_dist = -min_dist
                else:
                    min_dist = min_dist
            else:
                min_dist = 0
            # dir = 0
            # if min_dist > 0.01:
            #     dir = np.dot([self.lx[target_i+1]-self.lx[target_i],self.ly[target_i+1]-self.ly[target_i]],[self.utm_y-self.ly[target_i],(self.utm_x-self.lx[target_i])])
            # else:
            #     min_dist = 0
                
            # if dir < 0:
            #     min_dist = -min_dist
            


            #최종 스탠리 앵글
            stanley_steer_angle = cte + math.atan2(Kp * min_dist, 0.1) * 180 / math.pi
            
            #확인용
            print("%%%%%%%%%%%%%%%%%%%%%%%")
            print('map_yaw:',map_yaw,'self.yaw:',self.yaw)
            #print('min_dist:',min_dist)
            print('idx:',target_i) 
            print('cte:',cte)
            print("dir: ",dir)
            print('math.atan2:',math.atan2(Kp * min_dist, 0.1) * 180 / math.pi)
            print("steer_angle : ", stanley_steer_angle)
            #print(target  - self.actual_speed_m_s)

            return stanley_steer_angle

    def drive_control(self, event):
        drive = Twist()
        if self.manual == 1:
            self.speed = self.joy_axes[4]
            self.steering = self.joy_axes[0]
            drive.linear.x = self.speed*100
            drive.angular.z = int(self.steering * 40)

            
        elif self.auto == 1:
            steer_angle = self.stanley_control_angle()
            if steer_angle is None: # 큰경우 정지
                drive.linear.x = 0
                drive.angular.z = 0
                #print(1)
            else:
                drive.linear.x = 60
                drive.angular.z = steer_angle
                #print('steer_angle:',steer_angle) 

        else:
            drive.linear.x = 0
            drive.angular.z = 0
        self.drive_pub.publish(drive)

if __name__ == '__main__':
    try:
        MoveCar = move_limo()
        rospy.spin()
    except KeyboardInterrupt:
        print("Program terminated")
