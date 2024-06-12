#!/usr/bin/env python3
# -*- coding:utf-8 -*-  
import rospy
import math
import pandas as pd
import numpy as np
import requests
import json
from PyKakao import Message
from shapely.geometry import Point, Polygon
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
import time

# import numpy as np ..벡터계산 미사용(주석처리)

class move_limo:
    def __init__(self):
        rospy.init_node('control', anonymous=True)
        self.manual = 0
        self.auto = 0
        self.speed = 0
        self.steering = 0
        self.yaw_raw = 0 #gps_mbc기준 자신의 헤딩값 (N 0도,시계반대방향 0~360)
        self.yaw = 0 #utm_pos기준 자신의 헤딩값 (E 0도,시계반대방향 0~360)
        self.utm_x_raw = 0 #실제 자신의 utm좌표_x
        self.utm_y_raw = 0 #실제 자신의 utm좌표_y
        self.utm_x = 0.0 #offset한 자신의 utm좌표_x
        self.utm_y = 0.0 #offset한 자신의 utm좌표_y
        self.lx = []  # 또는 기본 웨이포인트 값으로 초기화
        self.ly = []  # 또는 기본 웨이포인트 값으로 초기화
        self.return_flag = False
        self.start = []

        #obstacle_init
        self.dist_obstacle = 3
        self.x_list = []
        self.y_list = []
        self.last_alarm_time = 0  # Timestamp of the last alarm

        #토픽 구독
        rospy.Subscriber("/heading_topic", Float64, self.heading_callback)
        rospy.Subscriber("/utm_x_topic", Float64, self.utm_x_callback)
        rospy.Subscriber("/utm_y_topic", Float64, self.utm_y_callback)
        rospy.Subscriber("/joy", Joy, self.send_joy)
        rospy.Subscriber("scan", LaserScan, self.scan_callback)

        #토픽 발행
        self.drive_pub = rospy.Publisher("cmd_vel_steer", Twist, queue_size=1)
        self.utm_x_offset = rospy.Publisher("utm_x_offset", Float64, queue_size=1)
        self.utm_y_offset = rospy.Publisher("utm_y_offset", Float64, queue_size=1)

        rospy.Timer(rospy.Duration(0.1), self.drive_control)

        #geofence point / 공대 전체
        self.boundery_points  = [(508834.51, 3888940.49),(508813.83, 3888856.47),(508625.42, 3888904.72),(508653.55, 3889011.09),(508834.51, 3888940.49)]
        
        #geofence point / a동 앞
        #self.boundery_points  = [(508801.121907, 3888884.02),(508814.23, 3888883.09),(508815.24, 3888904.29),(508802.92, 3888906.05),(508801.121907, 3888884.02)] 

    def scan_callback(self, data):
        self.x_list = []
        self.y_list = []

        for i, n in enumerate(data.ranges):
            angle = data.angle_min + data.angle_increment * i
            angle_degree = angle * 180 / math.pi
            x = -n * math.cos(angle)
            y = n * math.sin(angle)
            if y > -0.25 and y < 0.25 and x > 0:
                self.x_list.append(x)
                self.y_list.append(y)
        if self.x_list:
            self.dist_obstacle = min(self.x_list)
        else:
            self.dist_obstacle = 3  # 기본값으로 설정하거나 적절한 값을 설정하십시오
        return self.dist_obstacle
        
    def waypoint_csv(self):
        file_path = '/home/lsy/csv_files/jot.csv' #주행 경로 csv파일 주소 , a동 회차가이드원(파란색)
        #file_path = '/home/lsy/csv_files/gongdae.csv' #주행 경로 csv파일 주소 , 공대 한바퀴
        reading_csv = pd.read_csv(file_path)
        way_lx = reading_csv.iloc[:, 0].tolist() #첫번째 열 가져와서 변수리스트 저장
        way_ly = reading_csv.iloc[:, 1].tolist() #두번째 열 가져와서 변수리스트 저장
        return way_lx, way_ly
    
    def utm_x_callback(self, data):
        self.utm_x_raw = data.data
        self.utm_x = self.utm_x_raw + 0.35 * math.cos(self.yaw * math.pi/180) #base_offset_x
        self.utm_x_offset.publish(self.utm_x)

    def utm_y_callback(self, data):
        self.utm_y_raw = data.data
        self.utm_y = self.utm_y_raw + 0.35 * math.sin(self.yaw * math.pi/180) #base_offset_y
        self.utm_y_offset.publish(self.utm_y)

    def send_joy(self, data):
        self.joy_btn = data.buttons
        self.joy_axes = data.axes
        self.auto = self.joy_btn[4]
        self.manual = self.joy_btn[5]
        
    def heading_callback(self, data): #헤딩 좌표계를 UTM 좌표계로 변환
        self.yaw_raw = data.data
        if self.yaw_raw > 0 and self.yaw_raw < 90:
            self.yaw = 90 - self.yaw_raw
        else:
            self.yaw = 450 - self.yaw_raw
    
    def boundery_alarm (self):
        API = Message(service_key = "d7fbc8a68b786c91e039530cc7d04832")
        url = "https://kapi.kakao.com/v2/api/talk/memo/default/send"

        token = "IKFEaXHofDXdE1VwJ2zVCx_B6pgQJkkZAAAAAQopyV4AAAGQB0gi5Fv0-avl6D9k" # 변경된 토큰 대입

        headers = {"Authorization": "Bearer " + token}

        data = {
            "template_object": json.dumps({
                "object_type": "feed",
                "content": {
                    "title": "!!차량 경고!!",
                    "description": "차량이 범위 밖으로 이동했습니다.",
                    "image_url": "https://avatars.githubusercontent.com/u/126976981?v=4", # 이미지 URL
                    "link": {}
                },
                "buttons": [{"title": "복귀명령을 실행합니다","link": {}}]
            })
        }
        response = requests.post(url, headers=headers, data=data)
        print(response.status_code, ", 알림 발송")

    def obs_alarm(self):
        API = Message(service_key = "d7fbc8a68b786c91e039530cc7d04832") # API키 (성환) # API키 (우영)
        url = "https://kapi.kakao.com/v2/api/talk/memo/default/send"

        token = "IKFEaXHofDXdE1VwJ2zVCx_B6pgQJkkZAAAAAQopyV4AAAGQB0gi5Fv0-avl6D9k" # 변경된 토큰 대입

        headers = {"Authorization": "Bearer " + token}

        data = {
            "template_object": json.dumps({
                "object_type": "feed",
                "content": {
                    "title": "!!차량!!",
                    "description": "차량의 앞에 장애물 있습니다.",
                    "image_url": "https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcTh4p-cl_IRGnmdxj5hIzneRzzO617LWM8ekw&s", # 이미지 URL
                    "link": {}
                },
                "buttons": [{"title": "확인하세요","link": {}}]
            })
        }

        response = requests.post(url, headers=headers, data=data)
        print(response.status_code, ", 알림 발송")


    def geofence (self): #geofence _ 생성 및 범위내부 판별코드
        boundery = Polygon(self.boundery_points)
        self_point = Point(self.utm_x, self.utm_y)
        inside_flag = boundery.contains(self_point)
        return inside_flag
    
    def return_path (self):
        
        if self.return_flag == False:
            self.boundery_alarm()
            self.start = [self.utm_x,self.utm_y]
            self.return_flag = True
        if len(self.boundery_points) != 4:
            print("4개의 점이 필요합니다.")
        x_center_cal_list = [p[0] for p in self.boundery_points]
        y_center_cal_list = [p[1] for p in self.boundery_points]

        center_x = sum(x_center_cal_list) / 4.0 #중심점_x
        center_y = sum(y_center_cal_list) / 4.0 #중심점_y
        center = [center_x , center_y]

        return_lx = np.linspace(self.start[0], center[0], 100)
        return_ly = np.linspace(self.start[1], center[1], 100)

        return return_lx, return_ly

    def stanley_control_angle(self): #스탠리 컨트롤
        min_dist = float('inf')
        target_i = 0
        
        #nearest point 계산 및 mindist 계산
        for i in range(len(self.lx)):
            dx = self.lx[i] - self.utm_x
            dy = self.ly[i] - self.utm_y
            dist = math.sqrt(dx ** 2 + dy ** 2)

            if dist < min_dist:
                min_dist = dist
                target_i = i

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

            Kp = 0.05 # 게인값 조정!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            
            # mindist가 가까우면 0으로
            # if min_dist < 0.1:
            #     min_dist = 0

            ##경로의 왼쪽인지 오른쪽인지 판별파트 (min_dist)###
            #map_yaw를 x축으로 하는 국부 좌표계 생성 및 y위치 값 양음수 구분 => flag
            #map_yaw 라디안으로 사용
            flag = ((self.utm_x - self.lx[target_i]) * -math.sin(map_yaw*(math.pi/180))) + ((self.utm_y - self.ly[target_i]) * math.cos(map_yaw*(math.pi/180)))

            if flag >= 0 :
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ")
                min_dist = -min_dist

            add_angle = math.atan2(Kp * min_dist, 0.1) * 180 / math.pi

            #최종 스탠리 앵글
            stanley_steer_angle = cte + add_angle
            
            #확인용
            print("%%%%%%%%%%%%%%%%%%%%%%%")
            print('map_yaw:',map_yaw,'self.yaw:',self.yaw)
            print('min_dist:',min_dist)
            print('flag:',flag)
            print('idx:',target_i) 
            print('cte:',cte)
            print('add_angle:', add_angle)
            print("steer_angle : ", stanley_steer_angle)

            return stanley_steer_angle

    def drive_control(self, event):
        drive = Twist()
        inside = self.geofence()
        current_time = time.time()
        
        if self.dist_obstacle < 2:
            print("장애물 있음")
            drive.linear.x = 0
            drive.angular.z = 0
            if current_time - self.last_alarm_time >= 10:
                self.obs_alarm()
                self.last_alarm_time = current_time

        else:
            print("장애물 없음!!!!!!!!!!!!!")
            if self.manual == 1:
                if inside:
                    self.speed = self.joy_axes[4]
                    self.steering = self.joy_axes[0]
                    drive.linear.x = self.speed*1
                    drive.angular.z = int(self.steering * 30)
                    print("상태: 범위 안")
                else:
                    self.speed = self.joy_axes[4]
                    self.steering = self.joy_axes[0]
                    drive.linear.x = self.speed*1
                    drive.angular.z = int(self.steering * 30)
                    print("상태: 범위 밖")

            elif self.auto == 1:
                if inside:
                    self.return_flag = False
                    self.lx , self.ly = self.waypoint_csv()
                    stanley_steer_angle = self.stanley_control_angle()
                    if stanley_steer_angle is None: # 큰경우 정지
                        print("AAAAAAAAA")
                        drive.linear.x = 0
                        drive.angular.z = 0
                    else:
                        drive.linear.x = 0.7
                        drive.angular.z = stanley_steer_angle
                    print("상태: 범위 안")
                else:
                    self.lx , self.ly = self.return_path()
                    stanley_steer_angle = self.stanley_control_angle()
                    drive.linear.x = 0.7
                    drive.angular.z = stanley_steer_angle
                    print("상태: 범위 밖")

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