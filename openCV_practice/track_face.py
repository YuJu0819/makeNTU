# -*- coding: utf-8 -*-
"""
Created on Mon Jan 15 18:03:22 2018

@author: James Wu
"""

import cv2
import numpy as np
import serial
# import RPi.GPIO as GPIO
import time
# import RPi.GPIO as GPIO
# import time 
# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(11,GPIO.OUT)
# servo = GPIO.PWM(11,50)

# servo.start(0)
fonts = cv2.FONT_HERSHEY_COMPLEX
face_cascade = cv2.CascadeClassifier('./data/haarcascade_frontalface_default.xml')

ref_image = cv2.imread("./data/ref_image.png")
# ref_image = io.imread("./ref_image.png")

# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(11,GPIO.OUT)
# servo = GPIO.PWM(11,50)
GREEN = (0, 255, 0)
RED = (0, 0, 255)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

# distance from camera to object(face) measured
# centimeter
Known_distance = 30.2
  
# width of face in the real world or Object Plane
# centimeter
Known_width = 15.3
#==============================================================================
#   1.多人脸形心检测函数 
#       输入：视频帧
#       输出：各人脸总形心坐标
#==============================================================================

# focal length finder function
def define_Servo(x, y):
    if x>600 and y>350 and x<580 and y<370:
        print("on the middle")
    elif x>=600 and y<=350:
        print("on the right up part")
    elif x<=580 and y<=350:
        print("on the left up part")
    elif x>=600 and y>=370:
        print("on the right down part")
    elif x<=580 and y>=370:
        print("on the left down part")
        

def Focal_Length_Finder(measured_distance, real_width, width_in_rf_image):
  
    # finding the focal length
    focal_length = (width_in_rf_image * measured_distance) / real_width
    return focal_length
  
# distance estimation function
def Distance_finder(Focal_Length, real_face_width, face_width_in_frame):
  
    distance = (real_face_width * Focal_Length)/face_width_in_frame
  
    # return the distance
    return distance

def face_data(image):
  
    face_width = 0  # making face width to zero
  
    # converting color image to gray scale image
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
  
    # detecting face in the image
    faces = face_cascade.detectMultiScale(gray_image, 1.3, 5)
    fonts = cv2.FONT_HERSHEY_COMPLEX
    # looping through the faces detect in the image
    # getting coordinates x, y , width and height
    for (x, y, h, w) in faces:
  
        # draw the rectangle on the face
        cv2.rectangle(image, (x, y), (x+w, y+h), GREEN, 2)
        # print(x, y)
        # getting face width in the pixels
        face_width = w
  
    # return the face width in pixel
    return face_width
  
ref_image_face_width = face_data(ref_image)

def Detection(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  #转换为灰度图
    
    faces = face_cascade.detectMultiScale(gray, 1.3, 5) #人脸检测
    f_width = 0
    if len(faces)>0: 
        for (x,y,w,h) in faces:
            cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
            X = x+w//2
            Y = y+h//2
            center_pt=(X,Y)   #各人脸中点坐标
            cv2.circle(frame, center_pt, 8, (0,0,255), -1)   #绘制各人脸中点
        centroid_X = int(np.mean(faces, axis=0)[0] + np.mean(faces, axis=0)[2]//2) # 各人脸形心横坐标
        centroid_Y = int(np.mean(faces, axis=0)[1] + np.mean(faces, axis=0)[3]//2) # 各人脸形心纵坐标
        centroid_pt=(centroid_X,centroid_Y)   #各人脸形心坐标
        cv2.circle(frame, centroid_pt, 8, (0,0,255), -1)   #绘制各人脸形心
        f_width = w
    else:
     centroid_X = 320
     centroid_Y = 240
    #==========================================================================
    #     绘制参考线
    #==========================================================================
    x = 0
    y = 0
    w = 320
    h = 240
    
    # rectangle_pts = np.array([[x,y],[x+w,y],[x+w,y+h],[x,y+h]], np.int32) #最小包围矩形各顶点
    # cv2.polylines(frame, [rectangle_pts], True, (0,255,0), 2) #绘制最小包围矩形
    
    # x2 = 320;
    # y2 = 240;
    # rectangle_pts2 = np.array([[x2,y2],[x2+w,y2],[x2+w,y2+h],[x2,y2+h]], np.int32) #最小包围矩形各顶点
    # cv2.polylines(frame, [rectangle_pts2], True, (0,255,0), 2) #绘制最小包围矩形

    #==========================================================================
    #     显示
    #==========================================================================
    cv2.imshow('frame',frame)
    
    return centroid_X,centroid_Y, f_width
                
#==============================================================================
#   ****************************主函数入口***********************************
#==============================================================================

# 设置串口参数
# ser = serial.Serial()
# ser.baudrate = 115200    # 设置比特率为115200bps
# ser.port = 'COM3'      # 单片机接在哪个串口，就写哪个串口。这里默认接在"COM3"端口
# ser.open()             # 打开串口

#先发送一个中心坐标使初始化时云台保持水平
# data = '#'+str('320')+'$'+str('240')+'\r\n'
# ser.write(data.encode())        

cap = cv2.VideoCapture(0) #打开摄像头
Focal_length_found = Focal_Length_Finder(
    Known_distance, Known_width, ref_image_face_width)
  
while(cap.isOpened()):
    _, frame = cap.read()
    
    X, Y, face_width_in_frame = Detection(frame) #执行多人脸形心检测函数
    Distance = Distance_finder(
            Focal_length_found, Known_width, face_width_in_frame)
    print(Distance)
    if(X<10000):
        # print('X = ')
        # print(X)
        # print('Y =')
        # print(Y)
        define_Servo(X, Y)
        #按照协议将形心坐标打包并发送至串口
        # data = '#'+str(X)+'$'+str(Y)+'\r\n'
        # ser.write(data.encode())
    cv2.line(frame, (30, 30), (230, 30), RED, 32)
    cv2.line(frame, (30, 30), (230, 30), BLACK, 28)
    cv2.putText(
            frame, f"Distance: {round(Distance,2)} CM", (30, 35), 
          fonts, 0.6, GREEN, 2)
    
    
    k = cv2.waitKey(5) & 0xFF
    if k==27:   #按“Esc”退出
        break

# ser.close()                                     # 关闭串口
cv2.destroyAllWindows()
cap.release()
