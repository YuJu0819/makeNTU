
import RPi.GPIO as GPIO
import time 
import cv2
import mediapipe as mp
import math
import pigpio
import sys
import serial
# mp_drawing = mp.solutions.drawing_utils
# mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands
servo = 13
servo2 = 12
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(servo_pin,GPIO.OUT)

# pwm = GPIO.PWM(servo_pin,50) # 50 Hz (20 ms PWM period)
# pwm.start(7.5) # start PWM by rotating to 90 degrees
pwm = pigpio.pi()
pwm.set_mode(servo, pigpio.OUTPUT)
pwm.set_mode(servo2, pigpio.OUTPUT)

pwm.set_PWM_frequency( servo, 50 )
pwm.set_PWM_frequency( servo2, 50 )
pwm.set_servo_pulsewidth( servo, 1500 ) 
pwm.set_servo_pulsewidth( servo2, 1500 ) 

if not pwm.connected:
    print('pwm failed')
    pwm.stop()

def get_pwm(angle):
    # return (angle/18.0) + 2.5
    return (angle-90)*10+1500

def servo_control(dir):
    if dir=='':
        return 

# from score import Scoreboard
class bluetooth:
    def __init__(self, port: str, baudrate: int=9600):
        """ Initialize an BT object, and auto-connect it. """
        # The port name is the name shown in control panel
        # And the baudrate is the communication setting, default value of HC-05 is 9600.
        self.ser = serial.Serial(port, baudrate=baudrate)
        
    def is_open(self) -> bool:
        return self.ser.is_open

    def waiting(self) -> bool:
        return self.ser.in_waiting

    def do_connect(self, port: str, baudrate: int=9600) -> bool:
        """ Connect to the specify port with particular baudrate """
        # Connection function. Disconnect the previous communication, specify a new one.
        self.disconnect()

        try:
            self.ser = serial.Serial(port, baudrate=baudrate)
            return True
        except:
            return False

    def disconnect(self):
        """ Close the connection. """
        self.ser.close()

    def write(self, output: str):
        # Write the byte to the output buffer, encoded by utf-8.
        send = output.encode("utf-8")
        self.ser.write(send)

    def readString(self) -> str:
        # Scan the input buffer until meet a '\n'. return none if doesn't exist.
        if(self.waiting()):
            receiveMsg = self.ser.readline().decode("utf-8")[:-1]

        return receiveMsg

def read():
    while True:
        if bt.waiting():
            print(bt.readString())

def write():
    while True:
        msgWrite = input()
        
        if msgWrite == "exit": sys.exit()
    
        bt.write(msgWrite + "\n") 
        
# bt = bluetooth("COM8")
# while not bt.is_open(): pass
# print("BT Connected!")
# def send_uno(text):
#     print(text)
#     bt.write(text)
                    # time.sleep(0.05)           
   
# 根據兩點的座標，計算角度
def vector_2d_angle(v1, v2):
    v1_x = v1[0]
    v1_y = v1[1]
    v2_x = v2[0]
    v2_y = v2[1]
    try:
        angle_= math.degrees(math.acos((v1_x*v2_x+v1_y*v2_y)/(((v1_x**2+v1_y**2)**0.5)*((v2_x**2+v2_y**2)**0.5))))
    except:
        angle_ = 180
    return angle_

# 根據傳入的 21 個節點座標，得到該手指的角度
def hand_angle(hand_):
    angle_list = []
    # thumb 大拇指角度
    angle_ = vector_2d_angle(
        ((int(hand_[0][0])- int(hand_[2][0])),(int(hand_[0][1])-int(hand_[2][1]))),
        ((int(hand_[3][0])- int(hand_[4][0])),(int(hand_[3][1])- int(hand_[4][1])))
        )
    angle_list.append(angle_)
    # index 食指角度
    angle_ = vector_2d_angle(
        ((int(hand_[0][0])-int(hand_[6][0])),(int(hand_[0][1])- int(hand_[6][1]))),
        ((int(hand_[7][0])- int(hand_[8][0])),(int(hand_[7][1])- int(hand_[8][1])))
        )
    angle_list.append(angle_)
    # middle 中指角度
    angle_ = vector_2d_angle(
        ((int(hand_[0][0])- int(hand_[10][0])),(int(hand_[0][1])- int(hand_[10][1]))),
        ((int(hand_[11][0])- int(hand_[12][0])),(int(hand_[11][1])- int(hand_[12][1])))
        )
    angle_list.append(angle_)
    # ring 無名指角度
    angle_ = vector_2d_angle(
        ((int(hand_[0][0])- int(hand_[14][0])),(int(hand_[0][1])- int(hand_[14][1]))),
        ((int(hand_[15][0])- int(hand_[16][0])),(int(hand_[15][1])- int(hand_[16][1])))
        )
    angle_list.append(angle_)
    # pink 小拇指角度
    angle_ = vector_2d_angle(
        ((int(hand_[0][0])- int(hand_[18][0])),(int(hand_[0][1])- int(hand_[18][1]))),
        ((int(hand_[19][0])- int(hand_[20][0])),(int(hand_[19][1])- int(hand_[20][1])))
        )
    angle_list.append(angle_)
    return angle_list



# 根據手指角度的串列內容，返回對應的手勢名稱
def hand_pos(finger_angle, hand_):
    f1 = finger_angle[0]   # 大拇指角度
    f2 = finger_angle[1]   # 食指角度
    f3 = finger_angle[2]   # 中指角度
    f4 = finger_angle[3]   # 無名指角度
    f5 = finger_angle[4]   # 小拇指角度
    
#     if 

    # 小於 50 表示手指伸直，大於等於 50 表示手指捲縮
    if f1<50 and f2>=50 and f3>=50 and f4>=50 and f5>=50 and (int(hand_[0][0])- int(hand_[2][0]))<0 and abs((int(hand_[0][1])- int(hand_[2][1])))<30:
        return 'right'
    elif f1<50 and f2>=50 and f3>=50 and f4>=50 and f5>=50 and -1*(int(hand_[0][0])- int(hand_[2][0]))<0 and abs((int(hand_[0][1])- int(hand_[2][1])))<30:
        return 'left'
    elif f1<50 and f2>=50 and f3>=50 and f4>=50 and f5>=50 and -1*(int(hand_[0][1])- int(hand_[2][1]))<0 and abs((int(hand_[0][0])- int(hand_[2][0])))<30:
        # servo.ChangeDutyCycle(2)
        # pwm.ChangeDutyCycle(7.0)
        # time.sleep(0.1)
        # print("up")
        # pwm.ChangeDutyCycle(2.0)
        # servo.ChangeDutyCycle(0)
        return 'up'
    elif f1<50 and f2>=50 and f3>=50 and f4>=50 and f5>=50 and (int(hand_[0][1])- int(hand_[2][1]))<0 and abs((int(hand_[0][0])- int(hand_[2][0])))<30:
        # servo.ChangeDutyCycle(2)
        # pwm.ChangeDutyCycle(7.0)
        # time.sleep(0.1)
        # # servo.ChangeDutyCycle(0)
        # print("down")
        # pwm.ChangeDutyCycle(2.0)
        return 'down'
#     if f1<50 and f2>=50 and f3>=50 and f4>=50 and f5>=50:
#         return 'good'
#     elif f1>=50 and f2>=50 and f3<50 and f4>=50 and f5>=50:
#         return 'no!!!'
#     elif f1<50 and f2<50 and f3>=50 and f4>=50 and f5<50:
#         return 'ROCK!'
#     elif f1>=50 and f2>=50 and f3>=50 and f4>=50 and f5>=50:
#         return '0'
#     elif f1>=50 and f2>=50 and f3>=50 and f4>=50 and f5<50:
#         return 'pink'
#     elif f1>=50 and f2<50 and f3>=50 and f4>=50 and f5>=50:
#         return '1'
#     elif f1>=50 and f2<50 and f3<50 and f4>=50 and f5>=50:
#         return '2'
#     elif f1>=50 and f2>=50 and f3<50 and f4<50 and f5<50:
#         return 'ok'
    elif f1<50 and f2>=50 and f3<50 and f4<50 and f5<50:
        return 'ok'
#     elif f1>=50 and f2<50 and f3<50 and f4<50 and f5>50:
#         return '3'
#     elif f1>=50 and f2<50 and f3<50 and f4<50 and f5<50:
#         return '4'
#     elif f1<50 and f2<50 and f3<50 and f4<50 and f5<50:
#         return '5'
#     elif f1<50 and f2>=50 and f3>=50 and f4>=50 and f5<50:
#         return '6'
#     elif f1<50 and f2<50 and f3>=50 and f4>=50 and f5>=50:
#         return '7'
#     elif f1<50 and f2<50 and f3<50 and f4>=50 and f5>=50:
#         return '8'
#     elif f1<50 and f2<50 and f3<50 and f4<50 and f5>=50:
#         return '9'
    else:
        return ''

cap = cv2.VideoCapture(0)            # 讀取攝影機
fontFace = cv2.FONT_HERSHEY_SIMPLEX  # 印出文字的字型
lineType = cv2.LINE_AA               # 印出文字的邊框

# mediapipe 啟用偵測手掌
with mp_hands.Hands(
    model_complexity=0,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as hands:

    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    w, h = 170, 100              # 影像尺寸
    angle = 90
    angle2 = 90
    delta = 5
    while True:
        
        ret, img = cap.read()
        img = cv2.resize(img, (w,h))                 # 縮小尺寸，加快處理效率
        if not ret:
            print("Cannot receive frame")
            break
        img2 = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # 轉換成 RGB 色彩
        results = hands.process(img2)                # 偵測手勢
        if results.multi_hand_landmarks:
            finger_angle = []
            finger_points = []
            
            for hand_landmarks in results.multi_hand_landmarks:
                # print(results.multi_hand_landmarks)
                # finger_points = []                     # 記錄手指節點座標的串列
                for i in hand_landmarks.landmark:
                    # 將 21 個節點換算成座標，記錄到 finger_points
                    x = i.x*w
                    y = i.y*h
                    finger_points.append((x,y))
                if finger_points:
                    finger_angle = hand_angle(finger_points) # 計算手指角度，回傳長度為 5 的串列
                    #print(finger_angle)                     # 印出角度 ( 有需要就開啟註解 )
                    # text = hand_pos(finger_angle, finger_points)            # 取得手勢所回傳的內容
                    # cv2.putText(img, text, (30,120), fontFace, 5, (255,255,255), 10, lineType) # 印出文字
            
            text = hand_pos(finger_angle, finger_points)
            send_uno(text)
            # servo_control(text)
                    # time.sleep(0.1)
            if text=='ok':
                break

        cv2.imshow('oxxostudio', img)
        if cv2.waitKey(5) == ord('q'):
            break
# pwm.ChangeDutyCycle(0) # this prevents jitter
# pwm.stop() # stops the pwm on 13
pwm.set_PWM_dutycycle( servo, 0 )
pwm.set_PWM_frequency( servo, 0 )
pwm.set_PWM_dutycycle( servo2, 0 )
pwm.set_PWM_frequency( servo2, 0 )
# GPIO.cleanup() # good practice when finished using a pin
cap.release()
cv2.destroyAllWindows()