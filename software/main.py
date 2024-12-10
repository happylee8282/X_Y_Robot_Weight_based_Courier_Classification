import RPi.GPIO as GPIO
import cv2
import numpy as np
import math 
import time
import threading
from bluetooth import *
from time import sleep
from collections import Counter
from hx711 import HX711
from math import trunc


CW = 1 # 시계방향
CCW = 0 # 반시계 방향
GPIO.setmode(GPIO.BCM) #gpio 핀을 식별하는 것

#모터 X/Z 축
DIR1 = 14 # 회전방향 시계, 반시계
STEP1 = 15 # 스텝단위로 모터를 회전시키기 위한 스텝간격
MODE1 = (2,3,4) #step mode의 정밀제어 부분을 선택(모터드라이버 마다 다름)

DIR2 = 23 # 회전방향 시계, 반시계
STEP2 = 18 # 스텝단위로 모터를 회전시키기 위한 스텝간격
MODE2 = (17,27,22) #step mode의 정밀제어 부분을 선택(모터드라이버 마다 다름)

GPIO.setup(20, GPIO.IN) #리미트 스위치 

#*******************************************************************************
#PIN OUTPUT 출력
#Motor driver1
GPIO.setup(DIR1, GPIO.OUT) # DIR output
GPIO.setup(STEP1, GPIO.OUT) # STEP output
GPIO.setup(MODE1, GPIO.OUT) #output으로 선정

#--------------------------------------------------------------------------------------
#Motor driver2dsm42wm63a+ig32
GPIO.setup(DIR2, GPIO.OUT) # DIR output
GPIO.setup(STEP2, GPIO.OUT) # STEP output
GPIO.setup(MODE2, GPIO.OUT) #output으로 선정

#--------------------------------------------------------------------------
#모터 단계 step2
RESOLUTION = {'Full' : (0,0,0), #data sheet 보고 작성
              'Half' : (1,0,0),
              '1/4' : (0,1,0),
              '1/8' : (1,1,0),
              '1/16' : (0,0,1),
              '1/32' : (1,0,1)}
#--------------------------------------------------------------------------------------
#Motor driver1
GPIO.output(MODE1, RESOLUTION['1/32'])

#Motor driver2
GPIO.output(MODE2, RESOLUTION['1/32'])

SPR = 200 # 모터의 스텝각도가 1.8도 +-5%인데 (360 / 1.8 = 200도로 확인)

step_count = SPR*32
delay = 0.005 / 32
delay_table = 0.0005 / 32

socket = BluetoothSocket( RFCOMM ) #get bluetooth
socket.connect(("98:DA:60:08:99:F5", 1)) #connect
# GPIO 모드 설정 (BCM 또는 BOARD)
GPIO.setmode(GPIO.BCM)

# 경고 비활성화
GPIO.setwarnings(False)

# 전역 변수
cap = cv2.VideoCapture(0)
box_x = [0, 0, 0, 0]
box_y = [0, 0, 0, 0]

sensor_values_x = []
sensor_values_y = []
sensor_values_angle = []
load_list = []

most_angle = ''
most_x = ''
most_y = ''

flag = ''
flag_1 = ''

second = ''


ser = serial.Serial('/dev/ttyUSB0', 9600) # USB 포트에 맞게 수정 필요

def send_command(command):
    ser.write(command.encode())

def my_callback21(channel):
    global flag
    right(44)
    flag = 1
    print("stop")

GPIO.add_event_detect(20, GPIO.FALLING, callback=my_callback21,bouncetime = 400)
    
#GPIO.add_event_detect(26, GPIO.FALLING, callback=my_callback20,bouncetime = 400)

#함수
def down(n):
    global flag
    GPIO.output(DIR1, CCW) # 초기 방향을 시계방향 설정 left
    GPIO.output(DIR2, CW) # 초기 방향을 시계방향 설정right
    for x in range(SPR*n):
        GPIO.output(STEP1, GPIO.HIGH)
        GPIO.output(STEP2, GPIO.HIGH)
        sleep(delay_table)
        GPIO.output(STEP1, GPIO.LOW)
        GPIO.output(STEP2, GPIO.LOW)
        
def up(n):
    global flag
    GPIO.output(DIR1, CW) # 초기 방향을 "반"시계방향 설정
    GPIO.output(DIR2, CCW) # 초기 방향을 "반"시계방향 설정
    for x in range(SPR*n):
        GPIO.output(STEP1, GPIO.HIGH)
        GPIO.output(STEP2, GPIO.HIGH)
        sleep(delay_table)
        GPIO.output(STEP1, GPIO.LOW)
        GPIO.output(STEP2, GPIO.LOW)

def left(n):
    global flag
    motor_running = True
  
    GPIO.output(DIR1, CW) # 초기 방향을 "반"시계방향 설정
    GPIO.output(DIR2, CW) # 초기 방향을 "반"시계방향 설정
    for x in range(SPR*n):
        if flag == 1:
            flag = 0
            break
        GPIO.output(STEP1, GPIO.HIGH)
        GPIO.output(STEP2, GPIO.HIGH)
        sleep(delay_table)
        GPIO.output(STEP1, GPIO.LOW)
        GPIO.output(STEP2, GPIO.LOW)

def right(n):
    global flag
    motor_running = True
    
    GPIO.output(DIR1, CCW) # 초기 방향을 "반"시계방향 설정
    GPIO.output(DIR2, CCW) # 초기 방향을 "반"시계방향 설정
    for x in range(SPR*n):
        if flag == 1:
            flag = 0
            break
        GPIO.output(STEP1, GPIO.HIGH)
        GPIO.output(STEP2, GPIO.HIGH)
        sleep(delay_table)
        GPIO.output(STEP1, GPIO.LOW)
        GPIO.output(STEP2, GPIO.LOW)
        
def Y_r(n):
    global flag_1
    motor_running = True
    GPIO.output(DIR3, CW) # 초기 방향을 "반"시계방향 설정
    GPIO.output(DIR4, CW) # 초기 방향을 "반"시계방향 설정
    for x in range(SPR*n):
        if flag_1 == 1:
            flag_1 = 0
            break
        GPIO.output(STEP3, GPIO.HIGH)
        GPIO.output(STEP4, GPIO.HIGH)
        sleep(delay)
        GPIO.output(STEP3, GPIO.LOW)
        GPIO.output(STEP4, GPIO.LOW)
        
def Y_l(n):
    global flag_1
    GPIO.output(DIR3, CCW) # 초기 방향을 "반"시계방향 설정
    GPIO.output(DIR4, CCW) # 초기 방향을 "반"시계방향 설정
    for x in range(SPR*n):
        if flag_1 == 1:
            flag_1 = 0
            break
        GPIO.output(STEP3, GPIO.HIGH)
        GPIO.output(STEP4, GPIO.HIGH)
        sleep(delay)
        GPIO.output(STEP3, GPIO.LOW)
        GPIO.output(STEP4, GPIO.LOW)

def my_callback(channel):
    msg = "ccw30"
    socket.send(msg)

      
def follow_box(x,y):
    if 260> x and x >170:
        f_box_x=(x-170)*0.875
        right(f_box_x)

        f_box_y = y*(38/9) #수정가능
        Y_l(f_box_y)
        #gripbox()
        print("grip")
        Y_r(f_box_y)
        left(f_box_x)
        
    elif 50<x and x<170:
        f_box_x=(170-x)*0.875
        left(f_box_x)

        f_box_y = y*(38/9) #수정가능
        Y_l(f_box_y)
        #gripbox()
        print("grip")
        Y_r(f_box_y)
        right(f_box_x)

        #move_back()

def gripbox():
    down(60)
    socket.send("g100")
    sleep(1)
    up(60)

def letbox():
    down(60)
    socket.send("p30")
    sleep(1)
    up(60)

def reset_list():
    sensor_values_x = []
    sensor_values_y = []
    sensor_values_angle = []
    load_list = []   


def Blue():
    right(5)
    letbox()
    left(5)

def Green():
    Y_l(10)
    letbox()
    Y_r(10)

def Red():
    left(5)
    letbox()
    right(5)

left(3000)
#Y_r(5000)
print("end camera")

def detect_ALL():
    while True:
        global sensor_values_x
        global sensor_values_y
        global sensor_values_angle
        global size
        global color1
        global load_list
        reading = hx.get_raw_data_mean()
        ax = (reading-228400)/100

        if ax >= 20:
            reset_list()

            load_list.append(ax)
            size = "checking"
            color1 = (255,255,255)

            time.sleep(5)  #5초동안 센서값을 받음

            if sensor_values_x and sensor_values_y and sensor_values_angle and load_list: #센서값을 받음
                x = Counter(sensor_values_x) #센서값에 들어간 값들 count함
                y = Counter(sensor_values_y)
                ang = Counter(sensor_values_angle)
                load = Counter(load_list)

                most_x = int(x.most_common(1)[0][0]) #그중에서 제일 많이 count한 값을 most_x에다가 넣음 이 값을 사용
                most_y = int(y.most_common(1)[0][0])
                most_angle = ang.most_common(1)[0][0]
                load = load.most_common(1)[0][0]
                socket.send(most_angle)
                reset_list() #sensor_values_x = [] 이렇게 다시 리셋해줌

                if 385 > load and load >=20:
                    size = "Blue : " + (time.strftime('%M:%S'))
                    color1 = (255, 0, 0)
                    follow_box(most_x,most_y)
                    Blue()

                
                elif 800 > load and load >=385:
                    size = "Green : " + (time.strftime('%M:%S'))
                    color1 = (0, 255, 0)
                    follow_box(most_x,most_y)
                    Green()

                elif load >=800:
                    size = "Red : " + (time.strftime('%M:%S'))
                    color1 = (0,0,255)
                    follow_box(most_x,most_y)
                    Red()

                reset_list()

        else:
            size = "nothing"
            color1 = (255,255,255)
    

def show_cv():
    global second
    cap.set(3, 320)  # 너비를 320으로 지정
    cap.set(4, 240)  # 높이를 240으로 지정

    while True:
        ret, image = cap.read()  # ret은 부울값이고 image에서 이미지 변수 가져옴
        if not ret:
            break

        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # 흑백사진

        cn = cv2.goodFeaturesToTrack(image, 4, 0.1, 0.0001, blockSize=12)

        if cn is not None:
            cn = np.int32(cn)
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

            idx_1 = 0
            idx_2 = 0

            for i in range(len(cn)):
                x, y = cn[i][0]
                if y > cn[idx_1][0][1]:
                    idx_2 = idx_1
                    idx_1 = i
                elif y > cn[idx_2][0][1]:
                    idx_2 = i

            for i in range(len(cn)):
                x, y = cn[i][0]
                box_x[i] = x
                box_y[i] = y
                cv2.circle(image, (x, y), 4, (0, 0, 255), -1)

            center_x = (box_x[0] + box_x[1] + box_x[2] + box_x[3]) / 4
            center_y = (box_y[0] + box_y[1] + box_y[2] + box_y[3]) / 4

            cv2.circle(image, (int(center_x), int(center_y)), 4, (255, 0, 0), -1)
            text = f"center : ({int(center_x)},{int(center_y)})"
            cv2.putText(image, text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

            x1 = cn[idx_1][0][0]
            y1 = cn[idx_1][0][1]
            x2 = cn[idx_2][0][0]
            y2 = cn[idx_2][0][1]

            dx = abs(x1 - x2)
            dy = abs(y1 - y2)

            if x1 - x2 >= 0:
                result = math.degrees(math.atan2(dy, dx))
                turn = "ccw"
            else:
                result = math.degrees(math.atan2(dy, dx))
                turn = "cw"

            send_text = f"{int(result)}{turn}"
            result_text = f"Angle : {send_text}"

            image_height, image_width = image.shape[:2]
            
            org = (50, 10)
            fontFace = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 0.7
            color = (255, 255, 255)
            thickness = 2
            lineType = cv2.LINE_AA
            
            text_size = cv2.getTextSize(result_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
            org = (image_width - text_size[0] - 20, image_height - 30)
            cv2.putText(image, result_text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            
            second = send_text
            
            text_size_1 = cv2.getTextSize(send_text,fontFace,fontScale,thickness)[0]
    
            org_1 = (image_width - text_size_1[0] - 20, image_height - 10)
    
            cv2.putText(image, send_text, org_1, fontFace, fontScale, color, thickness, lineType)
            
            
            sensor_values_x.append(center_x) #카메라 쓰레드 부분에 센서값을 sendor_values 리스트 값에다가 넣어줌
            sensor_values_y.append(center_y)
            sensor_values_angle.append(send_text)
            
            cv2.imshow("tutorial", image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


thread1 = threading.Thread(target=show_cv)
thread2 = threading.Thread(target=detect_ALL)

# 쓰레드 시작

thread1.start()
thread2.start()

# 메인 쓰레드가 두 쓰레드가 종료될 때까지 기다림

thread1.join()
thread2.join()
