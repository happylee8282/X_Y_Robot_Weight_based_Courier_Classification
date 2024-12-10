from machine import Pin, PWM
import sys
from time import sleep



uart = machine.UART(0, baudrate=9600, tx=machine.Pin(0), rx=machine.Pin(1))

pwm1 = PWM(Pin(4))  # 서보모터 16번핀에 연결#grip
pwm1.freq(50)        # PWM 주파수 설정 (50Hz)
pwm2 = PWM(Pin(5))  # 서보모터 1
#6번핀에 연결    
pwm2.freq(50)  

state=0

print("Start input")

"""
def set_motor_speed1(speed1):
    if -100 <= speed1 <= 100:
        duty1 = int((speed1 / 100.0) * (6554 - 3277) + 4915)  # 속도를 duty 값으로 변환 (-100에서 100 사이)
        pwm1.duty_u16(duty1)
        pwm1.duty_u16(0)
    else:
        print("Speed ~100~100")
"""

def set_motor_speed1(speed1):
    if -100 <= speed1 <= 100:
        duty1 = int((speed1 / 100.0) * (6554 - 3277) + 4915)  # 속도를 duty 값으로 변환 (-100에서 100 사이)
        pwm1.duty_u16(duty1)
    else:
        print("Speed ~100~100")

def set_motor_speed2(speed2):
    if -100 <= speed2 <= 100:
        duty2 = int((speed2 / 100.0) * (6554 - 3277) + 4915)  # 속도를 duty 값으로 변환 (-100에서 100 사이)
        pwm2.duty_u16(duty2)
    else:
        print("Speed ~100~100")
        
set_motor_speed2(60)#ture
set_motor_speed1(-30)#grip
while True:
    if uart.any():  # 시리얼로부터 데이터를 받았을 때
        data = uart.readline().strip().decode('utf-8')  # 시리얼로부터 데이터 읽기
        if data:  # 데이터가 비어있지 않을 경우에만notnoer
            num_part = ''.join(filter(str.isdigit, data))  # 숫자 부분 추출
            char_part = ''.join(filter(str.isalpha, data))  # 문자 부분 추출
            speed1=int(num_part)
            if char_part =="ccw":
                set_motor_speed2(speed1*-1-30) #20
                uart.write("go")  # 시리얼로 데이터 전송
                
            elif char_part =="cw":
                set_motor_speed2(speed1-30)#50
                uart.write("go")  # 시리얼로 데이터 전송
                
            elif char_part =="g":
                set_motor_speed1(speed1)
                
            elif char_part =="p":
                set_motor_speed1(speed1*-1)
                
            print("Number part:", speed1)
            print("Character part:", char_part)
    sleep(1)  # 1초간 대기

