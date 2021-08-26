import RPi.GPIO as IO
import time


#모터 1번, 제일 하부, 좌우 담당
Motor1E = 2
Motor1A = 3
Motor1B = 4
encPin1A = 17
encPin1B = 27

#모터 2번, 하부에서 두번째, 거리 담당.
Motor2E = 22
Motor2A = 10
Motor2B = 9
encPin2A = 11
encPin2B = 5

#모터 3번 가운데, 거리담당
Motor3E = 19
Motor3A = 6
Motor3B = 13
encPin3A = 26
encPin3B = 18

#모터 4번 제일 끝, 휴대폰 각도 담당.
Motor4E = 25
Motor4A = 23
Motor4B = 24
encPin4A = 8
encPin4B = 7
 
########################################################## Pin assign

IO.setmode(IO.BCM)
IO.setwarnings(False)

#Encoder 기본 설정들, 모터 1234 다 완료.
IO.setup(encPin1A, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encPin1B, IO.IN, pull_up_down=IO.PUD_UP)

IO.setup(encPin2A, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encPin2B, IO.IN, pull_up_down=IO.PUD_UP)

IO.setup(encPin3A, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encPin3B, IO.IN, pull_up_down=IO.PUD_UP)

IO.setup(encPin4A, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encPin4B, IO.IN, pull_up_down=IO.PUD_UP)

#모터 1번, IN1, IN2, Enable 핀 모두 출력으로.
IO.setup(Motor1A, IO.OUT)
IO.setup(Motor1B, IO.OUT)
IO.setup(Motor1E, IO.OUT)

#모터 2번, IN1, IN2, Enable 핀 모두 출력으로.
IO.setup(Motor2A, IO.OUT)
IO.setup(Motor2B, IO.OUT)
IO.setup(Motor2E, IO.OUT)

#모터 3번, IN1, IN2, Enable 핀 모두 출력으로.
IO.setup(Motor3A, IO.OUT)
IO.setup(Motor3B, IO.OUT)
IO.setup(Motor3E, IO.OUT)

#모터 4번, IN1, IN2, Enable 핀 모두 출력으로.
IO.setup(Motor4A, IO.OUT)
IO.setup(Motor4B, IO.OUT)
IO.setup(Motor4E, IO.OUT)

#################################################### setting

#모터 4개에 대한 PWM제어 설정.
PWM1 = IO.PWM(Motor1E, 100)
PWM1.start(0)

PWM2 = IO.PWM(Motor2E, 100)
PWM2.start(0)

PWM3 = IO.PWM(Motor3E, 100)
PWM3.start(0)

PWM4 = IO.PWM(Motor4E, 100)
PWM4.start(0)

####################################################### PWM setting

#Encoder Position 선언 및 초기화.

encoderPos1 = 0
encoderPos2 = 0
encoderPos3 = 0
encoderPos4 = 0

##########################################################     /encoder position setting/


#모터 1번 Encoder설정.
def encoder1A(channel):
    global encoderPos1
    if IO.input(encPin1A) == IO.input(encPin1B):
        encoderPos1 += 1
    else:
        encoderPos1 -= 1


def encoder1B(channel):
    global encoderPos1
    if IO.input(encPin1A) == IO.input(encPin1B):
        encoderPos1 -= 1
    else:
        encoderPos1 += 1


IO.add_event_detect(encPin1A, IO.BOTH, callback=encoder1A)
IO.add_event_detect(encPin1B, IO.BOTH, callback=encoder1B)

#-----------------------------------------------------------------

#모터 2번 Encoder설정.
def encoder2A(channel):
    global encoderPos2
    if IO.input(encPin2A) == IO.input(encPin2B):
        encoderPos2 += 1
    else:
        encoderPos2 -= 1


def encoder2B(channel):
    global encoderPos2
    if IO.input(encPin2A) == IO.input(encPin2B):
        encoderPos2 -= 1
    else:
        encoderPos2 += 1


IO.add_event_detect(encPin2A, IO.BOTH, callback=encoder2A)
IO.add_event_detect(encPin2B, IO.BOTH, callback=encoder2B)

#-----------------------------------------------------------------

#모터 3번 Encoder설정.
def encoder3A(channel):
    global encoderPos3
    if IO.input(encPin3A) == IO.input(encPin3B):
        encoderPos3 += 1
    else:
        encoderPos3 -= 1


def encoder3B(channel):
    global encoderPos3
    if IO.input(encPin3A) == IO.input(encPin3B):
        encoderPos3 -= 1
    else:
        encoderPos3 += 1


IO.add_event_detect(encPin3A, IO.BOTH, callback=encoder3A)
IO.add_event_detect(encPin3B, IO.BOTH, callback=encoder3B)

#-----------------------------------------------------------------

#모터 4번 Encoder설정.
def encoder4A(channel):
    global encoderPos4
    if IO.input(encPin4A) == IO.input(encPin4B):
        encoderPos4 += 1
    else:
        encoderPos4 -= 1


def encoder4B(channel):
    global encoderPos4
    if IO.input(encPin4A) == IO.input(encPin4B):
        encoderPos4 -= 1
    else:
        encoderPos4 += 1


IO.add_event_detect(encPin4A, IO.BOTH, callback=encoder4A)
IO.add_event_detect(encPin4B, IO.BOTH, callback=encoder4B)



#################################################################################################



ratio = 360. / 270. / 64. 

############################################Target Degree 세팅
targetDeg1 = encoderPos1 * ratio
targetDeg2 = encoderPos2 * ratio
targetDeg3 = encoderPos3 * ratio
targetDeg4 = encoderPos4 * ratio

#################################################### PID 계수 설정.

Kp = 5.
Kd = 3.
Ki = 3.



dt = 0.


dt_sleep = 0.01
tolerance = 0.01 #마진, 어느 정도를 그냥 묵인할 것인지.


start_time = time.time()

#에러 미분용 변수 설정. 뒤에 .붙이는 이유는 실수라는 것을 알려주기 위함. 
error_prev1 = 0.
error_prev2 = 0.
error_prev3 = 0.
error_prev4 = 0.

time_prev = 0.

while True:
    #모터 각도 계산.
    motorDeg1 = encoderPos1 * ratio
    motorDeg2 = encoderPos2 * ratio
    motorDeg3 = encoderPos3 * ratio
    motorDeg4 = encoderPos4 * ratio

    #에러 계산 (목표 각도에서 지금 각도 빼는거임)
    error1 = targetDeg1 - motorDeg1
    error2 = targetDeg2 - motorDeg2
    error3 = targetDeg3 - motorDeg3
    error4 = targetDeg4 - motorDeg4
    
    #에러 미분값
    de1 = error1 - error_prev1
    de2 = error2 - error_prev2
    de3 = error3 - error_prev3
    de4 = error4 - error_prev4
    
    #dt 설정.
    dt = time.time() - time_prev

    #컨트롤 변수 설정. dt는 동일함.
    control1 = Kp * error1 + Kd * de1 / dt + Ki * error1 * dt;
    control2 = Kp * error2 + Kd * de2 / dt + Ki * error2 * dt;
    control3 = Kp * error3 + Kd * de3 / dt + Ki * error3 * dt;
    control4 = Kp * error4 + Kd * de4 / dt + Ki * error4 * dt;

    #에러 미분용 이전 값.
    error_prev1 = error1
    error_prev2 = error2
    error_prev3 = error3
    error_prev4 = error4
    
    time_prev = time.time()

    #모터 돌아가게 하는거임. ################################################이부분 약간 위험함. Motor output 설정이 정확한지 모르겠음. 
    #각 모터 옆에 control 조건문이 결국 1, 0을 담당하는데(참 거짓에 따라서 참은1 거짓은0), 이렇게 써도 되려나????
    IO.output(Motor1A, control1 >= 0)
    IO.output(Motor1B, control1 <= 0)
    PWM1.ChangeDutyCycle(min(abs(control1), 100))
    
    IO.output(Motor2A, control2 >= 0)
    IO.output(Motor2B, control2 <= 0)
    PWM1.ChangeDutyCycle(min(abs(control2), 100))
    
    IO.output(Motor3A, control3 >= 0)
    IO.output(Motor3B, control3 <= 0)
    PWM1.ChangeDutyCycle(min(abs(control3), 100))

    IO.output(Motor4A, control4 >= 0) 
    IO.output(Motor4B, control4 <= 0)
    PWM1.ChangeDutyCycle(min(abs(control4), 100))
    ######################### 에러랑 등등 확인용 print 임.

    print('1___P-term = %7.1f, D-term = %7.1f, I-term = %7.1f' % (Kp * error1, Kd * de1 / dt, Ki * de1 * dt))
    print('1__time = %6.3f, enc = %d, deg = %5.1f, err = %5.1f, ctrl = %7.1f' % (time.time() - start_time, encoderPos1, motorDeg1, error1, control1))
    print('1__%f, %f' % (de1, dt))
    
    print('2___P-term = %7.1f, D-term = %7.1f, I-term = %7.1f' % (Kp * error2, Kd * de2 / dt, Ki * de2 * dt))
    print('2__time = %6.3f, enc = %d, deg = %5.1f, err = %5.1f, ctrl = %7.1f' % (time.time() - start_time, encoderPos2, motorDeg2, error2, control2))
    print('2__%f, %f' % (de1, dt))
    
    print('3___P-term = %7.1f, D-term = %7.1f, I-term = %7.1f' % (Kp * error3, Kd * de3 / dt, Ki * de3 * dt))
    print('3__time = %6.3f, enc = %d, deg = %5.1f, err = %5.1f, ctrl = %7.1f' % (time.time() - start_time, encoderPos3, motorDeg3, error3, control3))
    print('3__%f, %f' % (de3, dt))
    
    print('4___P-term = %7.1f, D-term = %7.1f, I-term = %7.1f' % (Kp * error4, Kd * de4 / dt, Ki * de4 * dt))
    print('4__time = %6.3f, enc = %d, deg = %5.1f, err = %5.1f, ctrl = %7.1f' % (time.time() - start_time, encoderPos4, motorDeg4, error4, control4))
    print('4__%f, %f' % (de4, dt))

    ########################## 이 부분 때문에 일정 시간 지난 다음 손으로 내리면 Detecting 안하는거임.
    
    Modifying_Token =0 #4가지 모터가 전부 범위 내에 있을 때를 위함.
    
    if abs(error1) <= tolerance :
        IO.output(Motor1A, control1 = 0)
        IO.output(Motor1B, control1 = 0)
        PWM1.ChangeDutyCycle(0)
        Modifying_Token +=1
        
    if abs(error2) <= tolerance:
        IO.output(Motor2A, control2 = 0)
        IO.output(Motor2B, control2 = 0)
        PWM2.ChangeDutyCycle(0)
        Modifying_Token +=1
        
    if abs(error3) <= tolerance:
        IO.output(Motor3A, control3 = 0)
        IO.output(Motor3B, control3 = 0)
        PWM3.ChangeDutyCycle(0)
        Modifying_Token +=1
        
    if abs(error4) <= tolerance:
        IO.output(Motor4A, control4 = 0)
        IO.output(Motor4B, control4 = 0)
        PWM4.ChangeDutyCycle(0)
        Modifying_Token +=1
        
    if Modifying_Token >= 4 :
        break

    time.sleep(dt_sleep)
    
    
    
    