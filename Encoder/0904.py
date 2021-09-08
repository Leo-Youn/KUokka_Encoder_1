import RPi.GPIO as IO
import time


pwmPin1 = 2
dirPin1A = 3
dirPin1B = 4
encPin1A = 17
encPin1B = 27

pwmPin2 = 22
dirPin2A = 10
dirPin2B = 9
encPin2A = 11
encPin2B = 5

pwmPin3 = 19
dirPin3A = 6
dirPin3B = 13
encPin3A = 26
encPin3B = 18

pwmPin4 = 25
dirPin4A = 23
dirPin4B = 24
encPin4A = 8
encPin4B = 7


IO.setmode(IO.BCM)
IO.setwarnings(False)

IO.setup(encPin1A, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encPin1B, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(pwmPin1,IO.OUT)
IO.setup(dirPin1A,IO.OUT)
IO.setup(dirPin1B,IO.OUT)

IO.setup(encPin2A, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encPin2B, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(pwmPin2,IO.OUT)
IO.setup(dirPin2A,IO.OUT)
IO.setup(dirPin2B,IO.OUT)

IO.setup(encPin3A, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encPin3B, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(pwmPin3,IO.OUT)
IO.setup(dirPin3A,IO.OUT)
IO.setup(dirPin3B,IO.OUT)

IO.setup(encPin4A, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encPin4B, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(pwmPin4,IO.OUT)
IO.setup(dirPin4A,IO.OUT)
IO.setup(dirPin4B,IO.OUT)


p1 = IO.PWM(pwmPin1,100)
p1.start(0)
p2 = IO.PWM(pwmPin2,100)
p2.start(0)
p3 = IO.PWM(pwmPin3,100)
p3.start(0)
p4 = IO.PWM(pwmPin4,100)
p4.start(0)


encoderPos1 = 0
encoderPos2 = 0
encoderPos3 = 0
encoderPos4 = 0


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


#targetDeg = 360.
ratio = 360./270./64.

targetDeg1 = encoderPos1 * ratio
targetDeg2 = encoderPos2 * ratio
targetDeg3 = encoderPos3 * ratio
targetDeg4 = encoderPos4 * ratio

Kp = 6.
Kd = 3.
Ki = 3.
dt = 0.

dt_sleep = 0.01
tolerance = 0.01



start_time = time.time()

error_prev1 = 0.
error_prev2 = 0.
error_prev3 = 0.
error_prev4 = 0.

time_prev = 0.


while True:
    motorDeg1 = encoderPos1 * ratio
    motorDeg2 = encoderPos2 * ratio
    motorDeg3 = encoderPos3 * ratio
    motorDeg4 = encoderPos4 * ratio

    error1 = targetDeg1 - motorDeg1
    error2 = targetDeg2 - motorDeg2
    error3 = targetDeg3 - motorDeg3
    error4 = targetDeg4 - motorDeg4

    de1 = error1-error_prev1
    de2 = error2-error_prev2
    de3 = error3-error_prev3
    de4 = error4-error_prev4

    dt = time.time() - time_prev

    control1 = Kp*error1 + Kd*de1/dt + Ki*error1*dt
    control2 = Kp*error2 + Kd*de2/dt + Ki*error2*dt
    control3 = Kp*error3 + Kd*de3/dt + Ki*error3*dt
    control4 = Kp*error4 + Kd*de4/dt + Ki*error4*dt

    error_prev1 = error1
    error_prev2 = error2
    error_prev3 = error3
    error_prev4 = error4

    time_prev = time.time()
   
    IO.output(dirPin1A, control1 >= 0)
    IO.output(dirPin1B, control1 < 0)
    p1.ChangeDutyCycle(min(abs(control1), 100))

    IO.output(dirPin2A, control2 >= 0)
    IO.output(dirPin2B, control2 < 0)
    p2.ChangeDutyCycle(min(abs(control2), 100))

    IO.output(dirPin3A, control3 >= 0)
    IO.output(dirPin3B, control3 < 0)
    p3.ChangeDutyCycle(min(abs(control3), 100))

    IO.output(dirPin4A, control4 >= 0)
    IO.output(dirPin4B, control4 < 0)
    p4.ChangeDutyCycle(min(abs(control4), 100))

    print('1-P-term = %7.1f, D-term = %7.1f, I-term = %7.1f' %(Kp*error1, Kd*de1/dt, Ki*de1*dt))
    print('1-time = %6.3f, enc = %d, deg = %5.1f, err = %5.1f, ctrl = %7.1f' %(time.time()-start_time, encoderPos1, motorDeg1, error1, control1))
    print('%f, %f' %(de1, dt))

    print('2-P-term = %7.1f, D-term = %7.1f, I-term = %7.1f' %(Kp*error2, Kd*de2/dt, Ki*de2*dt))
    print('2-time = %6.3f, enc = %d, deg = %5.1f, err = %5.1f, ctrl = %7.1f' %(time.time()-start_time, encoderPos2, motorDeg2, error2, control2))
    print('%f, %f' %(de2, dt))

    print('3-P-term = %7.1f, D-term = %7.1f, I-term = %7.1f' %(Kp*error3, Kd*de3/dt, Ki*de3*dt))
    print('3-time = %6.3f, enc = %d, deg = %5.1f, err = %5.1f, ctrl = %7.1f' %(time.time()-start_time, encoderPos3, motorDeg3, error3, control3))
    print('%f, %f' %(de3, dt))

    print('4-P-term = %7.1f, D-term = %7.1f, I-term = %7.1f' %(Kp*error4, Kd*de4/dt, Ki*de4*dt))
    print('4-time = %6.3f, enc = %d, deg = %5.1f, err = %5.1f, ctrl = %7.1f' %(time.time()-start_time, encoderPos4, motorDeg4, error4, control4))
    print('%f, %f' %(de4, dt))
 
    if abs(error1) <= tolerance:
        IO.output(dirPin1A, control1>=0)
        IO.output(dirPin1B, control1<0)
        p1.ChangeDutyCycle(0)
    
    if abs(error2) <= tolerance:
        IO.output(dirPin2A, control2>=0)
        IO.output(dirPin2B, control2<0)
        p2.ChangeDutyCycle(0)
   
    if abs(error3) <= tolerance:
        IO.output(dirPin3A, control3>=0)
        IO.output(dirPin3B, control3<0)
        p3.ChangeDutyCycle(0)
   
    if abs(error4) <= tolerance:
        IO.output(dirPin4A, control4>=0)
        IO.output(dirPin4B, control4<0)
        p4.ChangeDutyCycle(0)

    targetDeg4 += 0.1

    time.sleep(dt_sleep)
