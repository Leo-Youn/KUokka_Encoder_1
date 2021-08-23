import RPi.GPIO as IO
import time

Motor1E = 2
Motor1A = 3
Motor1B = 4
encPin1A = 17
encPin1B = 27

Motor2E = 22
Motor2A = 10
Motor2B = 9
encPin2A = 11
encPin2B = 5

Motor3E = 19
Motor3A = 6
Motor3B = 13
encPin3A = 26
encPin3B = 18

Motor4E = 25
Motor4A = 23
Motor4B = 24
encPin4A = 8
encPin4B = 7

########################################################## Pin assign
IO.setmode(IO.BCM)
IO.setwarnings(False)

IO.setup(encPin1A, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encPin1B, IO.IN, pull_up_down=IO.PUD_UP)

IO.setup(encPin2A, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encPin2B, IO.IN, pull_up_down=IO.PUD_UP)

IO.setup(encPin3A, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encPin3B, IO.IN, pull_up_down=IO.PUD_UP)

IO.setup(encPin4A, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encPin4B, IO.IN, pull_up_down=IO.PUD_UP)


IO.setup(Motor1A, IO.OUT)
IO.setup(Motor1B, IO.OUT)
IO.setup(Motor1E, IO.OUT)

IO.setup(Motor2A, IO.OUT)
IO.setup(Motor2B, IO.OUT)
IO.setup(Motor2E, IO.OUT)

IO.setup(Motor3A, IO.OUT)
IO.setup(Motor3B, IO.OUT)
IO.setup(Motor3E, IO.OUT)

IO.setup(Motor4A, IO.OUT)
IO.setup(Motor4B, IO.OUT)
IO.setup(Motor4E, IO.OUT)

#################################################### setting

PWM1 = IO.PWM(Motor1E, 100)
PWM1.start(0)

PWM2 = IO.PWM(Motor2E, 100)
PWM2.start(0)

PWM3 = IO.PWM(Motor3E, 100)
PWM3.start(0)

PWM4 = IO.PWM(Motor4E, 100)
PWM4.start(0)

####################################################### PWM setting

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


IO.add_event_detect(encPin1A, IO.BOTH, callback=encoderA)
IO.add_event_detect(encPin1B, IO.BOTH, callback=encoderB)





ratio = 360. / 270. / 64.
targetDeg = encoderPos * ratio
Kp = 5.
Kd = 3.
Ki = 3.
dt = 0.
dt_sleep = 0.01
tolerance = 0.01


start_time = time.time()
error_prev = 0.
time_prev = 0.

while True:
    motorDeg = encoderPos * ratio

    error = targetDeg - motorDeg
    de = error - error_prev
    dt = time.time() - time_prev
    control = Kp * error + Kd * de / dt + Ki * error * dt;

    error_prev = error
    time_prev = time.time()

    IO.output(Motor1A, control >= 0)
    p.ChangeDutyCycle(min(abs(control), 100))
    IO.output(Motor1B, control >= 0)
    p.ChangeDutyCycle(min(abs(control), 100))

    print('P-term = %7.1f, D-term = %7.1f, I-term = %7.1f' % (Kp * error, Kd * de / dt, Ki * de * dt))
    print('time = %6.3f, enc = %d, deg = %5.1f, err = %5.1f, ctrl = %7.1f' % (
    time.time() - start_time, encoderPos, motorDeg, error, control))
    print('%f, %f' % (de, dt))

    if abs(error) <= tolerance:
        IO.output(dirPin, control >= 0)
        p.ChangeDutyCycle(0)
        break

    time.sleep(dt_sleep)