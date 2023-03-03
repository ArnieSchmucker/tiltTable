from imu import MPU6050
from machine import I2C, Pin, PWM
import time
import math

i2c = I2C(1, sda=Pin(6), scl=Pin(7), freq=400000)
mpu = MPU6050(i2c)

def interval_mapping(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def servo_write(pin,angle):
    pulse_width=interval_mapping(angle, 0, 180, 0.5,2.5)
    duty=int(interval_mapping(pulse_width, 0, 20, 0,65535))
    pin.duty_u16(duty)

servo2 = PWM(Pin(14))
servo1 = PWM(Pin(15))
servo1.freq(50)
servo2.freq(50)

angleXA = 0.0
angleXG = 0.0
# angleYG = 0.0
filteredAngleX = 0.0
angleXTarget = 0.0
angleXActual = 0.0
angleXError = 0.0
servo1Value = 90
servo2Value = 90

# filteredAngleY = 0.0
alpha = 0.6
timeOld = time.ticks_ms()

servo_write(servo1,servo1Value)
servo_write(servo2,servo2Value)

while True:
    # print("x: %s, y: %s, z: %s"%(mpu.accel.x, mpu.accel.y, mpu.accel.z))
    # time.sleep(0.1)
    # print("A: %s, B: %s, Y: %s"%(mpu.gyro.x, mpu.gyro.y, mpu.gyro.z))
    # time.sleep(0.1)
    ax=round(mpu.accel.x-.04,3)
    ay=round(mpu.accel.y-.07,3)
    az=round(mpu.accel.z-.06,3)
    gx=round(mpu.gyro.x+3.6,3)
    gy=round(mpu.gyro.y-2.2,3)
    gz=round(mpu.gyro.z-.5,3)
    #print("ax",ax,"ay",ay,"az",az,"gx",gx,"gy",gy,"gz",gz)
    #print(ax, ay, az)
    #print(gx,gy,gz)
    #no drift from accelerometers
    angleXA = math.atan2(ax,az)*57.3
#     angleYA = math.atan2(ay,az)*57.3
    #print(angleXA,angleYA)
    #no accelerations from gyrometers
    dt = time.ticks_diff(time.ticks_ms(), timeOld)/1000
    timeOld = time.ticks_ms()
    angleXG = angleXG + gy*dt
#     angleYG = angleYG + gx*dt
    #print(angleXG, angleYG)
#    filteredAngleX = alpha*(angleXG) + (1-alpha)*angleXA
    filteredAngleX = alpha*(filteredAngleX+gy*dt) + (1-alpha)*angleXA
#     filteredAngleY = alpha*(filteredAngleY+gx*dt) + (1-alpha)*angleYA
#     print(filteredAngleX, filteredAngleY)
    angleXActual = filteredAngleX
    angleXError = angleXTarget - angleXActual
    
#     if (angleXError > 1.5):
#         servo1Value = servo1Value+1
#         servo_write(servo1,servo1Value)
#     elif (angleXError < -1.5):
#         servo1Value = servo1Value-1
#         servo_write(servo1,servo1Value)

    if (abs(angleXError) > 1.5):
        servo1Value=servo1Value+angleXError/2
        servo_write(servo1,servo1Value)
    time.sleep(.1)
    