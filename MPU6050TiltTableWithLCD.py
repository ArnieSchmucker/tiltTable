from imu import MPU6050
from machine import I2C, Pin
import time
from math import atan2
from lcd1602 import LCD

i2c = I2C(1, sda=Pin(6), scl=Pin(7), freq=400000)
mpu = MPU6050(i2c)
lcd = LCD()

thetaXF = 0
thetaYF = 0
thetaXA = 0
thetaXAOld = 0
thetaYA = 0
thetaYAOld = 0
thetaXG = 0.0
thetaYG = 0.0
thetaX = 0.0
thetaY = 0.0
dt = 0.0
millisOld = time.ticks_ms()
#print(millisOld)
millisCur = 0.0

while True:
#     print("x: %s, y: %s, z: %s"%(mpu.accel.x, mpu.accel.y, mpu.accel.z-0.06))
#     time.sleep(0.1)
#     print("A: %s, B: %s"%(mpu.gyro.x+3.65, mpu.gyro.y-2.2))
#     time.sleep(0.1)
    dt = time.ticks_diff(time.ticks_ms(), millisOld)/1000.0
    millisOld = time.ticks_ms()
    thetaXG = thetaXG + (mpu.gyro.y-2.2)*dt
    thetaYG = thetaYG + (mpu.gyro.x+3.65)*dt
    #print(dt,thetaXG,thetaYG)

    
    thetaXA = atan2(mpu.accel.x,mpu.accel.z)*57.3
    thetaYA = atan2(mpu.accel.y,mpu.accel.z)*57.3
#     print(thetaXCur, thetaYCur)
    thetaXF = .9*thetaXAOld + .1*thetaXA
    thetaYF = .9*thetaYAOld + .1*thetaYA
    thetaXAOld = thetaX
    thetaYAOld = thetaY
#     print(thetaXF, thetaYF)
    thetaX = 0.95*(thetaX+(mpu.gyro.y-2.2)*dt) + 0.05*thetaXA
    thetaY = 0.95*(thetaY+(mpu.gyro.x+3.65)*dt) + 0.05*thetaYA
    print(thetaX, thetaY)
    lcd.clear()
    string1 = "thetaX %2.2f "%thetaX
    string2 = "thetaY %2.2f "%thetaY
    lcd.write(2,0,string1)
    lcd.write(2,1,string2)
    time.sleep(.1)

    lcd.clear()