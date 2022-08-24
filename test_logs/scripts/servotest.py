from adafruit_servokit import ServoKit
import time
kit=ServoKit(channels=16)
tilt= 90
pan = 90
kit.servo[0].angle = pan
kit.servo[1].angle = tilt
time.sleep(10)

for x in range(16):
    pan = x*10
    tilt = 80 +x * 5
    kit.servo[0].angle = pan
    kit.servo[1].angle = tilt
    print(f'Pan: {pan}, Tilt: {tilt}')
    time.sleep(.5)
