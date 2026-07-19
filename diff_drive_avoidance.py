

from gpiozero import DistanceSensor, Motor
import time


left_motor  = Motor(forward=17, backward=18)
right_motor = Motor(forward=22, backward=23)

sensor = DistanceSensor(echo=21, trigger=20, max_distance=3, threshold_distance=0.2)

SAFE_DISTANCE = 40     
FORWARD_SPEED = 0.6    
TURN_SPEED = 0.5
BACKUP_TIME = 0.4
TURN_TIME = 0.5

def move_forward(speed=FORWARD_SPEED):
    left_motor.forward(speed)
    right_motor.forward(speed)

def move_backward(speed=FORWARD_SPEED):
    left_motor.backward(speed)
    right_motor.backward(speed)

def turn_left(speed=TURN_SPEED):
    left_motor.backward(speed)
    right_motor.forward(speed)

def turn_right(speed=TURN_SPEED):
    left_motor.forward(speed)
    right_motor.backward(speed)

def stop():
    left_motor.stop()
    right_motor.stop()

def loop():
    while True:
        dis = sensor.distance * 100 
        print(f"Distance: {dis:.1f} cm")

        if dis < SAFE_DISTANCE:
            print("Obstacle detected! Avoiding...")
            stop()
            time.sleep(0.1)

            move_backward()
            time.sleep(BACKUP_TIME)
            stop()

            turn_right()
            time.sleep(TURN_TIME)
            stop()
        else:
            move_forward()

        time.sleep(0.05)

def destroy():
    stop()

if __name__ == "__main__":
    stop()
    try:
        loop()
    except KeyboardInterrupt:
        destroy()
