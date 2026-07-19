from robot_control import FSDEROBOT   # Load robot library
from gpiozero import DistanceSensor, Button, LED
import time

# Ultrasonic sensor module
makerobo_sensor = DistanceSensor(echo=21, trigger=20, max_distance=3, threshold_distance=0.2)

Btn = Button(19, pull_up=True)   # Button port
Gpin = LED(5)    # Green LED interface
Rpin = LED(6)    # Red LED interface

Bingrobot = FSDEROBOT()   # Instantiate robot object

# Button flag
keyflag = 0

def keysacn():
    global keyflag
    print('*****************************************')
    print('* Button Pressed!*')
    print('*****************************************')
    Rpin.on()
    Gpin.off()
    keyflag = 1

def released():
    print("button was released")
    Rpin.off()
    Gpin.on()

Btn.when_pressed = keysacn
Btn.when_released = released

def loop():
    speed = 20
    while True:
        if keyflag == 1:
            # Measure distance and convert from meters to centimeters
            dis = makerobo_sensor.distance * 100
            if (dis < 40) == True:
                # Continue avoiding until distance is safe
                while (dis < 40) == True:
                    Bingrobot.t_down(speed, 0.5)     # Move backward
                    Bingrobot.turnRight(speed, 0.1)  # Turn right
                    dis = makerobo_sensor.distance * 100  # Update distance measurement
            else:
                Bingrobot.t_up(speed, 0)  # Move forward

            print(dis, 'cm')
            print('')

def destroy():
    Bingrobot.t_stop(.1)

if __name__ == "__main__":
    Bingrobot.t_stop(.1)
    try:
        loop()
    except KeyboardInterrupt:
        destroy()
