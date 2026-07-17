from robot_control import FSDEROBOT #Load robot libary 
from gpiozero import Button
from gpiozero import LED

Yangrobot = FSDEROBOT() #Instantiate robot objects

SensorRight = Button(16,pull_up=True) # Right infrared obstacle avoidance sensor
SensorLeft = Button(12,pull_up=True)  # Left infrared obstacle avoidance sensor

Btn = Button(19,pull_up=True) # Button port

Gpin = LED(5) # Green LED interface
Rpin = LED(6) # Red LED interface

#Button Flag
keyflag = 0

def keyscan():
    global keyflag
    print("- *23")
    print(" *Button Pressed!* ")
    print("- *23")
    Rpin.on()  # Turns red pin LED On
    Gpin.off() # Turns Green pin LED off
    keyflag = 1 # Sets button flag to 1

def released():
    print("Button was released")
    Rpin.off() #Turn off red LED
    Gpin.on() #Turn on green LED

#Button interrupt function
Btn.when_pressed = keyscan
Btn.when_released = released

if __name__ == "__main__":
    speed = 20
    try:
        while True:
            if keyflag == 1:
                SR_2 = SensorRight.value
                SL_2 = SensorLeft.value
                print("SensorRight=" + " " + str(SR_2))
                print("SensorLeft=" + " " + str(SR_1))
                if SL_2 == 0 and SR_2 == 0:
                    print("t_up")
                elif SL_2 == 0 and SR_2 == 1:
                    print("Left")
                elif SL_2 == 1 and SR_2 == 0:
                    print("Right")
                    Yangrobot.turnRight(speed, 0)
                else:
                    Yangrobot.t_stop(0.3)
                    Yangrobot.t_down(speed,0.4)
                    Yangrobot.turnLeft(speed,0.5)
    except KeyboardInterrupt:
        Yangrobot.t_stop(0)