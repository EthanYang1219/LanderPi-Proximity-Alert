from robot_control import FSDEROBOT
from gpiozero import Button, LED

Bingrobot = FSDEROBOT()  # Instantiate robot object

SensorRight = Button(16, pull_up=True)   # Right infrared obstacle avoidance sensor
SensorLeft  = Button(12, pull_up=True)   # Left infrared obstacle avoidance sensor

Btn = Button(19, pull_up=True)   # Button port

Gpin = LED(5)   # Green LED interface
Rpin = LED(6)   # Red LED interface

# Button flag
keyflag = 0

# Button control function
def keysacn():
    global keyflag
    print('*****************************************')
    print('* Button Pressed!*')
    print('*****************************************')
    Rpin.on()    # Turn on red LED
    Gpin.off()   # Turn off green LED
    keyflag = 1  # Set button flag to 1

def released():
    print("button was released")
    Rpin.off()   # Turn off red LED
    Gpin.on()    # Turn on green LED

# Button interrupt function
Btn.when_pressed = keysacn
Btn.when_released = released

if __name__ == '__main__':
    speed = 20
    try:
        while True:
            if keyflag == 1:
                SR_2 = SensorRight.value
                SL_2 = SensorLeft.value
                print("SensorRight=" + " " + str(SR_2))
                print("SensorLeft=" + " " + str(SL_2))
                if SL_2 == 0 and SR_2 == 0:      # No obstacles detected on either side
                    print("t_up")
                    Bingrobot.t_up(speed, 0)
                elif SL_2 == 0 and SR_2 == 1:    # Obstacle detected on right side
                    print("Left")
                    Bingrobot.turnLeft(speed, 0)
                elif SL_2 == 1 and SR_2 == 0:    # Obstacle detected on left side
                    print("Right")
                    Bingrobot.turnRight(speed, 0)
                else:
                    Bingrobot.t_stop(0.3)
                    Bingrobot.t_down(speed, 0.4)
                    Bingrobot.turnLeft(speed, 0.5)
    except KeyboardInterrupt:  # When Ctrl+C is pressed, execute the following cleanup.
        Bingrobot.t_stop(0)
