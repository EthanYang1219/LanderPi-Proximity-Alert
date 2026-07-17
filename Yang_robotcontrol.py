import signal
import sys
from robot_control import FSDEROBOT #Import robot libary

if __name__ == "__main__":
    Yang_robot = FSDEROBOT() # Initalize robot object

    def stop_and_exit(signum, frame):
        Yang_robot.t_stop(0) #Robot stops (interrupt handler)
        sys.exit(0)

    # Catch both Ctrl+C (SIGINT) and termination requests (SIGTERM),
    # since KeyboardInterrupt alone can be missed if the signal arrives
    # while inside a C extension call (e.g. smbus/gpiozero) that doesn't
    # check for pending Python signals until it returns.
    signal.signal(signal.SIGINT, stop_and_exit)
    signal.signal(signal.SIGTERM, stop_and_exit)

    try:
        while True:
            Yang_robot.t_up(50,1) # Robot moves forward
            Yang_robot.t_stop(0.2)  # Robot stops for 1 second
            Yang_robot.moveRight(50,2) # robot strafes right
            Yang_robot.t_stop(0.1)  # Robot stops for 1 second
            Yang_robot.t_down(50,1) # Robot moves down
            Yang_robot.t_stop(0.2)  # Robot stops for 1 second
            Yang_robot.moveLeft(50,2) # robot strafes left
            Yang_robot.t_stop(0.2)
            
    except KeyboardInterrupt:
        Yang_robot.t_stop(0) #Robot stops (interrupt handler)
    finally:
        Yang_robot.t_stop(0) #Ensure motors are stopped no matter how the loop exits