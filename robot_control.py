import time # For time delays (E.g time.sleep())
import math # For mathematical operations e.g (math.floor)
import smbus # For I2C communication with PCA9685
from gpiozero import LED # For controlling GPIO pins (used for motor D direction)

#Define the different direction options as a list
Dir = [
    "forward", # Motor forward rotation
    "backward" # Motor backward rotation
]

class PCA9685:
    # Registers/etc. These values correspond to specific control registers in the PCA9685 chip (Hexadecimal)
    __SUBADR1 = 0x02
    __SUBADR2 = 0x03
    __SUBADR3 = 0x04
    __MODE1   = 0x00
    __PRESCALE = 0xFE
    __LED0_ON_L = 0x06
    __LED0_ON_H = 0x07
    __LED0_OFF_L = 0x08
    __LED0_OFF_H = 0x09
    __ALLLED_ON_L = 0xFA
    __ALLLED_ON_H = 0xFB
    __ALLLED_OFF_L = 0xFC
    __ALLLED_OFF_H = 0xFD
    
    def __init__(self, address, debug=False): #constructor for the class 
        self.bus = smbus.SMBus(1) #Creates an SMBus/I2C communication instance to access Raspberry Pi's I2C bus 1 (/dev/i2c-1); allows the program to send and receive data with I2C devices (like the PCA9685 chip) connected to the SDA1 (GPIO2) and SCL1 (GPIO3) pins.
        self.address = address
        self.debug = debug
        if (self.debug): 
            print("Resetting PCA9685") #Prints troubleshooting info if debugging is invoked by the user
        self.write(self.__MODE1, 0x00)
        
    def write(self, reg, value): # Writes the value into the register (reg)
        "Writes an 8-bit value to the specified register/address"
        self.bus.write_byte_data(self.address, reg, value)
        if(self.debug):
            print("I2C: Write 0x%02X to register 0x%02X" % (value,reg)) # Prints the value and register address in hexadecimal format for debugging
    
    def read(self, reg):
        "Read an unsigned byte from the I2C device"
        result = self.bus.read_byte_data(self.address, reg)
        if (self.debug):
            print("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X" % (self.address, result & 0xFF, reg))
        return result
    
    def setPWMFreq(self, freq):
        "Sets the PWM frequency"
        prescaleval = 25000000.0 #25MHz
        prescaleval /= 4096.0 # 12-bit
        prescaleval /= float(freq)
        prescaleval -= 1.0 
        if (self.debug):
            print("Setting PWM frequency to %d Hz" % freq)
            print("Estimated pre-scale: %d" % prescaleval)
        prescale = math.floor(prescaleval + 0.5)
        if (self.debug):
            print("Final pre-scale %d" % prescale)

        # Read the current value of MODE1 register to preserve other settings
        oldmode = self.read(self.__MODE1)
        
        # Set sleep bit (bit 4) to 1 while keeping other bits unchanged
        # 0x7F = 0b01111111 masks bit 7 (clear RESTART bit if it was set)
        # 0x10 = 0b00010000 sets SLEEP bit (bit 4) to 1
        newmode = (oldmode & 0x7F) | 0x10  # Enters sleep mode
        
        # Write the new mode to put PCA9685 into sleep mode
        # **IMPORTANT** PRE_SCALE register can only be written in sleep mode
        self.write(self.__MODE1, newmode)  # go to sleep
        
        # Write the calculated prescale value to PRE_SCALE register
        # This sets the PWM frequency = 25MHz / (4096 * (prescale + 1))
        self.write(self.__PRESCALE, int(math.floor(prescale)))
        
        # Restore original mode (clears sleep bit, wake up the chip)
        self.write(self.__MODE1, oldmode)
        
        # Wait for oscillator to stabilize (datasheet recommends at least 500μs (500x10^-6 s))
        time.sleep(0.005)
        
        # Set RESTART bit (bit 7) to restart PWM generation
        # This ensures all PWM channels start from a count of 0
        self.write(self.__MODE1, oldmode | 0x80)
        
    def setPWM(self, channel, on, off):
        "Sets a single PWM channel"
        # Configure PWM duty cycle for specified channel (0-15) in hexadecimals Ex. channel = 6 __LED6_ON_L=__LED0_ON_L+4*6 = 0x1E (base 16 so)
        # Each channel uses 4 consecutive registers: ON_L, ON_H, OFF_L, OFF_H
        self.write(self.__LED0_ON_L + 4*channel, on & 0xFF)     # ON time low byte (0-255)
        self.write(self.__LED0_ON_H + 4*channel, on >> 8)       # ON time high byte (bits 8-11)
        self.write(self.__LED0_OFF_L + 4*channel, off & 0xFF)   # OFF time low byte (0-255)
        self.write(self.__LED0_OFF_H + 4*channel, off >> 8)     # OFF time high byte (bits 8-11)
        if (self.debug):
            print("channel: %d LED_ON: %d LED_OFF %d" % (channel,on,off))
            
    def setDutycycle(self, channel, pulse):
        self.setPWM(channel, 0, int(pulse *(4096 /100))) #Sets the duty cycle percentage (0-100%) for specified channel
    
    def setLevel(self, channel, value):
        if (value == 1):
            self.setPWM(channel , 0, 4095) #Set duty cycle 100% for specified channel (since its a 12-bit timer chip 2^12 = 4096 and its from 0-4095 inclusive)
        else:
            self.setPWM(channel, 0, 0) #Set duty cycle 0% for specified channel
    
class FSDEROBOT():
    def __init__(self):
        #Motor A control Pins
        self.PWMA = 0 # PWM channel for motor A speed
        self.AIN1 = 2 # Direction control for pin 2
        self.AIN2 = 1 # Direction control for pin 1

        #Motor B control Pins
        self.PWMB = 5 # PWM channel for motor B speed
        self.BIN1 = 3 # Direction control for pin 2
        self.BIN2 = 4 # Direction control for pin 1

        #Motor C control Pins
        self.PWMC = 6 # PWM channel for motor C speed
        self.CIN2 = 7 # Direction control for pin 2
        self.CIN1 = 8 # Direction control for pin 1

        #Motor D control Pins
        self.PWMD = 11
        self.DIN2 = 24 # GPIO pin for direction control 2, from Raspberry Pi
        self.DIN1 = 25 # GPIO pin for direction control 1, from Raspberry Pi

        # Initialize PCA9685 PWM controller at I2C address 0x40
        self.pwm = PCA9685(0x40, debug=False)
        self.pwm.setPWMFreq(50) #Sets PWM frequency to 50Hz

        #Setup GPIO pins for motor D direction control
        self.motorD1 = LED(self.DIN1) # Direction pin 1 as output
        self.motorD2 = LED(self.DIN2) # Direction pin 2 as output

    def MotorRun(self, motor, index, speed):
        """Controls individual motors with specified direction and speed"""
        if speed > 100: #Limit speed to 100%
            return

        # Motor A controls:
        if (motor == 0):
            self.pwm.setDutycycle(self.PWMA, speed) #Sets the motor speed as a percentage of the full speed (For example, 30 means 30% of full speed)
            if(index == Dir[0]):#if the index equals "forward" or the forward direction
                self.pwm.setLevel(self.AIN1, 0)
                self.pwm.setLevel(self.AIN2, 1)
            else:
                self.pwm.setLevel(self.AIN1, 1)
                self.pwm.setLevel(self.AIN2, 0)
        # Motor B controls:
        elif (motor == 1):
            self.pwm.setDutycycle(self.PWMB, speed) #Sets the motor speed as a percentage of the full speed (For example, 30 means 30% of full speed)
            if(index == Dir[0]):#if the index equals "forward" or the forward direction
                self.pwm.setLevel(self.BIN1, 1)
                self.pwm.setLevel(self.BIN2, 0)
            else:   #Backwards direction
                self.pwm.setLevel(self.BIN1, 0)
                self.pwm.setLevel(self.BIN2, 1)

        # Motor C controls:
        elif (motor == 2):
            self.pwm.setDutycycle(self.PWMC, speed) #Sets the motor speed as a percentage of the full speed (For example, 30 means 30% of full speed)
            if(index == Dir[0]):#if the index equals "forward" or the forward direction
                self.pwm.setLevel(self.CIN1, 1)
                self.pwm.setLevel(self.CIN2, 0)
            else:   #Backwards direction
                self.pwm.setLevel(self.CIN1, 0)
                self.pwm.setLevel(self.CIN2, 1)

        # Motor D controls (using GPIO):
        elif (motor == 3):
            self.pwm.setDutycycle(self.PWMD, speed) #Sets the motor speed as a percentage of the full speed (For example, 30 means 30% of full speed)
            if(index == Dir[0]):#if the index equals "forward" or the forward direction
                self.motorD1.off()  # Set DIN1 low
                self.motorD2.on()   # Set DIn2 High
            else:
                self.motorD1.on()   # DIn 1 set High
                self.motorD2.off()  # DIn 2 set low

    def MotorStop(self, motor):
        """Stop specified motor by setting speed to 0%"""
        if (motor == 0):
            self.pwm.setDutycycle(self.PWMA, 0)
        elif (motor == 1):
            self.pwm.setDutycycle(self.PWMB, 0)
        elif (motor == 2):
            self.pwm.setDutycycle(self.PWMC, 0)
        elif (motor == 3):
            self.pwm.setDutycycle(self.PWMD, 0)

    #functions for robot movement
    # Motor configurations (from 0-3 inclusive): {Left_Front: 0, Left_Back = 2, Right_Front = 1, Right_Back = 3}

    # Forward movement
    def t_up(self,speed,t_time):
        self.MotorRun(0, "forward" ,speed)
        self.MotorRun(1, "forward" ,speed)
        self.MotorRun(2, "forward" ,speed)
        self.MotorRun(3, "forward" ,speed)
        time.sleep(t_time)

    # Backward movement
    def t_down(self,speed,t_time):
        self.MotorRun(0, "backward" ,speed)
        self.MotorRun(1, "backward" ,speed)
        self.MotorRun(2, "backward" ,speed)
        self.MotorRun(3, "backward" ,speed)
        time.sleep(t_time)

    def t_stop(self,t_time):
        self.MotorStop(0)
        self.MotorStop(1)
        self.MotorStop(2)
        self.MotorStop(3)
        time.sleep(t_time)

    # Sideways translation (To the left)
    def moveLeft(self,speed,t_time):
        self.MotorRun(0, "backward" ,speed)
        self.MotorRun(1, "forward" ,speed)
        self.MotorRun(2, "forward" ,speed)
        self.MotorRun(3, "backward" ,speed)
        time.sleep(t_time)

    # Sideways translation (To the right)
    def moveRight(self,speed,t_time):
        self.MotorRun(0, "forward" ,speed)
        self.MotorRun(1, "backward" ,speed)
        self.MotorRun(2, "backward" ,speed)
        self.MotorRun(3, "forward" ,speed)
        time.sleep(t_time)

    # Rotate Left (Spin in Place)
    def turnLeft(self,speed,t_time):
        self.MotorRun(0, "backward" ,speed)
        self.MotorRun(1, "forward" ,speed)
        self.MotorRun(2, "backward" ,speed)
        self.MotorRun(3, "forward" ,speed)
        time.sleep(t_time)

    # Rotate Right (Spin in Place)
    def turnRight(self,speed,t_time):
        self.MotorRun(0, "forward" ,speed)
        self.MotorRun(1, "backward" ,speed)
        self.MotorRun(2, "forward" ,speed)
        self.MotorRun(3, "backward" ,speed)
        time.sleep(t_time)

    # Forward-Left diagonal movement
    def forward_Left(self,speed,t_time):
        self.MotorStop(0)
        self.MotorRun(1, "forward" ,speed)
        self.MotorRun(2, "forward" ,speed)
        self.MotorStop(3)
        time.sleep(t_time)

    # Forward-Right diagonal movement
    def forward_Right(self,speed,t_time):
        self.MotorRun(0, "forward" ,speed)
        self.MotorStop(1)
        self.MotorStop(2)
        self.MotorRun(3, "forward" ,speed)
        time.sleep(t_time)

    # Backward-left diagonal movement
    def backward_Left(self,speed,t_time):
        self.MotorRun(0, "backward" ,speed)
        self.MotorStop(1)
        self.MotorStop(2)
        self.MotorRun(3, "backward" ,speed)
        time.sleep(t_time)

    # Backward-Right diagonal movement
    def backward_Right(self,speed,t_time):
        self.MotorStop(0)
        self.MotorRun(1, "backward" ,speed)
        self.MotorRun(2, "backward" ,speed)
        self.MotorStop(3)
        time.sleep(t_time)