# Automatic-Line-Follower-Vehicle
A project to create an Automatic Line Follower Vehicle

Our project has the following goals:
1) To create a system that will be able to detect a black line on defferent surfaces.
2) Our vehicle to move as steadily as possible along a black line
 

For the constuction of the robot we will need:
1) 1 board with Raspberry RP2040 ( https://nettop.gr/index.php/en/raspberry-pi-en/pico/raspberry-pi-pico-accessories/maker-pi-rp2040-simplifying-robotics-with-raspberry-pi-rp2040.html )
2) 2 motors
3) 2 wheels
4) 1 small paper box for the body
5) 1 power bank which will be the battery of the vehicle
6) 4 sensors for the detection of a black line
7) cables

Alternatively, you can buy an arduino car that comes with the main body, 3 wheels and 2 motors (+ a socket to use batteries instead of a power bank if you wish for) like this one: https://www.skroutz.gr/s/32863022/Haitronic-2WD-Smart-Robot-Car-Chassis-Kit-for-Arduino-HR0238.html 

CODE:
For the code we will need the 'Thonny' application.
Our program has been created inside 'Thonny' application with the use of the programming language called MicroPython.
Below there is the code of the vehicle with some comments to make it more easily to understand:

P.S: Whenever you see a line starting with the word 'print' is being put as a comment. That is because they delay the program, thus making it slower. These lines serve no purpose to our code, unless the programmer wish to see the values for their own interest. 
You can make them visible again by just removing this symbol '#' from a line. 

import machine

import utime

# Initialize PWM for motor control
    left_motor_forward = machine.PWM(machine.Pin(8))
    left_motor_backward = machine.PWM(machine.Pin(9))
    right_motor_forward = machine.PWM(machine.Pin(10))
    right_motor_backward = machine.PWM(machine.Pin(11))

# Initialize sensor pins
    sensor_pins = [machine.Pin(pin, machine.Pin.IN) for pin in [26, 6, 4, 2]]
    
    button = machine.Pin(20, machine.Pin.IN, machine.Pin.PULL_DOWN)

# Set PWM frequency and initial speed
running = False, frequency = 10000, speed = 30000

    left_motor_forward.freq(frequency)
    left_motor_backward.freq(frequency)
    right_motor_forward.freq(frequency)
    right_motor_backward.freq(frequency)

# Functions for the hardware:
seansor value reading
 
    def read_sensor_values():
        """Read sensor values."""
        return [pin.value() for pin in sensor_pins]
both motors forward with duty cycles = speed

    def move_forward(speed):
        """Move robot forward."""
        left_motor_forward.duty_u16(speed)
        left_motor_backward.duty_u16(0)
        right_motor_forward.duty_u16(speed)
        right_motor_backward.duty_u16(0)
only right motor forward with duty cycles = speed

    def turn_left(speed):
        """Turn robot left."""
        left_motor_forward.duty_u16(0)
        left_motor_backward.duty_u16(0)
        right_motor_forward.duty_u16(speed)
        right_motor_backward.duty_u16(0)
only left motor forward with duty cycles = speed
    
    def turn_right(speed):
        """Turn robot right."""
        left_motor_forward.duty_u16(speed)
        left_motor_backward.duty_u16(0)
        right_motor_forward.duty_u16(0)
        right_motor_backward.duty_u16(0)
both motors stoped
        
    def stop():
        """Stop robot."""
        left_motor_forward.duty_u16(0)
        left_motor_backward.duty_u16(0)
        right_motor_forward.duty_u16(0)
        right_motor_backward.duty_u16(0)

# Main loop
    while True:
start/stop robot with button

    button_state = button.value()
    
    if button_state == 0 and not running:
        running = True
        #print("Robot started!")
        utime.sleep_ms(500)
    
    elif button_state == 0 and running:
        running = False
        #print("Robot stopped!")
        stop()
        utime.sleep_ms(500)
if running:

    if running:
            sensor_values = read_sensor_values()
            
            print(sensor_values)
            if sensor_values[1]==0 and sensor_values[2]==0:
                #print("stop")
                stop()
                #print("Robot stopped!")
                running = False
                utime.sleep_ms(500)
            elif sensor_values[0]==0:
                #print("left")
                turn_left(speed)
            elif sensor_values[3]==0:
                #print("right")
                turn_right(speed)
            elif sensor_values[1]==0 or sensor_values[2]==0:
                #print("forward")
                move_forward(speed)
