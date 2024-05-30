# Automatic-Line-Follower-Vehicle
A project to create an Automatic Line Follower Vehicle

Our project has the following goals:
1) To crate a system that will be able to detect a black line on defferent surfaces.
2) Our vehicle to move as steadily as possible along a black line
 

For the constuction of the robot we will need:
1) 1 Raspberry RP2040
2) 2 μοτεράκια
3) 2 wheels
4) 1 small paper box for the body
5) 1 power bank which will be the battery of the vehicle
6) 4 sensors for the detection of a black line
7) cables

CODE:
For the code we will need the 'Thonny' application.
Our program has been created inside 'Thonny' application with the use of the programming language called MicroPython.
Below there is the code of the vehicle with some comments to make it more easily to understand:

P.S: Whenever you see a line starting with the word 'print' is being put as a comment. That is because they delay the program, thus making it slower. These lines serve no purpose to our code, unless the programmer wish to see the values for their own interest. 
You can make them visible again by just removing this symbol '#' from a line. 

import machine

import utime

# Define the PID class
class PID:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def compute(self, measurement):
        error = self.setpoint - measurement
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

# Initialize PWM for motor control
left_motor_forward = machine.PWM(machine.Pin(8))
left_motor_backward = machine.PWM(machine.Pin(9))
right_motor_forward = machine.PWM(machine.Pin(10))
right_motor_backward = machine.PWM(machine.Pin(11))

# Initialize sensor pins
sensor_pins = [machine.Pin(pin, machine.Pin.IN) for pin in [26, 6, 4, 2]]

button = machine.Pin(20, machine.Pin.IN, machine.Pin.PULL_DOWN)

# Set PWM frequency and initial speed
running = False
frequency = 10000
speed = 30000
left_motor_forward.freq(frequency)
left_motor_backward.freq(frequency)
right_motor_forward.freq(frequency)
right_motor_backward.freq(frequency)

# Initialize PID controller
pid = PID(kp=1.0, ki=0.0, kd=0.1, setpoint=0)  # Tune these parameters as needed

def read_sensor_values():
    """Read sensor values."""
    return [pin.value() for pin in sensor_pins]

def set_motor_speed(left_speed, right_speed):
    """Set the speed of both motors."""
    if left_speed > 0:
        left_motor_forward.duty_u16(int(left_speed))
        left_motor_backward.duty_u16(0)
    else:
        left_motor_forward.duty_u16(0)
        left_motor_backward.duty_u16(int(-left_speed))

    if right_speed > 0:
        right_motor_forward.duty_u16(int(right_speed))
        right_motor_backward.duty_u16(0)
    else:
        right_motor_forward.duty_u16(0)
        right_motor_backward.duty_u16(int(-right_speed))

def stop():
    """Stop robot."""
    left_motor_forward.duty_u16(0)
    left_motor_backward.duty_u16(0)
    right_motor_forward.duty_u16(0)
    right_motor_backward.duty_u16(0)

# Main loop
while True:
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
        sensor_values = read_sensor_values()
        
        print(sensor_values)
        if sensor_values[1] == 0 and sensor_values[2] == 0:
            #print("stop")
            stop()
            #print("Robot stopped!")
            running = False
            utime.sleep_ms(500)
        else:
            # Calculate the error based on sensor values
            error = sensor_values[0] - sensor_values[3]
            correction = pid.compute(error)
            
            # Adjust motor speeds based on PID correction
            left_motor_speed = speed - correction
            right_motor_speed = speed + correction
            
            set_motor_speed(left_motor_speed, right_motor_speed)
