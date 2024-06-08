#make this line following code better by using pid
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
running = False
frequency = 10000
speed = 35000
left_motor_forward.freq(frequency)
left_motor_backward.freq(frequency)
right_motor_forward.freq(frequency)
right_motor_backward.freq(frequency)

def read_sensor_values():
    """Read sensor values."""
    return [pin.value() for pin in sensor_pins]

def move_forward(speed):
    """Move robot forward."""
    left_motor_forward.duty_u16(speed)
    left_motor_backward.duty_u16(0)
    right_motor_forward.duty_u16(speed)
    right_motor_backward.duty_u16(0)

def turn_left(speed):
    """Turn robot left."""
    left_motor_forward.duty_u16(0)
    left_motor_backward.duty_u16(0)
    right_motor_forward.duty_u16(speed)
    right_motor_backward.duty_u16(0)

def turn_right(speed):
    """Turn robot right."""
    left_motor_forward.duty_u16(speed)
    left_motor_backward.duty_u16(0)
    right_motor_forward.duty_u16(0)
    right_motor_backward.duty_u16(0)

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
        print("Robot started!")
        move_forward(speed)
        utime.sleep_ms(500)

    
    elif button_state == 0 and running:
        running = False
        print("Robot stopped!")
        stop()
        utime.sleep_ms(500)

    if running:
        sensor_values = read_sensor_values()
        
        print(sensor_values)
        if sensor_values[1]==0 and sensor_values[2]==0:
            print("stop")
            stop()
            print("Robot stopped!")
            running = False
            utime.sleep_ms(500)
        elif sensor_values[0]==0:
            print("left")
            turn_left(speed)
        elif sensor_values[3]==0:
            print("right")
            turn_right(speed)
        elif sensor_values[1]==0 or sensor_values[2]==0:
            print("forward")
            move_forward(speed)
 
