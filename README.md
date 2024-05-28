# Automatic-Line-Follower-Vehicle
A project to create an Automatic Line Follower Vehicle
Το έργο μας έχει τους εξης στόχους:
1) Την δημιουργία ενός συστήματος που θα μπορεί να ανιχνεύει και να παρακολουθεί μια μαύρη γραμμή σε διαφορετικά είδη επιφανειών.
2) Την σταθερή κίνηση του οχήματος κατά μήκος μίας μάυρης γραμμής
 

Για την κατασκευή Θα χρειαστούμε:
1) Ένα Raspberry RP2040 Pico
2) 2 μοτεράκια
3) 3 ρόδες
4) 1 μικρό χάρτινο κουτί για τον σκελετό
5) 1 power bunk για την τροφοδοσία του οχήματος
6) 4 αισθητήρες για την ανίχνευση της μάυρης γραμμής
7) Καλώδια

ΚΩΔΙΚΑΣ:
Για τον κώδικα θα χρειαστούμε το περιβάλλον Thonny.
Το πρόγραμμα μας έχει δημιουργηθεί με την χρήση της γλώσσας MicroPython.
Παρακάτω υπάρχει ο κώδικας του οχήματος μαζί με διάφορα σχόλια για την ευκολότερη κατανόηση του:

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
        print("Robot started!")
        utime.sleep_ms(500)
    
    elif button_state == 0 and running:
        running = False
        print("Robot stopped!")
        stop()
        utime.sleep_ms(500)

    if running:
        sensor_values = read_sensor_values()
        
        print(sensor_values)
        if sensor_values[1] == 0 and sensor_values[2] == 0:
            print("stop")
            stop()
            print("Robot stopped!")
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
