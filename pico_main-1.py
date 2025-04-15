from machine import Pin, PWM, UART
import utime

# === Pins and Motor Setup ===
ENC_A_PIN = 6
ENC_B_PIN = 7
PWM_PIN = 2
DIR_PIN = 14

ENCODER_CPR = 20
GEAR_RATIO = 78
COUNTS_PER_REV = ENCODER_CPR * GEAR_RATIO

Kp, Ki, Kd = 0.1, 0.02, 0.002
TOLERANCE_DEG = 2

def angle_to_counts(deg): return (deg / 360.0) * COUNTS_PER_REV
def counts_to_angle(cnt): return (cnt / COUNTS_PER_REV) * 360.0

uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))

encoder_a = Pin(ENC_A_PIN, Pin.IN, Pin.PULL_UP)
encoder_b = Pin(ENC_B_PIN, Pin.IN, Pin.PULL_UP)

def encoder_callback(pin):
    global current_counts
    a, b = encoder_a.value(), encoder_b.value()
    if pin is encoder_a:
        current_counts += 1 if a != b else -1
    else:
        current_counts += 1 if a == b else -1

encoder_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder_callback)
encoder_b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder_callback)

class Motor:
    def __init__(self, pwm_pin, dir_pin):
        self.dir = Pin(dir_pin, Pin.OUT)
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(10000)
        self.pwm.duty_u16(0)

    def set(self, duty):
        if 0 <= duty <= 1.0:
            self.dir.on()
            self.pwm.duty_u16(int(duty * 65535))
        elif -1.0 <= duty < 0:
            self.dir.off()
            self.pwm.duty_u16(int(-duty * 65535))
        else:
            self.pwm.duty_u16(0)

motor = Motor(PWM_PIN, DIR_PIN)

# ===  State Machine ===
state = 0
input_buffer = ""
current_angle = 0.0
current_counts = 0
target_angle = 0.0
target_counts = 0


print("Pico initialized")

while True:
    if(state == 0):
        # 0 = ready to receive current angle data
        if uart.any():
            byte = uart.read(1)
            if not byte:
                continue
            char = byte.decode()
            if char == 'C':
                print("Entering C")
                input_buffer = ""
                state = 1
            else:
                print("Invalid comm.")
        
    elif(state == 1):
        # 1 = setting current angle internally
        if uart.any():
            byte = uart.read(1)
            if not byte:
                continue
            char = byte.decode()
            if char == 'D':
                angle = float(input_buffer)
                print(f"New current angle: {angle:.2f}")
                current_angle = angle
                current_counts = int(angle_to_counts(current_angle))
                uart.write('C'.encode()) #Send back C to signify we're ready to read in target angle
                state = 2
            else:
                input_buffer += char
        
    elif(state == 2):
        # 2 = ready to receive target angle data
        if uart.any():
            byte = uart.read(1)
            if not byte:
                continue
            char = byte.decode()
            if char == 'S':
                print("Entering S")
                input_buffer = ""
                state = 3
            else:
                print("Invalid comm.")
        
    elif(state == 3):
        # 3 = setting target angle internally
        if uart.any():
            byte = uart.read(1)
            if not byte:
                continue
            char = byte.decode()
            if char == 'D':
                angle = float(input_buffer)
                print(f"New target angle: {angle:.2f}")
                target_angle = angle
                target_counts = angle_to_counts(target_angle)
                uart.write('S'.encode()) #Send back S to signify we're not reading data anymore
                state = 4
            else:
                input_buffer += char
            
    elif(state == 4):
        # 4 = Spin motor to position
        integral = 0
        prev_error = 0
        dt = 0.025
        t = 0.0
        tolerance_counts = angle_to_counts(TOLERANCE_DEG)
        seconds = 4.0
        while t < seconds:
            error = target_counts - current_counts
            integral += error * dt
            derivative = (error - prev_error) / dt
            output = Kp * error + Ki * integral + Kd * derivative
            output = max(min(output, 0.9), -0.9)
            if abs(output) < 0.2 and abs(error) > tolerance_counts:
                output = 0.2 if output > 0 else -0.2
            motor.set(output)
            prev_error = error
            print(f"PID loop | Error: {error:.2f}, Output: {output:.2f}")
            if abs(error) < tolerance_counts:
                current_angle = int(counts_to_angle(current_counts))
                state = 5
                break
            utime.sleep(dt)
            t += dt
        # didnt reach position
        current_angle = counts_to_angle(current_counts)
        motor.set(0)
        if state != 5:
            state = 6
    elif(state == 5):
        # 5 = position reached
        print("Target position reached.")
        time.sleep(1)
        uart.write('P'.encode()) #this means take a photo
        state = 6
    elif(state == 6):
        # 6 = sending new current_angle to pi
        time.sleep(1)
        uart.write(f"{current_angle:.2f}".encode()) #this means current angle
        state = 0
    else:
        # Any other state is an error state
        pass
