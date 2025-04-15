import time
import datetime
import RPi.GPIO as GPIO
import math
import os
import serial
import csv
import sys
import traceback
from picamera2 import Picamera2

current_angle = 0.0

pir_pins = {0: 17, 90: 24, 180: 26, 270: 16}
pir_power_pins = {0: 27, 90: 23, 180: 19, 270: 20}

pico_serial = serial.Serial('/dev/serial0', 9600, timeout=2)
time.sleep(2)

camera = Picamera2()

GPIO.setmode(GPIO.BCM)
for pin in pir_pins.values():
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
for pin in pir_power_pins.values():
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.HIGH)

log_file = './log.csv'
if not os.path.exists(log_file):
    with open(log_file, 'w', newline='') as f:
        csv.writer(f).writerow(['Timestamp', 'Angle', 'Image Path'])

state = 0
triggered_angle = 0.0
image_path = None
TIMEOUT = 10.0
dt = 0.025
t = 0.0

def rotate_to_angle(target_angle, mode):
    print(f"\n[ROTATE] Sending {'target' if mode else 'current'} angle: {target_angle}")
    pico_serial.reset_output_buffer()
    pico_serial.reset_input_buffer()

    pico_serial.write(b'S' if mode else b'C')
    print(f"[ROTATE] Sent {'S' if mode else 'C'}")

    for digit in f"{int(target_angle):03d}":
        pico_serial.write(digit.encode())
        print(f"[ROTATE] Sent digit: {digit}")
        time.sleep(0.01)

    pico_serial.write(b'D')
    print("[ROTATE] Sent D (done)")

def take_photo(angle):
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    folder = "./photos"
    os.makedirs(folder, exist_ok=True)
    filename = f"pest_{timestamp}_{angle}.jpg"
    path = os.path.join(folder, filename)

    print(f"[CAMERA] Preparing to capture at {angle}°")

    try:
        # Use raw Bayer format for compatibility
        print(f"[TRY] .create_still_configuration")
        config = camera.create_still_configuration(
            main={"size": (1536, 864), "format": "RGB888"}, # for preview or capture
            raw={"size": (1536, 864)}, # raw stream
            buffer_count=1
        )
        print(f"[TRY] .configure")
        camera.configure(config)

        camera.start()
        time.sleep(1)
        print(f"[TRY] .capture_file")
        camera.capture_file(path)
        print(f"[CAMERA] Photo saved to {path}")
    except Exception as e:
        print(f"[CAMERA ERROR] Failed to capture image: {e}")
    finally:
        camera.stop()

    return path



def average_angles(angles):
    x = sum(math.cos(math.radians(a)) for a in angles)
    y = sum(math.sin(math.radians(a)) for a in angles)
    avg_rad = math.atan2(y, x)
    return round(math.degrees(avg_rad) % 360)

def get_target_angle_from_pir(pir_pins):
    triggered = [angle for angle, pin in pir_pins.items() if GPIO.input(pin)]
    return None if not triggered else (triggered[0] if len(triggered) == 1 else average_angles(triggered))

def annotate_image(path):
    print(f"[CV] Annotating image {path} [future CV work]")
    return path

def log_photo(image_path, angle):
    print(f"[LOG] Saving photo taken at {angle}°: {image_path}")
    with open(log_file, 'a', newline='') as f:
        csv.writer(f).writerow([datetime.datetime.now().isoformat(), angle, image_path])

try:
    print("🌱 Garden sentry system running. Waiting for pests...")
    while True:
        print(f"\n--- STATE {state} ---")
        if state == 0:
            angle = get_target_angle_from_pir(pir_pins)
            if angle is not None:
                print(f"[STATE 0] PIR triggered! Angle: {angle}")
                triggered_angle = angle
                state = 1

        elif state == 1:
            print("[STATE 1] Motion confirmed. Preparing to send current angle.")
            state = 2

        elif state == 2:
            print(f"[STATE 2] Sending current angle: {current_angle}")
            rotate_to_angle(current_angle, 0)
            state = 3

        elif state == 3:
            print("[STATE 3] Waiting for Pico response 'C'...")
            pico_comms = pico_serial.readline().decode().strip()
            print(f"[STATE 3] Received: {pico_comms}")
            if pico_comms == 'C':
                state = 4

        elif state == 4:
            print(f"[STATE 4] Sending target angle: {triggered_angle}")
            rotate_to_angle(triggered_angle, 1)
            state = 5

        elif state == 5:
            print("[STATE 5] Waiting for Pico response 'S'...")
            pico_comms = pico_serial.readline().decode().strip()
            print(f"[STATE 5] Received: {pico_comms}")
            if pico_comms == 'S':
                t = 0
                state = 6

        elif state == 6:
            print("[STATE 6] Waiting for spin completion or photo signal...")
            pico_comms = pico_serial.readline().decode().strip()
            print(f"[STATE 6] Received: {pico_comms}")
            if pico_comms == 'P':
                image_path = take_photo(triggered_angle)
            elif pico_comms.replace('.', '', 1).isdigit():
                current_angle = float(pico_comms)
                print(f"[STATE 6] New current angle set to: {current_angle}")
                state = 7
            t += dt
            if t > TIMEOUT:
                print("[STATE 6] TIMEOUT: Motor did not confirm in time.")
                state = 8

        elif state == 7:
            print("[STATE 7] Logging and returning to idle.")
            annotate_image(image_path)
            log_photo(image_path, triggered_angle)
            time.sleep(1)
            state = 0

        else:
            print("[STATE ?] Unknown state, resetting.")
            state = 0

except KeyboardInterrupt:
    print("🛑 Shutting down (KeyboardInterrupt)...")
    GPIO.cleanup()
    pico_serial.close()

except Exception as e:
    print("[FATAL ERROR] Unhandled exception:")
    traceback.print_exc()
    GPIO.cleanup()
    pico_serial.close()
