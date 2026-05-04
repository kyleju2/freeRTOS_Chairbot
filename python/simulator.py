import serial
import random
import time

with serial.Serial(port="COM8", baudrate=115200, timeout=1) as ser:
    while True:
        line = f"CH0: {random.randint(55000, 110000)}, CH1: {random.randint(55000, 110000)}, CH2: {random.randint(55000, 110000)}, CH3: {random.randint(55000, 110000)}\r\n"
        ser.write(line.encode())
        time.sleep(0.2)
