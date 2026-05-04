import serial
import serial.tools.list_ports
import re

def parse_line(line):
    m = re.match(r'CH0:\s*(-?\d+),\s*CH1:\s*(-?\d+),\s*CH2:\s*(-?\d+),\s*CH3:\s*(-?\d+)', line)
    return [int(v) for v in m.groups()] if m else None

ports = list(serial.tools.list_ports.comports())
for i, p in enumerate(ports):
    print(f"[{i}] {p.device} — {p.description}")
port = ports[int(input("Select port: "))].device

with serial.Serial(port=port, baudrate=115200, timeout=1) as ser:
    print(f"\nConnected to {port}\n")
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        result = parse_line(line)
        if result:
            ch0, ch1, ch2, ch3 = result
            print(f"CH0={ch0}  CH1={ch1}  CH2={ch2}  CH3={ch3}")