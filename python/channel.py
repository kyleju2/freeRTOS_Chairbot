import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import serial
import serial.tools.list_ports
import threading
from collections import deque

# --- config ---
WINDOW = 500
BAUD   = 115200

# --- buffers ---
ch_bufs = [deque(maxlen=WINDOW) for _ in range(4)]
t_buf   = deque(maxlen=WINDOW)
t       = [0.0]
lock    = threading.Lock()

# --- parser ---
def parse_line(line):
    parts = line.split()
    if len(parts) == 4:
        try:
            return [float(p) for p in parts]
        except ValueError:
            return None
    return None

# --- reader thread ---
def reader_thread(port):
    with serial.Serial(port=port, baudrate=BAUD, timeout=1) as ser:
        while True:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                result = parse_line(line)
                if result:
                    with lock:
                        t[0] += 0.01
                        t_buf.append(t[0])
                        for i in range(4):
                            ch_bufs[i].append(result[i])
            except Exception:
                pass

# --- port selection ---
ports = list(serial.tools.list_ports.comports())
for i, p in enumerate(ports):
    print(f"[{i}] {p.device} — {p.description}")
port = ports[int(input("Select port: "))].device
threading.Thread(target=reader_thread, args=(port,), daemon=True).start()

# --- plot ---
COLORS = ['#378ADD', '#1D9E75', '#BA7517', '#D4537E']
LABELS = ['CH0', 'CH1', 'CH2', 'CH3']

app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(title="ADC Channel Readings", show=True)
win.resize(900, 800)

plots  = []
curves = []
for i in range(4):
    p = win.addPlot(title=f"{LABELS[i]} — Filtered")
    p.setLabel('left', 'counts')
    p.setLabel('bottom', 'time', units='s')
    p.showGrid(x=True, y=True)
    curves.append(p.plot(pen=pg.mkPen(COLORS[i], width=2)))
    plots.append(p)
    win.nextRow()

def update():
    with lock:
        tv = list(t_buf)
        ch = [list(b) for b in ch_bufs]

    if len(tv) > 1:
        for i in range(4):
            curves[i].setData(tv, ch[i])

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)

app.exec()