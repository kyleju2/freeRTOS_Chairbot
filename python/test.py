import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import serial
import serial.tools.list_ports
import threading
from collections import deque

# --- config ---
WINDOW = 500
BAUD   = 115200

# --- buffers: 4 raw + 4 filtered ---
raw_bufs  = [deque(maxlen=WINDOW) for _ in range(4)]
filt_bufs = [deque(maxlen=WINDOW) for _ in range(4)]
t_buf     = deque(maxlen=WINDOW)
t         = [0.0]
lock      = threading.Lock()

# --- parser ---
def parse_line(line):
    parts = line.split()
    if len(parts) == 9 and parts[0] == 'C':
        try:
            return [float(p) for p in parts[1:]]
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
                            raw_bufs[i].append(result[i])
                            filt_bufs[i].append(result[i + 4])
            except Exception:
                pass

# --- port selection ---
ports = list(serial.tools.list_ports.comports())
for i, p in enumerate(ports):
    print(f"[{i}] {p.device} — {p.description}")
port = ports[int(input("Select port: "))].device
threading.Thread(target=reader_thread, args=(port,), daemon=True).start()

# --- plot: 4 rows, 2 columns (raw | filtered) ---
COLORS_RAW  = ['#888888', '#888888', '#888888', '#888888']
COLORS_FILT = ['#378ADD', '#1D9E75', '#BA7517', '#D4537E']
LABELS      = ['CH0', 'CH1', 'CH2', 'CH3']

app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(title="Raw vs Filtered ADC", show=True)
win.resize(1200, 900)

raw_curves  = []
filt_curves = []

for i in range(4):
    # raw plot
    p_raw = win.addPlot(title=f"{LABELS[i]} Raw")
    p_raw.setLabel('left', 'counts')
    p_raw.setLabel('bottom', 'time', units='s')
    p_raw.showGrid(x=True, y=True)
    raw_curves.append(p_raw.plot(pen=pg.mkPen(COLORS_RAW[i], width=1)))

    # filtered plot same row
    p_filt = win.addPlot(title=f"{LABELS[i]} Filtered")
    p_filt.setLabel('left', 'counts')
    p_filt.setLabel('bottom', 'time', units='s')
    p_filt.showGrid(x=True, y=True)
    filt_curves.append(p_filt.plot(pen=pg.mkPen(COLORS_FILT[i], width=2)))

    # link x axes so they scroll together
    if i > 0:
        p_raw.setXLink(win.getItem(0, 0))
        p_filt.setXLink(win.getItem(0, 1))

    win.nextRow()

def update():
    with lock:
        tv   = list(t_buf)
        rawv = [list(b) for b in raw_bufs]
        filtv = [list(b) for b in filt_bufs]

    if len(tv) > 1:
        for i in range(4):
            raw_curves[i].setData(tv, rawv[i])
            filt_curves[i].setData(tv, filtv[i])

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)

app.exec()