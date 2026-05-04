import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import serial
import serial.tools.list_ports
import re
import threading
import numpy as np

MAX_FORCE   = 150000
ARROW_SCALE = 0.6
COLORS  = ['#378ADD', '#1D9E75', '#BA7517', '#D4537E']
CORNERS = [(1,0), (1,1), (0,0), (0,1)]
LABELS  = ['FR', 'RR', 'FL', 'RL']

latest = [0, 0, 0, 0]
lock   = threading.Lock()

C30 = np.cos(np.radians(30))
S30 = np.sin(np.radians(30))

def iso(x, y, z=0):
    return (x - y) * C30, (x + y) * S30 + z

def parse_line(line):
    m = re.match(r'CH0:\s*(-?\d+),\s*CH1:\s*(-?\d+),\s*CH2:\s*(-?\d+),\s*CH3:\s*(-?\d+)', line)
    return [int(v) for v in m.groups()] if m else None

def reader_thread(port):
    with serial.Serial(port=port, baudrate=115200, timeout=1) as ser:
        while True:
            result = parse_line(ser.readline().decode('utf-8', errors='ignore').strip())
            if result:
                with lock:
                    for i in range(4):
                        latest[i] = result[i]

ports = list(serial.tools.list_ports.comports())
for i, p in enumerate(ports):
    print(f"[{i}] {p.device} — {p.description}")
port = ports[int(input("Select port: "))].device
threading.Thread(target=reader_thread, args=(port,), daemon=True).start()

app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(title="Platform Forces — Isometric", show=True)
win.resize(700, 620)

plot = win.addPlot()
plot.setAspectLocked(True)
plot.setXRange(-1.2, 1.2)
plot.setYRange(-0.3, 2.0)
plot.hideAxis('bottom')
plot.hideAxis('left')

# Platform outline
OUTLINE = [(0,0), (1,0), (1,1), (0,1)]
outline = [iso(*c) for c in OUTLINE] + [iso(*OUTLINE[0])]
plot.plot([p[0] for p in outline], [p[1] for p in outline],
          pen=pg.mkPen('#888888', width=2))

# Corner dots
for cx, cy in CORNERS:
    sx, sy = iso(cx, cy)
    plot.plot([sx], [sy], pen=None, symbol='o',
              symbolSize=7, symbolBrush='#888888', symbolPen=None)

# Corner arrows
lines  = []
heads  = []
labels = []
for i, (cx, cy) in enumerate(CORNERS):
    sx, sy = iso(cx, cy)
    lines.append(plot.plot([sx, sx], [sy, sy], pen=pg.mkPen(COLORS[i], width=3)))
    head = pg.ArrowItem(angle=-90, headLen=14, headWidth=10,
                        brush=pg.mkBrush(COLORS[i]), pen=pg.mkPen(None))
    plot.addItem(head)
    heads.append(head)
    lbl = pg.TextItem(anchor=(0.5, 1.2), color=COLORS[i])
    lbl.setPos(sx, sy)
    plot.addItem(lbl)
    labels.append((lbl, sx, sy))

# Centre of pressure arrow (white/yellow)
cop_line = plot.plot([0, 0], [0, 0], pen=pg.mkPen('#FFD700', width=3))
cop_head = pg.ArrowItem(angle=-90, headLen=16, headWidth=12,
                         brush=pg.mkBrush('#FFD700'), pen=pg.mkPen(None))
plot.addItem(cop_head)
cop_label = pg.TextItem(anchor=(0.0, 0.5), color='#FFD700')
plot.addItem(cop_label)

def update():
    with lock:
        vals = list(latest)

    # Corner arrows
    for i, (cx, cy) in enumerate(CORNERS):
        v      = vals[i]
        length = (v / MAX_FORCE) * ARROW_SCALE
        bx, by = iso(cx, cy, 0)
        tx, ty = iso(cx, cy, length)

        lines[i].setData([tx, bx], [ty, by])
        heads[i].setPos(bx, by)   # arrowhead at the plane

        lbl, sx, sy = labels[i]
        lbl.setText(f'{LABELS[i]}: {v:,}')
        lbl.setPos(sx, sy - 0.04)

    # Centre of pressure
    total = sum(vals)
    if total != 0:
        cop_x = sum(vals[i] * CORNERS[i][0] for i in range(4)) / total
        cop_y = sum(vals[i] * CORNERS[i][1] for i in range(4)) / total
    else:
        cop_x, cop_y = 0.5, 0.5   # default to platform centre

    cop_length = (total / (MAX_FORCE * 4)) * ARROW_SCALE

    bx, by = iso(cop_x, cop_y, 0)
    tx, ty = iso(cop_x, cop_y, cop_length)

    cop_line.setData([tx, bx], [ty, by])
    cop_head.setPos(bx, by)
    cop_label.setText(f'CoP  total: {total:,}')
    cop_label.setPos(bx + 0.05, ty + 0.05)

    # CoP dot on platform surface
    plot.plot([bx], [by], pen=None, symbol='o',
              symbolSize=10, symbolBrush='#FFD700', symbolPen=None)

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)

app.exec()