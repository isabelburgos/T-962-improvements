# bake.py — Reflow profile builder + runner
# pip install PyQt6 pyqtgraph pyserial
from __future__ import annotations
from typing import Optional, List, Tuple
from dataclasses import dataclass
import csv, math, sys
from datetime import datetime

from PyQt6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

from shared import AsciiProtocol, SerialWorker, list_serial_ports

@dataclass
class Step:
    time_s: Optional[int] = None
    temp_c: Optional[int] = None
    rate_c_s: Optional[float] = None
    hold_s: int = 0

# -------- profile math: sample every 10 s for 7 minutes (42 points) -------
def build_42_sample_profile(steps: List[Step], start_c: int = 25) -> List[int]:
    total_s = 420
    dt = 10
    n = total_s // dt
    temps = [start_c]
    t_elapsed = 0

    for st in steps:
        # choose any 2 of (time, temp, rate)
        t = st.time_s
        T = st.temp_c
        R = st.rate_c_s

        if R is None and (t is not None) and (T is not None):
            R = (T - temps[-1]) / float(t) if t != 0 else 0.0
        elif t is None and (R is not None) and (T is not None):
            t = int(round((T - temps[-1]) / R)) if R != 0 else 0
        elif T is None and (t is not None) and (R is not None):
            T = int(round(temps[-1] + R * t))
        elif (t is None) and (T is None) and (R is None):
            continue  # empty row

        t = 0 if t is None else int(t)
        T = temps[-1] if T is None else int(T)
        R = 0.0 if R is None else float(R)

        # ramp piecewise at 10 s
        rem = total_s - t_elapsed
        seg = min(rem, t)
        # number of 10s ticks in this segment
        ticks = seg // dt
        for k in range(1, ticks + 1):
            temps.append(int(round(temps[-1] + R * dt)))

        # force last of the segment to exact T if we still have time left
        if seg > 0 and temps:
            temps[-1] = T

        # hold
        hold_ticks = int(st.hold_s // dt)
        for _ in range(hold_ticks):
            temps.append(T)

        t_elapsed += seg + st.hold_s
        if t_elapsed >= total_s:
            break

    # after steps, pad with last temp or zeros if none
    while len(temps) < n:
        temps.append(temps[-1] if temps else start_c)

    # truncate or pad to exactly 42
    temps = temps[:n]
    if len(temps) < n:
        temps += [temps[-1]] * (n - len(temps))
    # snap negative small drift to 0
    return [int(round(x)) for x in temps]

# coalesce consecutive identical temps -> (setpoint, duration_s)
def coalesce(temps: List[int], step_s: int) -> List[Tuple[int, int]]:
    if not temps: return []
    out: List[Tuple[int, int]] = []
    cur = temps[0]; run = 1
    for t in temps[1:]:
        if t == cur: run += 1
        else:
            out.append((cur, run * step_s))
            cur = t; run = 1
    out.append((cur, run * step_s))
    return out

# ------------------------- main window -------------------------
class BakeApp(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Bake Profile Builder + Runner (PyQt6)")
        self.resize(1100, 720)
        self.proto = AsciiProtocol()
        self.worker: Optional[SerialWorker] = None
        self.default_port = "/dev/tty.usbserial-BG01OX7L"
        self.default_baud = 115200

        # telemetry buffers
        self.tsec: List[float] = []
        self.actual: List[float] = []
        self.setpoint_seen: List[float] = []
        self.temp0: List[float] = []
        self.temp1: List[float] = []
        self.coldj: List[float] = []
        self.rows: List[List] = []
        self._t0_epoch: Optional[float] = None

        # target profile (computed)
        self.target_ts: List[float] = [i*10 for i in range(42)]
        self.target_Ts: List[int] = []

        self._run_seq: List[Tuple[int,int]] = []  # (setpoint, duration)
        self._run_index = 0
        self._run_active = False

        self.init_ui()
        self.refresh_ports()
        QtCore.QTimer.singleShot(0, self.on_connect)

    # ---- UI
    def init_ui(self):
        central = QtWidgets.QWidget(); self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)

        # LEFT: profile builder
        left = QtWidgets.QVBoxLayout()
        root.addLayout(left, 1)

        self.table = QtWidgets.QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["Time (s)", "Temp (°C)", "Rate (°C/s)", "Hold (s)"])
        self.table.horizontalHeader().setStretchLastSection(True)
        for i in range(4):
            self.table.horizontalHeader().setSectionResizeMode(i, QtWidgets.QHeaderView.ResizeMode.Stretch)
        left.addWidget(self.table)

        rowbtns = QtWidgets.QHBoxLayout()
        self.addBtn = QtWidgets.QPushButton("Add Step")
        self.remBtn = QtWidgets.QPushButton("Remove")
        self.upBtn = QtWidgets.QPushButton("Up")
        self.downBtn = QtWidgets.QPushButton("Down")
        rowbtns.addWidget(self.addBtn); rowbtns.addWidget(self.remBtn)
        rowbtns.addWidget(self.upBtn); rowbtns.addWidget(self.downBtn)
        left.addLayout(rowbtns)

        filebtns = QtWidgets.QHBoxLayout()
        self.loadBtn = QtWidgets.QPushButton("Load CSV…")
        self.saveBtn = QtWidgets.QPushButton("Save CSV…")
        self.buildBtn = QtWidgets.QPushButton("Build 42-point")
        filebtns.addWidget(self.loadBtn); filebtns.addWidget(self.saveBtn); filebtns.addWidget(self.buildBtn)
        left.addLayout(filebtns)

        self.profilePlot = pg.PlotWidget(background="w")
        self.profilePlot.showGrid(x=True,y=True,alpha=0.25)
        self.profilePlot.setLabel("bottom","Time (s)"); self.profilePlot.setLabel("left","Temperature (°C)")
        self.curvePts = self.profilePlot.plot([], [], pen=None, symbol='o', symbolSize=6)
        self.curveLine = self.profilePlot.plot([], [], pen=pg.mkPen(0.2))
        left.addWidget(self.profilePlot, 1)

        self.outTxt = QtWidgets.QPlainTextEdit(); self.outTxt.setReadOnly(True)
        self.outTxt.setPlaceholderText("42-sample list will appear here after Build")
        left.addWidget(self.outTxt, 0)

        # RIGHT: run + telemetry
        right = QtWidgets.QVBoxLayout()
        root.addLayout(right, 1)

        # port line
        portLay = QtWidgets.QHBoxLayout()
        portLay.addWidget(QtWidgets.QLabel("Port:"))
        self.portCombo = QtWidgets.QComboBox(); self.portCombo.setEditable(True)
        portLay.addWidget(self.portCombo, 1)
        self.refreshBtn = QtWidgets.QPushButton("Refresh")
        portLay.addWidget(self.refreshBtn)
        portLay.addSpacing(8)
        portLay.addWidget(QtWidgets.QLabel("Baud:"))
        self.baudEdit = QtWidgets.QLineEdit(str(self.default_baud))
        self.baudEdit.setFixedWidth(100)
        self.baudEdit.setValidator(QtGui.QIntValidator(1200, 10000000))
        portLay.addWidget(self.baudEdit)
        portLay.addStretch(1)
        self.connectBtn = QtWidgets.QPushButton("Connect")
        self.disconnectBtn = QtWidgets.QPushButton("Disconnect"); self.disconnectBtn.setEnabled(False)
        portLay.addWidget(self.connectBtn); portLay.addWidget(self.disconnectBtn)
        self.connLbl = QtWidgets.QLabel("Disconnected"); portLay.addWidget(self.connLbl)
        right.addLayout(portLay)

        # controls
        ctrl = QtWidgets.QGroupBox("Run Control")
        cl = QtWidgets.QHBoxLayout(ctrl)
        cl.addWidget(QtWidgets.QLabel("Step time (s):"))
        self.stepSpin = QtWidgets.QSpinBox(); self.stepSpin.setRange(1, 60); self.stepSpin.setValue(10)
        cl.addWidget(self.stepSpin)
        self.copyBtn = QtWidgets.QPushButton("Copy → Target")
        self.runBtn = QtWidgets.QPushButton("Run Built Profile"); self.runBtn.setEnabled(False)
        self.abortBtn = QtWidgets.QPushButton("Abort"); self.abortBtn.setEnabled(False)
        cl.addStretch(1); cl.addWidget(self.copyBtn); cl.addWidget(self.runBtn); cl.addWidget(self.abortBtn)
        right.addWidget(ctrl)

        # telemetry plot
        self.telePlot = pg.PlotWidget(background="w")
        self.telePlot.showGrid(x=True,y=True,alpha=0.25)
        self.telePlot.setLabel("bottom","Time (s)"); self.telePlot.setLabel("left","Temperature (°C)")
        self.curActual = self.telePlot.plot([], [], pen=pg.mkPen('r', width=2), name="Actual")
        self.curTarget = self.telePlot.plot([], [], pen=pg.mkPen((120,120,120), width=2, style=QtCore.Qt.PenStyle.DashLine),
                                            name="Target Temperature")
        self.curPoints = self.telePlot.plot([], [], pen=None, symbol='o', symbolBrush=(40,40,180), symbolSize=5,
                                            name="Target points")
        if not self.telePlot.plotItem.legend:
            self.telePlot.addLegend()
        right.addWidget(self.telePlot, 1)

        # log + save
        self.saveTeleBtn = QtWidgets.QPushButton("Save Telemetry CSV…"); self.saveTeleBtn.setEnabled(False)
        self.logBox = QtWidgets.QPlainTextEdit(); self.logBox.setReadOnly(True)
        right.addWidget(self.saveTeleBtn); right.addWidget(self.logBox, 1)

        # timers
        self.segTimer = QtCore.QTimer(self); self.segTimer.timeout.connect(self.on_next_segment)

        # wire
        self.addBtn.clicked.connect(self.on_add)
        self.remBtn.clicked.connect(self.on_remove)
        self.upBtn.clicked.connect(lambda: self.on_move(-1))
        self.downBtn.clicked.connect(lambda: self.on_move(+1))
        self.saveBtn.clicked.connect(self.on_save_csv)
        self.loadBtn.clicked.connect(self.on_load_csv)
        self.buildBtn.clicked.connect(self.on_build)
        self.copyBtn.clicked.connect(self.on_copy_target)
        self.runBtn.clicked.connect(self.on_run)
        self.abortBtn.clicked.connect(self.on_abort)

        self.refreshBtn.clicked.connect(self.refresh_ports)
        self.connectBtn.clicked.connect(self.on_connect)
        self.disconnectBtn.clicked.connect(self.on_disconnect)

    # ---- profile table helpers
    def table_to_steps(self) -> List[Step]:
        steps: List[Step] = []
        for r in range(self.table.rowCount()):
            def get(c):
                it = self.table.item(r, c)
                return it.text().strip() if it else ""
            def as_int(s): 
                try: return int(s)
                except: return None
            def as_float(s):
                try: return float(s)
                except: return None
            steps.append(Step(as_int(get(0)), as_int(get(1)), as_float(get(2)), as_int(get(3)) or 0))
        return steps

    def on_add(self):
        r = self.table.rowCount()
        self.table.insertRow(r)
        for c, txt in enumerate(["", "", "", "0"]):
            self.table.setItem(r, c, QtWidgets.QTableWidgetItem(txt))

    def on_remove(self):
        r = self.table.currentRow()
        if r >= 0: self.table.removeRow(r)

    def on_move(self, delta: int):
        r = self.table.currentRow()
        if r < 0: return
        nr = max(0, min(self.table.rowCount()-1, r+delta))
        if nr == r: return
        row = [self.table.takeItem(r, c) for c in range(4)]
        self.table.removeRow(r)
        self.table.insertRow(nr)
        for c,it in enumerate(row):
            self.table.setItem(nr, c, it)
        self.table.setCurrentCell(nr, 0)

    # ---- CSV load/save
    def on_save_csv(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save Profile CSV", "profile.csv", "CSV Files (*.csv)")
        if not path: return
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_s", "temp_c", "rate_c_s", "hold_s"])
            for st in self.table_to_steps():
                w.writerow([st.time_s or "", st.temp_c or "", st.rate_c_s if st.rate_c_s is not None else "", st.hold_s or 0])

    def on_load_csv(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Load Profile CSV", "", "CSV Files (*.csv)")
        if not path: return
        with open(path, "r", newline="") as f:
            rdr = csv.DictReader(f)
            rows = list(rdr)
        self.table.setRowCount(0)
        for row in rows:
            r = self.table.rowCount(); self.table.insertRow(r)
            vals = [
                row.get("time_s", ""), row.get("temp_c",""), row.get("rate_c_s",""), row.get("hold_s","0")
            ]
            for c,txt in enumerate(vals):
                self.table.setItem(r, c, QtWidgets.QTableWidgetItem(txt))

    # ---- build/copy
    def on_build(self):
        steps = self.table_to_steps()
        self.target_Ts = build_42_sample_profile(steps, start_c=25)
        xs = [i*10 for i in range(42)]
        self.curvePts.setData(xs, self.target_Ts)
        self.curveLine.setData(xs, self.target_Ts)
        self.outTxt.setPlainText(str(self.target_Ts))
        # also mirror onto telemetry plot for preview
        self.curPoints.setData(xs, self.target_Ts)
        self.curTarget.setData(xs, self.target_Ts)
        self.profilePlot.setXRange(0, 420, padding=0); self.profilePlot.setYRange(0, 300, padding=0)
        self.telePlot.setXRange(0, 420, padding=0); self.telePlot.setYRange(0, 300, padding=0)
        self.runBtn.setEnabled(self.worker is not None and len(self.target_Ts)==42)

    def on_copy_target(self):
        if not self.target_Ts: self.on_build()
        self.curPoints.setData(self.target_ts, self.target_Ts)
        self.curTarget.setData(self.target_ts, self.target_Ts)

    # ---- connect/disconnect
    def refresh_ports(self):
        sel = self.portCombo.currentText()
        ports = list_serial_ports()
        if self.default_port and self.default_port not in ports:
            ports.insert(0, self.default_port)
        self.portCombo.clear(); self.portCombo.addItems(ports)
        if sel in ports: self.portCombo.setCurrentText(sel)

    def on_connect(self):
        if self.worker is not None: return
        port = (self.portCombo.currentText() or self.default_port).strip()
        if not port: self.log("[UI] No port"); return
        try: baud = int(self.baudEdit.text())
        except: self.log("[UI] Bad baud"); return
        self.worker = SerialWorker(port, baud, self.proto)
        self.worker.rx_text.connect(self.on_rx_text)
        self.worker.rx_raw.connect(lambda b: None)
        self.worker.status.connect(self.log)
        self.worker.error.connect(lambda e: self.log(f"[ERR] {e}"))
        self.worker.connected_changed.connect(self.on_conn_changed)
        self.worker.start()

    def on_disconnect(self):
        if not self.worker: return
        self.on_abort()
        try: self.worker.rx_text.disconnect(self.on_rx_text)
        except Exception: pass
        try: self.worker.connected_changed.disconnect(self.on_conn_changed)
        except Exception: pass
        try: self.worker.stop(); self.worker.wait(800)
        except Exception: pass
        self.worker = None
        self.on_conn_changed(False)
        self.log("[UI] Disconnected")

    def on_conn_changed(self, ok: bool):
        self.connectBtn.setEnabled(ok if hasattr(bool, "__invert__") else not ok)
        self.disconnectBtn.setEnabled(ok)
        self.runBtn.setEnabled(ok and bool(self.target_Ts))
        self.connLbl.setText("Connected" if ok else "Disconnected")

    # ---- run flow
    def on_run(self):
        if not self.worker:
            self.log("[UI] Not connected"); return
        if not self.target_Ts:
            self.on_build()
        step_s = int(self.stepSpin.value())
        self._run_seq = coalesce(self.target_Ts, step_s)
        self._run_index = 0
        self._run_active = True
        self.abortBtn.setEnabled(True)
        self.runBtn.setEnabled(False)
        # reset telemetry buffers
        self._t0_epoch = None; self.tsec.clear(); self.actual.clear(); self.setpoint_seen.clear()
        self.temp0.clear(); self.temp1.clear(); self.coldj.clear(); self.rows.clear()
        self.curActual.setData([], [])
        # kick first segment
        self.on_next_segment()

    def on_abort(self):
        if self._run_active and self.worker:
            self.send_ascii("stop")
        self._run_active = False
        self.segTimer.stop()
        self.abortBtn.setEnabled(False)
        self.runBtn.setEnabled(self.worker is not None and bool(self.target_Ts))

    def on_next_segment(self):
        if not self._run_active: return
        if self._run_index >= len(self._run_seq):
            self.on_abort()
            self.log("[RUN] complete")
            self.saveTeleBtn.setEnabled(bool(self.rows))
            return
        setp, dur = self._run_seq[self._run_index]
        self._run_index += 1
        self.send_ascii(f"bake {setp} {int(dur)}")
        self.log(f"[RUN] bake {setp} for {dur}s")
        self.segTimer.start(dur * 1000)

    # ---- RX parse (CSV-ish bake stream + misc)
    @QtCore.pyqtSlot(str)
    def on_rx_text(self, line: str):
        s = line.strip()
        self.log(s)
        # CSV rows look like:
        # time, t0,t1,t2,t3, Set,Actual, Heat, Fan, ColdJ, Mode
        if "," in s and ("BAKE" in s or "REFLOW" in s or "BAKE-PREHEAT" in s or "IDLE" in s):
            parts = [p.strip() for p in s.split(",")]
            try:
                tm = float(parts[0])
                t0 = float(parts[1]); t1 = float(parts[2])
                SP = float(parts[5]); ACT = float(parts[6])
                HEAT = float(parts[7]); FAN = float(parts[8])
                CJ = float(parts[9]); MODE = parts[10]
            except Exception:
                return
            now = datetime.now().timestamp()
            if self._t0_epoch is None: self._t0_epoch = now
            t = now - self._t0_epoch
            self.tsec.append(t); self.actual.append(ACT); self.setpoint_seen.append(SP)
            self.temp0.append(t0); self.temp1.append(t1); self.coldj.append(CJ)
            self.curActual.setData(self.tsec, self.actual)
            # draw target overlay (static)
            if self.target_Ts:
                self.curTarget.setData(self.target_ts, self.target_Ts)
                self.curPoints.setData(self.target_ts, self.target_Ts)
            self.rows.append([datetime.fromtimestamp(now).isoformat(timespec="milliseconds"), t, t0, t1, SP, ACT, HEAT, FAN, CJ, MODE])
            self.saveTeleBtn.setEnabled(True)
        elif s.startswith("[RUN] Complete") or s.endswith("Complete"):
            self.on_abort()

    # ---- Save telemetry
    def on_save_telemetry(self):
        if not self.rows:
            QtWidgets.QMessageBox.information(self, "No data", "No telemetry captured.")
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save Telemetry CSV", "telemetry.csv", "CSV Files (*.csv)")
        if not path: return
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp_iso","t_sec","temp0","temp1","set","actual","heat","fan","coldj","mode"])
            w.writerows(self.rows)
        self.statusBar().showMessage(f"Saved: {path}", 4000)

    # ---- misc
    def send_ascii(self, line: str):
        if not self.worker: return
        self.worker.send(self.proto.encode(line))

    def log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.logBox.appendPlainText(f"[{ts}] {msg}")


def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setFont(QtGui.QFont("Helvetica", 11))
    pg.setConfigOptions(antialias=True)
    w = BakeApp(); w.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()