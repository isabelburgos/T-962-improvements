# passive_datalogger.py
# Minimal applet that polls "values" and logs/plots passively
# pip install PyQt6 pyqtgraph pyserial

from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, List, Tuple, Deque
from collections import deque
from datetime import datetime
import sys, math

from PyQt6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

# Import protocol/serial worker/port list from shared.py
from shared import AsciiProtocol, SerialWorker, list_serial_ports


# -------------------------
# Passive Datalogger UI
# -------------------------
class Datalogger(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Passive Datalogger (PyQt6)")
        self.resize(950, 600)

        # Defaults
        self.default_port = "/dev/tty.usbserial-BG01OX7L"
        self.default_baud = 115200
        self.proto = AsciiProtocol()
        self.worker: Optional[SerialWorker] = None

        # Buffers for telemetry + CSV
        self._t0_epoch: Optional[float] = None
        self.tsec: List[float] = []
        self.left: List[Optional[float]] = []
        self.right: List[Optional[float]] = []
        self.cj: List[Optional[float]] = []
        self.actual: List[Optional[float]] = []  # avg of left/right
        self.rows: List[Tuple[str, float, Optional[float], Optional[float], Optional[float], Optional[float]]] = []

        self._passive_active = False

        self.init_ui()
        self.refresh_ports()
        # Auto-connect on launch
        QtCore.QTimer.singleShot(0, self.on_connect)

    # ---------- UI ----------
    def init_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        # Top bar: Port / Baud / Connect / Disconnect
        top = QtWidgets.QHBoxLayout()
        top.addWidget(QtWidgets.QLabel("Port:"))
        self.portCombo = QtWidgets.QComboBox(); self.portCombo.setEditable(True)
        top.addWidget(self.portCombo, 1)
        self.refreshBtn = QtWidgets.QPushButton("Refresh")
        top.addWidget(self.refreshBtn)
        top.addSpacing(12)
        top.addWidget(QtWidgets.QLabel("Baud:"))
        self.baudEdit = QtWidgets.QLineEdit(str(self.default_baud))
        self.baudEdit.setValidator(QtGui.QIntValidator(1200, 10_000_000))
        self.baudEdit.setFixedWidth(100)
        top.addWidget(self.baudEdit)
        top.addStretch(1)
        self.connectBtn = QtWidgets.QPushButton("Connect")
        self.disconnectBtn = QtWidgets.QPushButton("Disconnect")
        self.disconnectBtn.setEnabled(False)
        top.addWidget(self.connectBtn)
        top.addWidget(self.disconnectBtn)
        self.connStatus = QtWidgets.QLabel("Disconnected")
        top.addSpacing(12)
        top.addWidget(self.connStatus)
        root.addLayout(top)

        # Passive controls
        ctrl = QtWidgets.QGroupBox("Passive Datalogging")
        cl = QtWidgets.QHBoxLayout(ctrl)
        cl.addWidget(QtWidgets.QLabel("Poll every (s):"))
        self.intervalSpin = QtWidgets.QSpinBox()
        self.intervalSpin.setRange(1, 60)
        self.intervalSpin.setValue(1)
        cl.addWidget(self.intervalSpin)
        cl.addStretch(1)
        self.startBtn = QtWidgets.QPushButton("Start")
        self.stopBtn = QtWidgets.QPushButton("Stop"); self.stopBtn.setEnabled(False)
        cl.addWidget(self.startBtn); cl.addWidget(self.stopBtn)
        self.saveBtn = QtWidgets.QPushButton("Save CSV…"); self.saveBtn.setEnabled(False)
        cl.addWidget(self.saveBtn)
        # Axes controls
        self.resetAxesBtn = QtWidgets.QPushButton("Reset Axes")
        cl.addWidget(self.resetAxesBtn)
        root.addWidget(ctrl)

        # Live values (compact grid)
        tele = QtWidgets.QGroupBox("Live Values")
        tlay = QtWidgets.QGridLayout(tele)
        tlay.setHorizontalSpacing(12); tlay.setVerticalSpacing(6)
        def mkbig():
            lab = QtWidgets.QLabel("—")
            f = lab.font(); f.setPointSize(12); f.setBold(True); lab.setFont(f)
            lab.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight | QtCore.Qt.AlignmentFlag.AlignVCenter)
            return lab
        self.vSet = mkbig(); self.vAct = mkbig(); self.vCJ = mkbig()
        self.vLeft = mkbig(); self.vRight = mkbig()
        def addp(title, w, r, c): tlay.addWidget(QtWidgets.QLabel(title), r, c*2); tlay.addWidget(w, r, c*2+1)
        addp("Set (°C):", self.vSet, 0, 0)
        addp("Actual (°C):", self.vAct, 0, 1)
        addp("Cold Jct (°C):", self.vCJ, 0, 2)
        addp("Left (°C):", self.vLeft, 1, 0)
        addp("Right (°C):", self.vRight, 1, 1)
        root.addWidget(tele)

        # Plot
        self.plot = pg.PlotWidget(background="w")
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.setLabel("bottom", "Time (s)")
        self.plot.setLabel("left", "Temperature (°C)")
        self.curveActual = self.plot.plot([], [], pen=pg.mkPen('r', width=2), name="Actual")
        self.curveLeft = self.plot.plot([], [], pen=pg.mkPen((0, 120, 255), width=1), name="Left")
        self.curveRight = self.plot.plot([], [], pen=pg.mkPen((0, 170, 0), width=1), name="Right")
        if not self.plot.plotItem.legend:
            self.plot.addLegend()
        self.configure_plot_axes(fixed_x_max=420)  # start with 0..420s, expand as needed
        root.addWidget(self.plot, 1)

        # Log
        self.logBox = QtWidgets.QPlainTextEdit(); self.logBox.setReadOnly(True)
        root.addWidget(self.logBox, 1)

        # Timers & signals
        self.pollTimer = QtCore.QTimer(self)
        self.pollTimer.timeout.connect(lambda: self.send_ascii("values"))

        self.refreshBtn.clicked.connect(self.refresh_ports)
        self.connectBtn.clicked.connect(self.on_connect)
        self.disconnectBtn.clicked.connect(self.on_disconnect)
        self.startBtn.clicked.connect(self.on_start)
        self.stopBtn.clicked.connect(self.on_stop)
        self.saveBtn.clicked.connect(self.on_save_csv)
        self.resetAxesBtn.clicked.connect(self.on_reset_axes)

    # ---------- Port helpers ----------
    def refresh_ports(self):
        sel = self.portCombo.currentText()
        self.portCombo.clear()
        ports = list_serial_ports()
        if self.default_port and self.default_port not in ports:
            ports.insert(0, self.default_port)
        self.portCombo.addItems(ports)
        if sel in ports:
            self.portCombo.setCurrentText(sel)
        elif self.default_port in ports:
            self.portCombo.setCurrentText(self.default_port)

    # ---------- Connect / Disconnect ----------
    def on_connect(self):
        if self.worker is not None:
            return
        port = self.portCombo.currentText().strip()
        if not port:
            self.log("[UI] No port selected"); return
        try:
            baud = int(self.baudEdit.text())
        except ValueError:
            self.log("[UI] Invalid baud"); return
        self.connStatus.setText("Connecting…"); QtWidgets.QApplication.processEvents()
        self.worker = SerialWorker(port, baud, self.proto)
        self.worker.rx_text.connect(self.on_rx_text)
        self.worker.rx_raw.connect(self.on_rx_raw)
        self.worker.error.connect(lambda e: self.log(f"[ERR] {e}"))
        self.worker.status.connect(lambda s: self.log(s))
        self.worker.connected_changed.connect(self.on_conn_changed)
        self.worker.start()

    def on_disconnect(self):
        w = self.worker
        if not w: return
        # Stop polling
        self.on_stop()
        try: w.rx_text.disconnect(self.on_rx_text)
        except Exception: pass
        try: w.rx_raw.disconnect(self.on_rx_raw)
        except Exception: pass
        try: w.connected_changed.disconnect(self.on_conn_changed)
        except Exception: pass
        try: w.error.disconnect()
        except Exception: pass
        try: w.status.disconnect()
        except Exception: pass
        try: w.stop()
        except Exception: pass
        try: w.wait(1000)
        except Exception: pass
        try: w.deleteLater()
        except Exception: pass
        self.worker = None
        self.on_conn_changed(False)
        self.log("[UI] Disconnected")

    def on_conn_changed(self, ok: bool):
        self.connectBtn.setEnabled(not ok)
        self.disconnectBtn.setEnabled(ok)
        self.portCombo.setEnabled(not ok)
        self.refreshBtn.setEnabled(not ok)
        self.baudEdit.setEnabled(not ok)
        self.startBtn.setEnabled(ok and not self._passive_active)
        self.stopBtn.setEnabled(ok and self._passive_active)
        self.saveBtn.setEnabled(bool(self.rows))
        self.connStatus.setText("Connected" if ok else "Disconnected")

    # ---------- Passive start/stop ----------
    def on_start(self):
        if self.worker is None:
            self.log("[UI] Not connected"); return
        if self._passive_active:
            return
        # Fresh buffers
        self._t0_epoch = None
        self.tsec.clear(); self.left.clear(); self.right.clear(); self.cj.clear(); self.actual.clear(); self.rows.clear()
        self.update_curves()
        # Kick immediate sample then periodic polling
        self._passive_active = True
        self.on_conn_changed(True)
        self.send_ascii("values")
        self.pollTimer.start(self.intervalSpin.value() * 1000)
        self.log("[PASSIVE] Datalogging started")

    def on_stop(self):
        if not self._passive_active:
            return
        try: self.pollTimer.stop()
        except Exception: pass
        self._passive_active = False
        self.on_conn_changed(self.worker is not None)
        self.log("[PASSIVE] Datalogging stopped")

    # ---------- RX/Decode ----------
    @QtCore.pyqtSlot(bytes)
    def on_rx_raw(self, data: bytes):
        # raw dump not shown to keep applet minimal
        pass

    @QtCore.pyqtSlot(str)
    def on_rx_text(self, line: str):
        s = line.strip()
        # Parse multi-line 'values' block:
        #   Actual measured values:
        #           Left: 27.0degC
        #          Right: 27.5degC
        #    Cold junction: 26.6degC
        if s.startswith("Actual measured values"):
            self._acc = {}
            self.log(line)
            return
        if s.startswith("Left:") or s.startswith("Right:") or s.startswith("Cold junction:"):
            self.log(line)
            try:
                v = float(s.replace("degC", "").split(":")[1])
            except Exception:
                v = None
            if s.startswith("Left:"):
                self._acc["left"] = v
            elif s.startswith("Right:"):
                self._acc["right"] = v
            else:
                self._acc["cj"] = v
            # If we have a full set, record a sample
            if all(k in self._acc for k in ("left", "right", "cj")):
                self.ingest_passive_sample(self._acc)
                self._acc = {}
            return
        # Show everything else in the log
        self.log(line)

    def ingest_passive_sample(self, acc: dict):
        try:
            l = float(acc.get("left"))
            r = float(acc.get("right"))
            cj = float(acc.get("cj"))
        except Exception:
            return
        act = (l + r) / 2.0
        now = datetime.now().timestamp()
        if self._t0_epoch is None:
            self._t0_epoch = now
        t = now - self._t0_epoch

        self.tsec.append(t); self.left.append(l); self.right.append(r); self.cj.append(cj); self.actual.append(act)
        # CSV row: timestamp_iso, t_sec, left, right, actual, coldJ
        self.rows.append((datetime.fromtimestamp(now).isoformat(timespec="milliseconds"),
                          float(t), l, r, act, cj))
        self.saveBtn.setEnabled(True)
        self.update_curves()
        self.ensure_x_range(t)

        # Small live-value display
        self.vLeft.setText(f"{l:.1f}")
        self.vRight.setText(f"{r:.1f}")
        self.vCJ.setText(f"{cj:.1f}")
        self.vAct.setText(f"{act:.1f}")
        self.vSet.setText("—")  # not known in passive mode

    # ---------- Plot helpers ----------
    def configure_plot_axes(self, fixed_x_max: int = 420):
        vb = self.plot.getViewBox()
        try: vb.enableAutoRange(x=False, y=False)
        except Exception: pass
        vb.setLimits(xMin=0, xMax=fixed_x_max, yMin=0, yMax=300)
        self.plot.setXRange(0, fixed_x_max, padding=0)
        self.plot.setYRange(0, 300, padding=0)
        vb.setAspectLocked(True, 300/float(fixed_x_max))

    def ensure_x_range(self, tsec: float):
        # Expand x-axis beyond 420s in passive mode
        if not self._passive_active:
            return
        if tsec <= 420:
            return
        xmax = int(math.ceil(tsec / 60.0) * 60)  # next full minute
        vb = self.plot.getViewBox()
        try: vb.enableAutoRange(x=False, y=False)
        except Exception: pass
        vb.setLimits(xMin=0, xMax=xmax, yMin=0, yMax=300)
        self.plot.setXRange(0, xmax, padding=0)
        self.plot.setYRange(0, 300, padding=0)
        try: vb.setAspectLocked(True, 300/float(xmax))
        except Exception: pass

    def update_curves(self):
        self.curveActual.setData(self.tsec, self.actual)
        self.curveLeft.setData(self.tsec, self.left)
        self.curveRight.setData(self.tsec, self.right)

    # ---------- TX ----------
    def send_ascii(self, line: str):
        if not self.worker:
            self.log("[UI] Not connected"); return
        self.worker.send(self.proto.encode(line))
        self.log(f"[TX] {line}")

    # ---------- Save CSV ----------
    def on_save_csv(self):
        if not self.rows:
            QtWidgets.QMessageBox.information(self, "No Data", "No samples recorded yet.")
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save CSV", "passive_log.csv", "CSV Files (*.csv)")
        if not path: return
        import csv
        try:
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["timestamp_iso", "t_sec", "left_c", "right_c", "actual_c", "coldj_c"])
                for row in self.rows:
                    w.writerow(row)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to save: {e}")
            return
        self.statusBar().showMessage(f"Saved: {path}", 4000)

    # ---------- Log ----------
    def log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.logBox.appendPlainText(f"[{ts}] {msg}")

    def on_reset_axes(self):
        """Reset plot to fixed 0..300 °C on Y and either 0..420 s or to current max time, whichever is larger."""
        xmax = 420
        if self.tsec:
            try:
                import math
                xmax = max(420, int(math.ceil(self.tsec[-1] / 60.0) * 60))
            except Exception:
                xmax = max(420, int(self.tsec[-1]))
        vb = self.plot.getViewBox()
        try: vb.enableAutoRange(x=False, y=False)
        except Exception: pass
        vb.setLimits(xMin=0, xMax=xmax, yMin=0, yMax=300)
        self.plot.setXRange(0, xmax, padding=0)
        self.plot.setYRange(0, 300, padding=0)
        try: vb.setAspectLocked(True, 300/float(xmax))
        except Exception: pass


def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setFont(QtGui.QFont("Helvetica", 11))
    pg.setConfigOptions(antialias=True)
    w = Datalogger()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
