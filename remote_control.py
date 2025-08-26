# reflow_profile_gui_pyqt6.py
# pip install PyQt6 pyqtgraph

from dataclasses import dataclass
from typing import Optional, List, Tuple
import sys

from PyQt6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

# --- pyserial ---
import serial
import serial.tools.list_ports as list_ports

from collections import deque
from datetime import datetime
from typing import Deque

# --------- Global App State (state machine) ---------
from typing import Literal

@dataclass
class AppState:
    conn: Literal['disconnected', 'connecting', 'connected'] = 'disconnected'
    mode: Literal['idle', 'manual', 'auto'] = 'idle'
    run_index: int = 0



# ---------------- Backend: profile math ----------------

# =========================
# Protocol Layer (pluggable)
# =========================

from dataclasses import dataclass
import struct
from typing import Optional, List

@dataclass
class ProtoConfig:
    mode: str = "ascii"          # "ascii" or "binary"
    eol: bytes = b"\n"           # for ascii mode
    header: bytes = b"RF"        # for binary mode
    use_crc16: bool = True       # for binary mode
    little_endian: bool = True   # for binary mode


def crc16_ibm(data: bytes, init: int = 0xFFFF) -> int:
    """CRC-16/IBM (aka CRC-16-ANSI), poly 0xA001, init FFFF."""
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


class Protocol:
    def __init__(self, cfg: ProtoConfig):
        self.cfg = cfg
        self.rx_buf = bytearray()

    # ---------- Encoding ----------
    def encode_ascii(self, line: str) -> bytes:
        return line.encode("utf-8") + self.cfg.eol

    def encode_binary(self, cmd: int, payload: bytes = b"") -> bytes:
        le = "<" if self.cfg.little_endian else ">"
        frame_wo_crc = self.cfg.header + struct.pack(le + "BH", cmd, len(payload)) + payload
        if self.cfg.use_crc16:
            crc = crc16_ibm(frame_wo_crc)
            frame = frame_wo_crc + struct.pack(le + "H", crc)
        else:
            frame = frame_wo_crc
        return frame

    # ---------- Decoding ----------
    def try_decode(self, incoming: bytes) -> List[str]:
        out: List[str] = []
        if self.cfg.mode == "ascii":
            self.rx_buf.extend(incoming)
            eol = self.cfg.eol
            while True:
                idx = self.rx_buf.find(eol)
                if idx < 0:
                    break
                line = self.rx_buf[:idx]
                del self.rx_buf[:idx + len(eol)]
                out.append(line.decode(errors="replace"))
            return out

        # binary mode (kept for future use)
        self.rx_buf.extend(incoming)
        le = "<" if self.cfg.little_endian else ">"
        hdr = self.cfg.header
        min_len = len(hdr) + 1 + 2
        while True:
            if len(self.rx_buf) < min_len:
                break
            hpos = self.rx_buf.find(hdr)
            if hpos < 0:
                self.rx_buf.clear()
                break
            if hpos > 0:
                del self.rx_buf[:hpos]
            if len(self.rx_buf) < min_len:
                break
            try:
                cmd = self.rx_buf[len(hdr)]
                plen = struct.unpack_from(le + "H", self.rx_buf, len(hdr) + 1)[0]
            except Exception:
                break
            total = len(hdr) + 1 + 2 + plen + (2 if self.cfg.use_crc16 else 0)
            if len(self.rx_buf) < total:
                break
            frame = bytes(self.rx_buf[:total])
            del self.rx_buf[:total]
            payload = frame[len(hdr)+3:len(hdr)+3+plen]
            if self.cfg.use_crc16:
                calc = crc16_ibm(frame[:-2])
                got = struct.unpack_from(le + "H", frame, total - 2)[0]
                crc_ok = (calc == got)
            else:
                crc_ok = True
            out.append(f"[BIN] cmd=0x{cmd:02X} len={plen} crc={'OK' if crc_ok else 'BAD'} payload={payload.hex()}")
        return out


# ========================================
# Serial I/O worker (runs on its own thread)
# ========================================
class SerialWorker(QtCore.QThread):
    rx_text = QtCore.pyqtSignal(str)
    rx_raw = QtCore.pyqtSignal(bytes)
    error = QtCore.pyqtSignal(str)
    status = QtCore.pyqtSignal(str)
    connected_changed = QtCore.pyqtSignal(bool)

    def __init__(self, port: str, baud: int, proto: Protocol, parent=None):
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self.proto = proto
        self._ser: Optional[serial.Serial] = None
        self._running = False
        self._tx_queue: Deque[bytes] = deque()
        self._lock = QtCore.QMutex()

    @QtCore.pyqtSlot(bytes)
    def send_bytes(self, data: bytes):
        with QtCore.QMutexLocker(self._lock):
            self._tx_queue.append(data)

    def run(self):
        try:
            self._ser = serial.Serial(port=self.port, baudrate=self.baud, timeout=0.05, write_timeout=0.5)
        except Exception as e:
            self.error.emit(f"Open failed: {e}")
            self.connected_changed.emit(False)
            return
        self.status.emit(f"Opened {self.port} @ {self.baud} baud")
        self.connected_changed.emit(True)
        self._running = True
        try:
            while self._running:
                to_write = None
                with QtCore.QMutexLocker(self._lock):
                    if self._tx_queue:
                        to_write = self._tx_queue.popleft()
                if to_write and self._ser:
                    try:
                        self._ser.write(to_write)
                    except Exception as e:
                        self.error.emit(f"Write error: {e}")
                try:
                    chunk = self._ser.read(4096) if self._ser else b""
                except Exception as e:
                    self.error.emit(f"Read error: {e}")
                    break
                if chunk:
                    self.rx_raw.emit(chunk)
                    for m in self.proto.try_decode(chunk):
                        self.rx_text.emit(m)
                else:
                    self.msleep(5)
        finally:
            try:
                if self._ser and self._ser.is_open:
                    self._ser.close()
            except Exception:
                pass
            self.connected_changed.emit(False)
            self.status.emit("Port closed")

    def stop(self):
        self._running = False

@dataclass
class Step:
    time: Optional[float] = None          # seconds (ramp duration)
    temperature: Optional[float] = None   # end temp (°C)
    rate: Optional[float] = None          # °C/s (can be negative)
    hold: float = 0.0                     # seconds (post-ramp flat)

def _solve_step(start_T: float, s: Step) -> Tuple[float, float, float]:
    """Compute (duration, end_temperature, rate) from any two fields."""
    provided = sum(x is not None for x in (s.time, s.temperature, s.rate))
    if provided != 2:
        raise ValueError("Each step must specify exactly TWO of: time, temperature, rate.")

    if s.time is not None and s.rate is not None:
        duration = float(s.time)
        rate = float(s.rate)
        end_T = start_T + rate * duration
    elif s.time is not None and s.temperature is not None:
        duration = float(s.time)
        end_T = float(s.temperature)
        if duration == 0:
            raise ValueError("Time cannot be zero when computing rate.")
        rate = (end_T - start_T) / duration
    elif s.temperature is not None and s.rate is not None:
        end_T = float(s.temperature)
        dT = end_T - start_T
        if s.rate == 0:
            raise ValueError("Rate cannot be zero when computing time.")
        duration = dT / float(s.rate)
        rate = float(s.rate)
    else:
        raise RuntimeError("Unexpected parameter combination.")

    if duration < 0:
        raise ValueError("Computed duration is negative; check signs.")
    return duration, end_T, rate

def build_profile(
    steps: List[Step],
    start_temperature: float = 25.0,
    total_duration: int = 420,  # 7 minutes
    sample_every: int = 10
) -> Tuple[List[int], List[Tuple[float, float]], List[Tuple[float, float]]]:
    """
    Returns:
      - 42-sample integer temps (every 10 s)
      - piecewise-linear polyline for ramps/holds: list of (t, T)
      - sample points polyline: list of (t, T)
    """
    # Build segments
    segs: List[Tuple[float, float, float]] = []  # (duration, T0, T1)
    holds_inline: List[float] = []               # hold durations paired to segs
    T = float(start_temperature)
    for s in steps:
        dur, end_T, _ = _solve_step(T, s)
        segs.append((dur, T, end_T))
        T = end_T
        holds_inline.append(float(s.hold or 0.0))

    # Flatten to timeline
    timeline: List[Tuple[float, float, float]] = []
    for (dur, T0, T1), h in zip(segs, holds_inline):
        if dur > 0:
            timeline.append((dur, T0, T1))
        if h > 0:
            timeline.append((h, T1, T1))

    # Pad/truncate to exactly total_duration
    built = sum(d for d, _, _ in timeline)
    if built < total_duration:
        timeline.append((total_duration - built, 0.0, 0.0))  # pad at 0 °C
    elif built > total_duration:
        excess = built - total_duration
        dur, T0, T1 = timeline[-1]
        new_dur = dur - excess
        if new_dur < 0:
            raise ValueError("Profile exceeds 420 s too much to trim.")
        frac = 1.0 if dur == 0 else new_dur / dur
        new_end = T0 + (T1 - T0) * frac
        timeline[-1] = (new_dur, T0, new_end)

    # Build polyline for the full profile
    poly: List[Tuple[float, float]] = [(0.0, start_temperature)]
    t = 0.0
    for dur, T0, T1 in timeline:
        if dur <= 0:
            continue
        t1 = t + dur
        poly.append((t1, T1))
        t = t1

    # Sample every 10 s
    samples: List[int] = []
    sample_poly: List[Tuple[float, float]] = []
    def interp_in_seg(t_query: float) -> float:
        # find segment containing t_query
        acc = 0.0
        Tcur = start_temperature
        for dur, T0, T1 in timeline:
            if dur <= 0:
                continue
            if acc + dur >= t_query:
                frac = (t_query - acc) / dur if dur > 0 else 1.0
                return T0 + (T1 - T0) * frac
            acc += dur
            Tcur = T1
        return Tcur  # after end

    for ts in range(sample_every, total_duration + 1, sample_every):
        Tnow = interp_in_seg(ts)
        sample_poly.append((ts, Tnow))
        samples.append(int(round(Tnow)))

    return samples, poly, sample_poly


# ---------------- GUI ----------------

class DoubleOrBlankDelegate(QtWidgets.QItemDelegate):
    """Allows blank cells or validated double values."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.validator = QtGui.QDoubleValidator(bottom=-1e12, top=1e12, decimals=3)

    def createEditor(self, parent, option, index):
        editor = QtWidgets.QLineEdit(parent)
        editor.setPlaceholderText("blank ok")
        editor.setValidator(self.validator)
        return editor

class StepsTable(QtWidgets.QTableWidget):
    HEADERS = ["Time (s)", "Temp (°C)", "Rate (°C/s)", "Hold (s)"]

    def __init__(self, parent=None):
        super().__init__(0, len(self.HEADERS), parent)
        self.setHorizontalHeaderLabels(self.HEADERS)
        self.setItemDelegate(DoubleOrBlankDelegate(self))
        hh = self.horizontalHeader()
        hh.setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Stretch)
        self.verticalHeader().setVisible(False)
        self.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.SingleSelection)
        self.setAlternatingRowColors(True)
        self.setEditTriggers(
            QtWidgets.QAbstractItemView.EditTrigger.DoubleClicked
            | QtWidgets.QAbstractItemView.EditTrigger.SelectedClicked
            | QtWidgets.QAbstractItemView.EditTrigger.EditKeyPressed
        )

    def add_row(self, step: Step = Step()):
        r = self.rowCount()
        self.insertRow(r)
        for c, val in enumerate([step.time, step.temperature, step.rate, step.hold]):
            it = QtWidgets.QTableWidgetItem("" if val is None else f"{val:g}")
            it.setTextAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            self.setItem(r, c, it)

    def remove_selected(self):
        rows = sorted({idx.row() for idx in self.selectedIndexes()}, reverse=True)
        for r in rows:
            self.removeRow(r)

    def move_selected(self, direction: int):
        rows = sorted({idx.row() for idx in self.selectedIndexes()})
        if not rows:
            return
        r = rows[0]
        new_r = r + direction
        if new_r < 0 or new_r >= self.rowCount():
            return
        self.insertRow(new_r)
        for c in range(self.columnCount()):
            self.setItem(new_r, c, self.takeItem(r + (1 if direction < 0 else 0), c))
        self.removeRow(r + (1 if direction > 0 else 0))
        self.selectRow(new_r)

    def read_steps(self) -> List[Step]:
        steps: List[Step] = []
        for r in range(self.rowCount()):
            vals: List[Optional[float]] = []
            for c in range(4):
                s = self.item(r, c).text().strip() if self.item(r, c) else ""
                if s == "":
                    vals.append(None if c < 3 else 0.0)  # hold defaults to 0
                else:
                    vals.append(float(s))
            time, temp, rate, hold = vals
            hold = 0.0 if hold is None else hold
            steps.append(Step(time=time, temperature=temp, rate=rate, hold=hold))
        return steps


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Reflow Profile Builder (PyQt6)")
        self.resize(1000, 700)
        self.state = AppState()

        # Target plot height (keeps graphs about half-height and prevents clipping)
        self.PLOT_MIN_H = 160
        self.PLOT_MAX_H = 240

        # Central layout
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)

        # Left pane: Profile builder
        leftWrap = QtWidgets.QWidget()
        leftLay = QtWidgets.QVBoxLayout(leftWrap)
        self.table = StepsTable()
        leftLay.addWidget(self.table)

        btns = QtWidgets.QHBoxLayout()
        leftLay.addLayout(btns)

        self.btnAdd = QtWidgets.QPushButton("Add Step")
        self.btnDel = QtWidgets.QPushButton("Remove Selected")
        self.btnUp = QtWidgets.QPushButton("Move Up")
        self.btnDown = QtWidgets.QPushButton("Move Down")
        self.btnBuild = QtWidgets.QPushButton("Build Profile")
        self.btnSaveCsv = QtWidgets.QPushButton("Save CSV…")
        self.btnLoadCsv = QtWidgets.QPushButton("Load CSV…")
        btns.addWidget(self.btnAdd)
        btns.addWidget(self.btnDel)
        btns.addWidget(self.btnUp)
        btns.addWidget(self.btnDown)
        btns.addStretch(1)
        btns.addWidget(self.btnSaveCsv)
        btns.addWidget(self.btnLoadCsv)
        btns.addWidget(self.btnBuild)

        # Plot + output
        split = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
        leftLay.addWidget(split, 1)
        # Keep plot sizes matched when the left splitter moves
        split.splitterMoved.connect(lambda *_: self.update_plot_sizes())

        self.plot = pg.PlotWidget(background="w")
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.setLabel("bottom", "Time (s)")
        self.plot.setLabel("left", "Temperature (°C)")
        # Fix axes and aspect to 420s × 300°C
        self.configure_plot_axes(self.plot, disable_mouse=True)
        # Add a small top margin to avoid visual clipping
        self.plot.plotItem.getViewBox().setDefaultPadding(0.02)
        # Apply fixed-height friendly size policy
        sp_fixed = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding,
                                         QtWidgets.QSizePolicy.Policy.Fixed)
        self.plot.setSizePolicy(sp_fixed)
        split.addWidget(self.plot)

        outputWrap = QtWidgets.QWidget()
        split.addWidget(outputWrap)
        outLay = QtWidgets.QVBoxLayout(outputWrap)
        outLay.addWidget(QtWidgets.QLabel("42-sample temperatures (10 s spacing):"))
        self.outText = QtWidgets.QPlainTextEdit()
        self.outText.setReadOnly(True)
        self.outText.setMaximumHeight(140)
        outLay.addWidget(self.outText)

        # Connections
        self.btnAdd.clicked.connect(self.on_add)
        self.btnDel.clicked.connect(self.table.remove_selected)
        self.btnUp.clicked.connect(lambda: self.table.move_selected(-1))
        self.btnDown.clicked.connect(lambda: self.table.move_selected(+1))
        self.btnBuild.clicked.connect(self.on_build)
        self.btnSaveCsv.clicked.connect(self.on_save_csv)
        self.btnLoadCsv.clicked.connect(self.on_load_csv)

        # Preload a sensible example
        self.table.add_row(Step(time=120, temperature=170))  # preheat
        self.table.add_row(Step(time=90,  temperature=217))  # soak
        self.table.add_row(Step(temperature=245, rate=0.5, hold=20))  # to peak + hold
        self.table.add_row(Step(temperature=75, rate=-3.0))  # cool

        self.on_build()


        # ==========================
        # Right pane container
        # ==========================
        rightWrap = QtWidgets.QWidget()
        rightLay = QtWidgets.QVBoxLayout(rightWrap)

        # --- Serial Console (collapsible header + body) ---
        # Header (always visible): toggle, title, status, connect/disconnect
        serialHeader = QtWidgets.QHBoxLayout()
        self.serialToggle = QtWidgets.QToolButton()
        self.serialToggle.setAutoRaise(True)
        self.serialToggle.setArrowType(QtCore.Qt.ArrowType.RightArrow)  # collapsed by default
        serialHeader.addWidget(self.serialToggle)
        serialHeader.addWidget(QtWidgets.QLabel("Serial Console"))
        serialHeader.addSpacing(8)
        self.connStatusLabel = QtWidgets.QLabel("Disconnected")
        serialHeader.addWidget(self.connStatusLabel)
        serialHeader.addStretch(1)
        self.connectBtn = QtWidgets.QPushButton("Connect")
        self.disconnectBtn = QtWidgets.QPushButton("Disconnect"); self.disconnectBtn.setEnabled(False)
        serialHeader.addWidget(self.connectBtn)
        serialHeader.addWidget(self.disconnectBtn)
        rightLay.addLayout(serialHeader)

        # Body (collapsible content): port/baud, transmit, commands, log
        self.serialContent = QtWidgets.QWidget()
        sLay = QtWidgets.QVBoxLayout(self.serialContent)
        sLay.setContentsMargins(0, 0, 0, 0)

        # Row: port/baud + refresh (connect/disconnect moved to header)
        rowConn = QtWidgets.QHBoxLayout()
        self.portCombo = QtWidgets.QComboBox(); self.portCombo.setEditable(True)
        self.refreshBtn = QtWidgets.QPushButton("Refresh")
        self.baudEdit = QtWidgets.QLineEdit("115200")
        self.baudEdit.setValidator(QtGui.QIntValidator(1200, 10000000))
        rowConn.addWidget(QtWidgets.QLabel("Port:")); rowConn.addWidget(self.portCombo, 2)
        rowConn.addWidget(self.refreshBtn)
        rowConn.addSpacing(10)
        rowConn.addWidget(QtWidgets.QLabel("Baud:")); rowConn.addWidget(self.baudEdit)
        rowConn.addStretch(1)
        sLay.addLayout(rowConn)

        # Transmit box
        txGroup = QtWidgets.QGroupBox("Transmit")
        txLay = QtWidgets.QVBoxLayout(txGroup)
        self.txEdit = QtWidgets.QLineEdit(); self.txHexChk = QtWidgets.QCheckBox("Hex input (spaces ok)")
        self.txAddNewline = QtWidgets.QCheckBox(r"Append '\n' (ASCII mode)"); self.txAddNewline.setChecked(True)
        self.sendBtn = QtWidgets.QPushButton("Send")
        txLay.addWidget(self.txEdit); txLay.addWidget(self.txHexChk); txLay.addWidget(self.txAddNewline); txLay.addWidget(self.sendBtn)
        sLay.addWidget(txGroup)

        # Quick commands
        cmdGroup = QtWidgets.QGroupBox("Commands (ASCII)")
        cmdLay = QtWidgets.QGridLayout(cmdGroup)
        self.btnAbout = QtWidgets.QPushButton("about"); self.btnValues = QtWidgets.QPushButton("values")
        self.btnStop = QtWidgets.QPushButton("stop")
        cmdLay.addWidget(self.btnAbout, 0, 0)
        cmdLay.addWidget(self.btnValues, 0, 1)
        cmdLay.addWidget(self.btnStop, 0, 2)
        sLay.addWidget(cmdGroup)

        # RX log
        self.rxLog = QtWidgets.QPlainTextEdit(); self.rxLog.setReadOnly(True)
        self.rxHexChk = QtWidgets.QCheckBox("Show raw hex")
        self.tsChk = QtWidgets.QCheckBox("Timestamp lines"); self.tsChk.setChecked(True)
        self.clearLogBtn = QtWidgets.QPushButton("Clear Log")
        sLay.addWidget(self.rxLog, 1)
        sLay.addWidget(self.rxHexChk)
        sLay.addWidget(self.tsChk)
        sLay.addWidget(self.clearLogBtn)

        rightLay.addWidget(self.serialContent)
        # collapsed by default
        self.serialContent.setVisible(False)

        # ==========================
        # Right pane: Bake Control
        # ==========================

        # Header + actions
        headerLay = QtWidgets.QHBoxLayout()
        headerLay.addWidget(QtWidgets.QLabel("Bake Control"))
        headerLay.addStretch(1)
        self.btnCopyTarget = QtWidgets.QPushButton("Copy from Profile → Target Temperature")
        headerLay.addWidget(self.btnCopyTarget)
        headerLay.addSpacing(12)
        headerLay.addWidget(QtWidgets.QLabel("Step time (s):"))
        self.stepSeconds = QtWidgets.QSpinBox()
        self.stepSeconds.setRange(1, 600)
        self.stepSeconds.setValue(10)
        headerLay.addWidget(self.stepSeconds)
        self.btnRunBuilt = QtWidgets.QPushButton("Run Built Profile")
        self.btnAbortRun = QtWidgets.QPushButton("Abort")
        self.btnAbortRun.setEnabled(False)
        headerLay.addWidget(self.btnRunBuilt)
        headerLay.addWidget(self.btnAbortRun)
        rightLay.addLayout(headerLay)

        # --- Manual Control ---
        manualGroup = QtWidgets.QGroupBox("Manual Control")
        mlay = QtWidgets.QHBoxLayout(manualGroup)
        mlay.addWidget(QtWidgets.QLabel("Setpoint (°C):"))
        self.manualSetpoint = QtWidgets.QSpinBox()
        self.manualSetpoint.setRange(0, 300)
        self.manualSetpoint.setValue(25)
        mlay.addWidget(self.manualSetpoint)
        mlay.addStretch(1)
        self.btnManualStart = QtWidgets.QPushButton("Start / Update")
        self.btnManualStop = QtWidgets.QPushButton("Stop Manual")
        self.btnManualStop.setEnabled(False)
        mlay.addWidget(self.btnManualStart)
        mlay.addWidget(self.btnManualStop)
        rightLay.addWidget(manualGroup)

        # Live telemetry labels
        tele = QtWidgets.QGroupBox("Live Telemetry")
        tlay = QtWidgets.QGridLayout(tele)
        def mkbig(lbl: str):
            lab = QtWidgets.QLabel(lbl); f = lab.font(); f.setPointSize(12); f.setBold(True); lab.setFont(f)
            lab.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight | QtCore.Qt.AlignmentFlag.AlignVCenter)
            return lab
        self.tSet = mkbig("—"); self.tActual = mkbig("—"); self.tCJ = mkbig("—")
        self.tHeat = mkbig("—"); self.tFan = mkbig("—"); self.tMode = mkbig("—")
        self.t0 = mkbig("—"); self.t1 = mkbig("—"); self.t2 = mkbig("—"); self.t3 = mkbig("—")
        row = 0
        tlay.addWidget(QtWidgets.QLabel("Set (°C):"), row, 0); tlay.addWidget(self.tSet, row, 1); row += 1
        tlay.addWidget(QtWidgets.QLabel("Actual (°C):"), row, 0); tlay.addWidget(self.tActual, row, 1); row += 1
        tlay.addWidget(QtWidgets.QLabel("Cold Jct (°C):"), row, 0); tlay.addWidget(self.tCJ, row, 1); row += 1
        tlay.addWidget(QtWidgets.QLabel("Heat:"), row, 0); tlay.addWidget(self.tHeat, row, 1); row += 1
        tlay.addWidget(QtWidgets.QLabel("Fan:"), row, 0); tlay.addWidget(self.tFan, row, 1); row += 1
        tlay.addWidget(QtWidgets.QLabel("Mode:"), row, 0); tlay.addWidget(self.tMode, row, 1); row += 1
        tlay.addWidget(QtWidgets.QLabel("Temp0 (°C):"), row, 0); tlay.addWidget(self.t0, row, 1); row += 1
        tlay.addWidget(QtWidgets.QLabel("Temp1 (°C):"), row, 0); tlay.addWidget(self.t1, row, 1); row += 1
        tlay.addWidget(QtWidgets.QLabel("Temp2 (°C):"), row, 0); tlay.addWidget(self.t2, row, 1); row += 1
        tlay.addWidget(QtWidgets.QLabel("Temp3 (°C):"), row, 0); tlay.addWidget(self.t3, row, 1); row += 1
        rightLay.addWidget(tele)
        # Prevent the telemetry section from stretching vertically
        tele.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding,
                           QtWidgets.QSizePolicy.Policy.Fixed)
        # Lock its max height to its size hint (plus a tiny buffer)
        tele.setMaximumHeight(tele.sizeHint().height() + 8)

        # Telemetry plot
        self.telePlot = pg.PlotWidget(background="w")
        self.configure_plot_axes(self.telePlot, disable_mouse=False)
        self.telePlot.showGrid(x=True, y=True, alpha=0.25)
        self.telePlot.setLabel("bottom", "Time (s)")
        self.telePlot.setLabel("left", "Temperature (°C)")
        # Add a small top margin to avoid visual clipping
        self.telePlot.plotItem.getViewBox().setDefaultPadding(0.02)
        # Apply fixed-height friendly size policy
        self.telePlot.setSizePolicy(sp_fixed)
        # Legend and curves
        if not self.telePlot.plotItem.legend:
            self.telePlot.addLegend()
        self.curveActual = self.telePlot.plot([], [], pen=pg.mkPen('r', width=2), name="Actual")
        self.curveTargetLine = self.telePlot.plot([], [], pen=pg.mkPen(width=2, style=QtCore.Qt.PenStyle.DashLine), name="Target Temperature")
        self.curveTargetPts = self.telePlot.plot([], [], pen=None, symbol="o", symbolSize=6, name="Target points")
        # Re-apply axes after legend/curves are attached to prevent any autorange surprises
        self.configure_plot_axes(self.telePlot, disable_mouse=False)
        rightLay.addWidget(self.telePlot, 1)
        # Secondary right Y-axis (0..255) for controller outputs (Heat/Fan)
        self.telePlot.plotItem.showAxis('right')
        self.telePlot.plotItem.getAxis('right').setLabel(text="Output")
        self.vb2 = pg.ViewBox()
        self.telePlot.plotItem.scene().addItem(self.vb2)
        self.telePlot.plotItem.getAxis('right').linkToView(self.vb2)
        self.vb2.setXLink(self.telePlot.plotItem.vb)
        # Create Heat/Fan curves on secondary viewbox
        self.curveHeat = pg.PlotDataItem([], [], pen=pg.mkPen((255, 100, 0), width=1), name="Heater")
        self.curveFan  = pg.PlotDataItem([], [], pen=pg.mkPen((0, 100, 255), width=1), name="Fan")
        self.vb2.addItem(self.curveHeat)
        self.vb2.addItem(self.curveFan)
        # Keep secondary viewbox aligned
        self.telePlot.plotItem.vb.sigResized.connect(self.update_secondary_viewbox)
        QtCore.QTimer.singleShot(0, self.update_secondary_viewbox)

        # Additional curves (initially empty)
        self.curveSet = self.telePlot.plot([], [], pen=pg.mkPen('k', width=1), name="Set")
        self.curveT0 = self.telePlot.plot([], [], pen=pg.mkPen((50, 100, 200), width=1), name="Temp0")
        self.curveT1 = self.telePlot.plot([], [], pen=pg.mkPen((0, 150, 0), width=1), name="Temp1")
        self.curveT2 = self.telePlot.plot([], [], pen=pg.mkPen((200, 120, 0), width=1), name="Temp2")
        self.curveT3 = self.telePlot.plot([], [], pen=pg.mkPen((120, 0, 180), width=1), name="Temp3")

        # Series display controls
        seriesGroup = QtWidgets.QGroupBox("Series Display")
        srl = QtWidgets.QGridLayout(seriesGroup)
        self.chkShowActual = QtWidgets.QCheckBox("Actual")
        self.chkShowActual.setChecked(True)
        self.chkShowTarget = QtWidgets.QCheckBox("Target")
        self.chkShowTarget.setChecked(True)
        self.chkShowSet = QtWidgets.QCheckBox("Setpoint")
        self.chkShowSet.setChecked(False)
        self.chkShowT0 = QtWidgets.QCheckBox("Temp0")
        self.chkShowT1 = QtWidgets.QCheckBox("Temp1")
        self.chkShowT2 = QtWidgets.QCheckBox("Temp2")
        self.chkShowT3 = QtWidgets.QCheckBox("Temp3")
        self.chkShowHeat = QtWidgets.QCheckBox("Heater")
        self.chkShowFan  = QtWidgets.QCheckBox("Fan")
        # layout
        srl.addWidget(self.chkShowActual, 0, 0)
        srl.addWidget(self.chkShowTarget, 0, 1)
        srl.addWidget(self.chkShowSet,    0, 2)
        srl.addWidget(self.chkShowT0,     1, 0)
        srl.addWidget(self.chkShowT1,     1, 1)
        srl.addWidget(self.chkShowT2,     1, 2)
        srl.addWidget(self.chkShowT3,     1, 3)
        srl.addWidget(self.chkShowHeat, 2, 0)
        srl.addWidget(self.chkShowFan,  2, 1)
        rightLay.addWidget(seriesGroup)

        # Download telemetry CSV button
        dlRow = QtWidgets.QHBoxLayout()
        self.btnSaveTelemetry = QtWidgets.QPushButton("Download Telemetry CSV…")
        dlRow.addStretch(1)
        dlRow.addWidget(self.btnSaveTelemetry)
        rightLay.addLayout(dlRow)

        rightLay.addStretch(1)

        # Add stretch at end of LEFT pane so extra space is pushed below (prevents plot from stretching)
        leftLay.addStretch(1)

        # ===============
        # Root 2-way split (left builder + right console+bake)
        # ===============
        splitLR = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        splitLR.addWidget(leftWrap)
        splitLR.addWidget(rightWrap)
        splitLR.setStretchFactor(0, 3)
        splitLR.setStretchFactor(1, 4)
        root.addWidget(splitLR)
        self.splitLR = splitLR
        # Keep plot sizes matched when splitters move
        splitLR.splitterMoved.connect(lambda *_: self.update_plot_sizes())


        # Serial defaults
        self.default_port = "/dev/tty.usbserial-BG01OX7L"
        self.default_baud = 115200
        self.proto_cfg = ProtoConfig(mode="ascii", eol=b"\n")
        self.proto = Protocol(self.proto_cfg)
        self.worker: Optional[SerialWorker] = None

        # Signals
        self.refreshBtn.clicked.connect(self.refresh_ports)
        self.connectBtn.clicked.connect(self.on_connect)
        self.disconnectBtn.clicked.connect(self.on_disconnect)
        self.sendBtn.clicked.connect(self.on_send_clicked)
        self.clearLogBtn.clicked.connect(self.rxLog.clear)
        self.btnAbout.clicked.connect(lambda: self.send_ascii("about"))
        self.btnValues.clicked.connect(lambda: self.send_ascii("values"))
        self.btnStop.clicked.connect(lambda: self.send_ascii("stop"))
        self.txEdit.returnPressed.connect(self.on_send_clicked)
        self.btnCopyTarget.clicked.connect(self.on_copy_target)
        self.serialToggle.clicked.connect(self.on_serial_toggle)
        self.btnManualStart.clicked.connect(self.on_manual_start)
        self.btnManualStop.clicked.connect(self.on_manual_stop)
        self.btnSaveTelemetry.clicked.connect(self.on_save_telemetry_csv)
        # Series visibility toggles
        for chk in (self.chkShowActual, self.chkShowTarget, self.chkShowSet,
                    self.chkShowT0, self.chkShowT1, self.chkShowT2, self.chkShowT3,
                    self.chkShowHeat, self.chkShowFan):
            chk.toggled.connect(self.update_series_visibility)
        # Ensure UI reflects initial disconnected state on startup
        self.set_connected_enabled(False)
        # Ensure serial/bake init runs even if the Serial panel stays collapsed
        self.init_serial_once()
        self.apply_state()
    def on_serial_toggle(self):
        vis = self.serialContent.isVisible()
        self.serialContent.setVisible(not vis)
        self.serialToggle.setArrowType(
            QtCore.Qt.ArrowType.DownArrow if not vis else QtCore.Qt.ArrowType.RightArrow
        )
        # keep plots sized nicely after the layout change
        QtCore.QTimer.singleShot(0, self.update_plot_sizes)
        # Run one-time init if not yet done
        self.init_serial_once()

    def init_serial_once(self):
        """Run serial/bake one-time initialization and attempt auto-connect if not connected."""
        if getattr(self, '_serial_inited', False):
            return
        self._serial_inited = True
        # Bake profile runner
        self.btnRunBuilt.clicked.connect(self.on_run_built_profile)
        self.btnAbortRun.clicked.connect(self.on_abort_run_profile)
        self.profileTimer = QtCore.QTimer(self)
        self.profileTimer.timeout.connect(self.on_profile_tick)
        self._run_queue: List[int] = []
        self._run_index: int = 0

        # Telemetry parsing state (must be set during init)
        self.telemetry_active = False
        self.telemetry_cols: dict[str, int] = {}
        self.telemetry_colnames: List[str] = []
        self.t0s: List[float] = []  # time axis (secs since first point)
        self.actuals: List[float] = []
        self.setpoints: List[float] = []
        self.t0_list: List[float] = []
        self.t1_list: List[float] = []
        self.t2_list: List[float] = []
        self.t3_list: List[float] = []
        self.heat_list: List[Optional[int]] = []
        self.fan_list:  List[Optional[int]] = []
        # Full-row buffer for CSV export
        self.teleRows: List[Tuple[str, float, Optional[float], Optional[float], Optional[float], Optional[int], Optional[int], Optional[str], Optional[float], Optional[float], Optional[float], Optional[float]]] = []

        # Init ports and size sync
        self.refresh_ports()
        QtCore.QTimer.singleShot(0, self.update_plot_sizes)

        # Reflect current state instead of forcing disabled
        self.set_connected_enabled(self.worker is not None)
        if self.worker is not None:
            self.connStatusLabel.setText("Connected")
        else:
            # Try auto-connect only if not already connected
            QtCore.QTimer.singleShot(0, self.on_connect)

    def set_connected_enabled(self, connected: bool):
        # Ignore the boolean; authoritative source is self.state
        self.apply_state()
    def build_run_queue_from_profile(self) -> List[int]:
        """Return a list of integer setpoints derived from the last built profile.
        Falls back to the 'Target Temperature' copied series if available.
        """
        ts = getattr(self, 'last_target_ts', [])
        Ts = getattr(self, 'last_target_Ts', [])
        if not ts or not Ts:
            return []
        return [int(round(v)) for v in Ts]

    def on_run_built_profile(self):
        if self.worker is None:
            self.log("[UI] Not connected")
            return
        temps = self.build_run_queue_from_profile()
        if not temps:
            QtWidgets.QMessageBox.information(self, "No Profile", "Build a profile on the left first.")
            return
        # Auto-copy the target series onto the telemetry plot
        ts = getattr(self, 'last_target_ts', [])
        Ts = getattr(self, 'last_target_Ts', [])
        if ts and Ts:
            self.curveTargetLine.setData(ts, Ts)
            self.curveTargetPts.setData(ts, Ts)
            self.configure_plot_axes(self.telePlot, disable_mouse=False)
        step_s = int(self.stepSeconds.value())
        if step_s <= 0:
            self.log("[UI] Step seconds must be > 0")
            return
        self._run_queue = temps
        self._run_index = 0
        self.transition('START_AUTO')
        # Kick off immediately then every step_s seconds
        self.on_profile_tick()
        self.profileTimer.start(step_s * 1000)
        self.log(f"[RUN] Built profile: {len(temps)} steps @ {step_s}s each")

    def on_abort_run_profile(self):
        self.transition('ABORT_AUTO')
        if self.state.mode != 'auto':
            return
        self.profileTimer.stop()
        try:
            self.send_ascii("stop")
        except Exception:
            pass
        self.log("[RUN] Aborted")

    def on_manual_start(self):
        """Enter or update Manual bake mode at the requested setpoint.
        Cancels any automatic run in progress to avoid interference.
        """
        if self.worker is None:
            self.log("[UI] Not connected")
            return
        # Cancel automatic run if active
        if self.state.mode == 'auto':
            self.on_abort_run_profile()
        sp = int(self.manualSetpoint.value())
        try:
            # Continuous bake at setpoint (no time argument)
            self.send_ascii(f"bake {sp}")
            self.transition('START_MANUAL')
        except Exception as e:
            self.log(f"[ERR] {e}")
            return
        # Reset live telemetry buffers/epoch so manual run shows a fresh plot
        self._t0_epoch = None
        self.t0s.clear()
        self.actuals.clear()
        # Clear target overlays (manual uses controller setpoint instead)
        self.curveTargetLine.setData([], [])
        self.curveTargetPts.setData([], [])
        self.curveActual.setData([], [])
        # Keep axes fixed and allow zoom
        self.configure_plot_axes(self.telePlot, disable_mouse=False)
        self.log(f"[MANUAL] Bake setpoint -> {sp}°C")

    def on_manual_stop(self, log_only: bool = False):
        """Exit Manual bake mode (sends 'stop' unless log_only)."""
        if not log_only:
            try:
                self.send_ascii("stop")
            except Exception:
                pass
        self.transition('STOP_MANUAL')
        # Keep axes stable (optional but keeps behavior consistent)
        self.configure_plot_axes(self.telePlot, disable_mouse=False)
        if not log_only:
            self.log("[MANUAL] Stopped")

    def on_profile_tick(self):
        if self.state.mode != 'auto':
            return
        if self._run_index >= len(self._run_queue):
            self.profileTimer.stop()
            try:
                self.send_ascii("stop")
            except Exception:
                pass
            self.transition('AUTO_COMPLETE')
            self.log("[RUN] Complete")
            return
        temp = self._run_queue[self._run_index]
        step_s = int(self.stepSeconds.value())
        self.send_ascii(f"bake {temp} {step_s}")
        self._run_index += 1
        self.transition('AUTO_STEP_TICK', index=self._run_index)

    def configure_plot_axes(self, pw: pg.PlotWidget, disable_mouse: bool = True):
        """Set 0..420 s (x), 0..300 °C (y), lock aspect. If disable_mouse is True,
        turn off pan/zoom; otherwise enable wheel zoom."""
        vb = pw.getViewBox()
        # Disable autorange so our ranges stick even before/after data arrives
        try:
            vb.enableAutoRange(x=False, y=False)
        except Exception:
            pass
        # Fixed axis limits and displayed range
        vb.setLimits(xMin=0, xMax=420, yMin=0, yMax=300)
        pw.setXRange(0, 420, padding=0)
        pw.setYRange(0, 300, padding=0)
        # Lock aspect so visual ratio is identical across plots (y/x scaling)
        vb.setAspectLocked(True, 300/420)
        # Mouse interactions
        if disable_mouse:
            pw.setMouseEnabled(x=False, y=False)
            pw.setMenuEnabled(False)
        else:
            pw.setMouseEnabled(x=True, y=True)  # wheel zoom + drag
            pw.setMenuEnabled(False)
        # If this is the telemetry plot and we have a secondary view, lock its Y-range too
        if hasattr(self, 'telePlot') and pw is self.telePlot and hasattr(self, 'vb2'):
            try:
                self.vb2.enableAutoRange(y=False)
            except Exception:
                pass
            self.vb2.setYRange(0, 255, padding=0)

    def update_secondary_viewbox(self):
        """Keep right-side output axis aligned and fixed to 0..255."""
        if not hasattr(self, 'vb2'):
            return
        # Match geometry
        self.vb2.setGeometry(self.telePlot.plotItem.vb.sceneBoundingRect())
        self.vb2.linkedViewChanged(self.telePlot.plotItem.vb, self.vb2.XAxis)
        # Lock Y for outputs
        try:
            self.vb2.enableAutoRange(y=False)
        except Exception:
            pass
        self.vb2.setYRange(0, 255, padding=0)
    
    def apply_state(self):
        """Derive all UI enablement from the single source of truth: self.state."""
        is_conn = (self.state.conn == 'connected')
        is_conn_or_connecting = self.state.conn in ('connected', 'connecting')

        # Connection widgets
        self.connectBtn.setEnabled(self.state.conn == 'disconnected')
        self.disconnectBtn.setEnabled(is_conn)
        self.portCombo.setEnabled(self.state.conn == 'disconnected')
        self.refreshBtn.setEnabled(self.state.conn == 'disconnected')
        self.baudEdit.setEnabled(self.state.conn == 'disconnected')
        self.connStatusLabel.setText(
            'Connected' if is_conn else ('Connecting…' if self.state.conn == 'connecting' else 'Disconnected')
        )

        # Transmit / commands
        for w in (self.txEdit, self.txHexChk, self.txAddNewline, self.sendBtn,
                self.btnAbout, self.btnValues, self.btnStop):
            w.setEnabled(is_conn)

        # Bake control
        auto_active = (self.state.mode == 'auto')
        manual_active = (self.state.mode == 'manual')
        self.btnCopyTarget.setEnabled(is_conn)
        self.stepSeconds.setEnabled(is_conn and not manual_active)
        self.btnRunBuilt.setEnabled(is_conn and not manual_active and not auto_active)
        self.btnAbortRun.setEnabled(is_conn and auto_active)

        # Manual control
        self.manualSetpoint.setEnabled(is_conn and not auto_active)
        self.btnManualStart.setEnabled(is_conn and not auto_active)
        self.btnManualStop.setEnabled(is_conn and manual_active)

    def transition(self, event: str, **kwargs):
        """Centralized state transitions. Updates self.state and calls apply_state()."""
        s = self.state
        if event == 'CONNECT_REQUESTED':
            s.conn = 'connecting'
        elif event == 'CONNECTED':
            s.conn = 'connected'
        elif event == 'CONNECT_FAILED':
            s.conn = 'disconnected'
        elif event == 'DISCONNECT_REQUESTED':
            s.conn = 'disconnected'; s.mode = 'idle'; s.run_index = 0
        elif event == 'DISCONNECTED':
            s.conn = 'disconnected'; s.mode = 'idle'; s.run_index = 0
        elif event == 'START_MANUAL':
            s.mode = 'manual'
        elif event == 'STOP_MANUAL':
            s.mode = 'idle'
        elif event == 'START_AUTO':
            s.mode = 'auto'; s.run_index = 0
        elif event == 'AUTO_STEP_TICK':
            s.run_index = kwargs.get('index', s.run_index)
        elif event == 'AUTO_COMPLETE':
            s.mode = 'idle'; s.run_index = 0
        elif event == 'ABORT_AUTO':
            s.mode = 'idle'; s.run_index = 0
        self.apply_state()

    def update_plot_sizes(self):
        """Force both plots to the same pixel height using aspect 300/420, but cap the height
        so the graphs are roughly half the previous size and avoid stretching other widgets."""
        try:
            ratio = 300 / 420.0
            w_left = max(1, self.plot.width())
            w_right = max(1, self.telePlot.width())
            common_w = min(w_left, w_right)
            # Ideal height from aspect
            ideal_h = int(common_w * ratio)
            # Clamp to comfortable bounds to prevent clipping and layout stretch
            target_h = max(self.PLOT_MIN_H, min(self.PLOT_MAX_H, ideal_h))
            for pw in (self.plot, self.telePlot):
                pw.setFixedHeight(target_h)
        except Exception:
            pass

    def resizeEvent(self, ev: QtGui.QResizeEvent):
        super().resizeEvent(ev)
        # Defer until layout settles
        QtCore.QTimer.singleShot(0, self.update_plot_sizes)

    def showEvent(self, ev: QtGui.QShowEvent):
        super().showEvent(ev)
        # After the window is visible and layouts finalized, lock both plots again
        QtCore.QTimer.singleShot(0, lambda: (
            self.configure_plot_axes(self.plot, disable_mouse=True),
            self.configure_plot_axes(self.telePlot, disable_mouse=False)
        ))

    # Removed middle-pane collapse utilities (on_mid_toggle, set_mid_collapsed)

    def on_add(self):
        self.table.add_row()

    def on_build(self):
        try:
            steps = self.table.read_steps()
            temps, poly, sample_poly = build_profile(steps, start_temperature=25.0)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", str(e))
            return

        # Update left plot
        self.plot.clear()
        if poly:
            t_poly, T_poly = zip(*poly)
            self.plot.plot(t_poly, T_poly, pen=pg.mkPen(width=2), name="Profile")
        if sample_poly:
            ts, Ts = zip(*sample_poly)
            self.plot.plot(ts, Ts, pen=None, symbol="o", symbolSize=6, name="Samples")
            self.last_target_ts, self.last_target_Ts = list(ts), list(Ts)
        else:
            self.last_target_ts, self.last_target_Ts = [], []

        # Keep axes fixed
        self.configure_plot_axes(self.plot, disable_mouse=True)
        # Update text output
        self.outText.setPlainText(str(temps))
    def on_save_csv(self):
        """Export the current steps table to a CSV file.
        Columns: time_s, temp_c, rate_cps, hold_s. Empty cells are saved as blank.
        """
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save Profile CSV", "profile.csv", "CSV Files (*.csv)"
        )
        if not path:
            return
        try:
            steps = self.table.read_steps()
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Can't read steps: {e}")
            return
        import csv
        try:
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["time_s", "temp_c", "rate_cps", "hold_s"])  # °C/s in rate_cps
                for s in steps:
                    w.writerow([
                        "" if s.time is None else s.time,
                        "" if s.temperature is None else s.temperature,
                        "" if s.rate is None else s.rate,
                        s.hold or 0.0,
                    ])
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to save CSV: {e}")
            return
        self.statusBar().showMessage(f"Saved: {path}", 4000)

    def on_load_csv(self):
        """Load a steps table from a CSV file. Accepts 3-4 columns with header optional.
        Columns (by position): time_s, temp_c, rate_cps, hold_s. Blanks/"none" -> None (except hold->0).
        """
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Load Profile CSV", "", "CSV Files (*.csv)"
        )
        if not path:
            return
        import csv
        rows: List[Step] = []
        try:
            with open(path, "r", newline="") as f:
                r = csv.reader(f)
                header = next(r, None)  # tolerate missing/mismatched header
                for rec in r:
                    if not rec:
                        continue
                    while len(rec) < 4:
                        rec.append("")
                    def parse_opt(s: str):
                        s = s.strip()
                        if s == "" or s.lower() == "none":
                            return None
                        try:
                            return float(s)
                        except ValueError:
                            return None
                    t = parse_opt(rec[0])
                    temp = parse_opt(rec[1])
                    rate = parse_opt(rec[2])
                    try:
                        hold = 0.0 if rec[3].strip() == "" else float(rec[3])
                    except ValueError:
                        hold = 0.0
                    rows.append(Step(time=t, temperature=temp, rate=rate, hold=hold))
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to load CSV: {e}")
            return
        # Populate table and rebuild
        self.table.setRowCount(0)
        for s in rows:
            self.table.add_row(s)
        self.on_build()
        self.statusBar().showMessage(f"Loaded: {path}", 4000)
    def on_copy_target(self):
        """Copy the last built profile samples to the right plot as the Target Temperature series."""
        ts = getattr(self, 'last_target_ts', [])
        Ts = getattr(self, 'last_target_Ts', [])
        if not ts or not Ts:
            QtWidgets.QMessageBox.information(self, "No Profile", "Build a profile on the left first.")
            return
        # Draw on telemetry plot: dashed line + points matching left style
        self.curveTargetLine.setData(ts, Ts)
        self.curveTargetPts.setData(ts, Ts)
        # Keep axes fixed and allow zoom on right plot
        self.configure_plot_axes(self.telePlot, disable_mouse=False)
        # Keep axes fixed
        self.configure_plot_axes(self.telePlot, disable_mouse=False)
        self.update_series_visibility()

    # -------- Serial helpers --------
    def log(self, text: str):
        if self.tsChk.isChecked():
            now = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            self.rxLog.appendPlainText(f"[{now}] {text}")
        else:
            self.rxLog.appendPlainText(text)

    def refresh_ports(self):
        sel = self.portCombo.currentText()
        self.portCombo.clear()
        ports = [p.device for p in list_ports.comports()]
        if self.default_port and self.default_port not in ports:
            ports.insert(0, self.default_port)
        self.portCombo.addItems(ports)
        if sel in ports:
            self.portCombo.setCurrentText(sel)
        elif self.default_port in ports:
            self.portCombo.setCurrentText(self.default_port)

    def on_connect(self):
        if self.worker is not None:
            return
        port = self.portCombo.currentText().strip()
        if not port:
            self.log("[UI] No port selected")
            return
        try:
            baud = int(self.baudEdit.text())
        except ValueError:
            self.log("[UI] Invalid baud")
            return
        self.transition('CONNECT_REQUESTED')
        QtWidgets.QApplication.processEvents()
        self.worker = SerialWorker(port, baud, self.proto)
        self.worker.rx_text.connect(self.on_rx_text)
        self.worker.rx_raw.connect(self.on_rx_raw)
        self.worker.error.connect(lambda e: self.log(f"[ERR] {e}"))
        self.worker.status.connect(lambda s: self.log(s))
        self.worker.connected_changed.connect(self.on_conn_changed)
        self.worker.start()

    def on_disconnect(self):
        self.transition('DISCONNECT_REQUESTED')
        w = self.worker
        if not w:
            return
        # Stop any active run BEFORE stopping the worker
        if hasattr(self, 'profileTimer'):
            try:
                self.profileTimer.stop()
            except Exception:
                pass

        # Detach signals to prevent late emissions into deleted UI during teardown
        try:
            w.rx_text.disconnect(self.on_rx_text)
        except Exception:
            pass
        try:
            w.rx_raw.disconnect(self.on_rx_raw)
        except Exception:
            pass
        try:
            w.error.disconnect()
        except Exception:
            pass
        try:
            w.status.disconnect()
        except Exception:
            pass
        try:
            w.connected_changed.disconnect(self.on_conn_changed)
        except Exception:
            pass

        # Ask the thread to stop and wait for clean exit
        try:
            w.stop()
        except Exception:
            pass
        try:
            w.wait(1500)
        except Exception:
            pass
        try:
            w.deleteLater()
        except Exception:
            pass

        # Clear reference only after the thread is fully stopped
        self.worker = None

        # Update UI state
        self.transition('DISCONNECTED')
        self.log("[UI] Disconnected")

    def on_conn_changed(self, ok: bool):
        self.transition('CONNECTED' if ok else 'DISCONNECTED')

    @QtCore.pyqtSlot(bytes)
    def on_rx_raw(self, data: bytes):
        if self.rxHexChk.isChecked():
            import binascii
            self.log(binascii.hexlify(data).decode("ascii"))

    def send_ascii(self, line: str):
        if not self.worker:
            self.log("[UI] Not connected")
            return
        data = self.proto.encode_ascii(line)
        self.worker.send_bytes(data)
        self.log(f"[TX] {line}")

    def on_send_clicked(self):
        if not self.worker:
            self.log("[UI] Not connected")
            return
        txt = self.txEdit.text().strip()
        if self.txHexChk.isChecked():
            try:
                payload = bytes.fromhex(txt.replace(" ", ""))
            except ValueError:
                self.log("[UI] Hex parse error")
                return
            self.worker.send_bytes(payload)
            self.txEdit.clear()
        else:
            if self.txAddNewline.isChecked() and not txt.endswith("\n"):
                data = self.proto.encode_ascii(txt)
            else:
                data = txt.encode("utf-8")
            self.worker.send_bytes(data)
            self.txEdit.clear()

    @QtCore.pyqtSlot(str)
    def on_rx_text(self, line: str):
        sline = line.strip()
        # Telemetry header or auto-bootstrap
        if sline.startswith("# Time,"):
            hdr = [h.strip() for h in sline.lstrip('#').split(',')]
            self.telemetry_colnames = [h.strip() for h in hdr]
            self.telemetry_cols = {name: idx for idx, name in enumerate(self.telemetry_colnames)}
            self.telemetry_active = True
        elif ',' in sline and not sline.startswith('#'):
            parts = [p.strip() for p in sline.split(',')]
            # If already active and counts match, just update
            if self.telemetry_active and self.telemetry_colnames and len(parts) >= len(self.telemetry_colnames):
                self.update_telemetry_from_parts(parts)
            else:
                # Try to bootstrap if header wasn't seen (common when switching to Manual quickly)
                default_hdr = ["Time", "Temp0", "Temp1", "Temp2", "Temp3", "Set", "Actual", "Heat", "Fan", "ColdJ", "Mode"]
                if len(parts) >= len(default_hdr):
                    self.telemetry_colnames = default_hdr
                    self.telemetry_cols = {name: idx for idx, name in enumerate(self.telemetry_colnames)}
                    self.telemetry_active = True
                    self.update_telemetry_from_parts(parts)
        if not self.rxHexChk.isChecked():
            self.log(line)

    def update_telemetry_from_parts(self, parts: List[str]):
        def getf(name: str):
            try:
                idx = self.telemetry_cols.get(name)
                if idx is None:
                    return None
                raw = parts[idx].replace('degC', '').strip()
                return float(raw)
            except Exception:
                return None
        def geti(name: str):
            try:
                idx = self.telemetry_cols.get(name)
                if idx is None:
                    return None
                return int(parts[idx])
            except Exception:
                return None
        def gets(name: str):
            try:
                idx = self.telemetry_cols.get(name)
                if idx is None:
                    return None
                return parts[idx]
            except Exception:
                return None

        t0 = getf('Temp0'); t1 = getf('Temp1'); t2 = getf('Temp2'); t3 = getf('Temp3')
        setp = getf('Set'); actual = getf('Actual'); cj = getf('ColdJ')
        heat = geti('Heat'); fan = geti('Fan'); mode = gets('Mode')

        def fmtf(x): return '—' if x is None else f"{x:.1f}"
        def fmti(x): return '—' if x is None else str(x)
        def fmts(x): return '—' if not x else x

        self.t0.setText(fmtf(t0)); self.t1.setText(fmtf(t1)); self.t2.setText(fmtf(t2)); self.t3.setText(fmtf(t3))
        self.tSet.setText(fmtf(setp)); self.tActual.setText(fmtf(actual)); self.tCJ.setText(fmtf(cj))
        self.tHeat.setText(fmti(heat)); self.tFan.setText(fmti(fan)); self.tMode.setText(fmts(mode))

        # Plot series (use wall time spacing)
        now = datetime.now().timestamp()
        if getattr(self, '_t0_epoch', None) is None:
            self._t0_epoch = now
        tsec = now - float(self._t0_epoch)
        if actual is not None:
            self.t0s.append(tsec); self.actuals.append(actual)
        if setp is not None:
            self.setpoints.append(setp)
        else:
            self.setpoints.append(None)
        self.t0_list.append(t0 if t0 is not None else None)
        self.t1_list.append(t1 if t1 is not None else None)
        self.t2_list.append(t2 if t2 is not None else None)
        self.t3_list.append(t3 if t3 is not None else None)

        self.heat_list.append(heat if heat is not None else None)
        self.fan_list.append(fan if fan is not None else None)

        # Keep a full record for CSV export (use ISO timestamp)
        iso = datetime.fromtimestamp(now).isoformat(timespec='milliseconds')
        self.teleRows.append((
            iso, float(tsec), setp, actual, cj, heat, fan, mode or "",
            t0, t1, t2, t3
        ))

        # Ignore controller 'Set' in the plot to reserve dashed line for Target copy-in
        # keep last N points
        N = 1200
        if len(self.t0s) > N:
            self.t0s = self.t0s[-N:]
            self.actuals = self.actuals[-N:]
            self.setpoints = self.setpoints[-N:]
            self.t0_list = self.t0_list[-N:]
            self.t1_list = self.t1_list[-N:]
            self.t2_list = self.t2_list[-N:]
            self.t3_list = self.t3_list[-N:]
            self.heat_list = self.heat_list[-N:]
            self.fan_list  = self.fan_list[-N:]
        if len(self.teleRows) > N:
            self.teleRows = self.teleRows[-N:]
        self.update_telemetry_curves()
        self.configure_plot_axes(self.telePlot, disable_mouse=False)


    def update_telemetry_curves(self):
        """Push current buffers to curves, respecting None gaps for sensors."""
        # Actual
        self.curveActual.setData(self.t0s, [v for v in self.actuals])
        # Setpoint
        if any(v is not None for v in self.setpoints):
            self.curveSet.setData(self.t0s, [v if v is not None else float('nan') for v in self.setpoints])
        else:
            self.curveSet.setData([], [])
        # Individual sensors
        def _nanify(lst):
            return [v if v is not None else float('nan') for v in lst]
        if self.t0_list:
            self.curveT0.setData(self.t0s, _nanify(self.t0_list))
            self.curveT1.setData(self.t0s, _nanify(self.t1_list))
            self.curveT2.setData(self.t0s, _nanify(self.t2_list))
            self.curveT3.setData(self.t0s, _nanify(self.t3_list))
        else:
            for c in (self.curveT0, self.curveT1, self.curveT2, self.curveT3):
                c.setData([], [])
        # Heater/Fan on secondary axis (use NaN for gaps)
        def _nanify_int(lst):
            return [float(v) if v is not None else float('nan') for v in lst]
        if hasattr(self, 'curveHeat'):
            self.curveHeat.setData(self.t0s, _nanify_int(self.heat_list) if self.heat_list else [])
        if hasattr(self, 'curveFan'):
            self.curveFan.setData(self.t0s, _nanify_int(self.fan_list) if self.fan_list else [])
        # Apply current visibility settings
        self.update_series_visibility()

    def update_series_visibility(self):
        """Show/hide curves based on checkbox state."""
        self.curveActual.setVisible(self.chkShowActual.isChecked())
        # Target is two curves
        show_target = self.chkShowTarget.isChecked()
        self.curveTargetLine.setVisible(show_target)
        self.curveTargetPts.setVisible(show_target)
        self.curveSet.setVisible(self.chkShowSet.isChecked())
        self.curveT0.setVisible(self.chkShowT0.isChecked())
        self.curveT1.setVisible(self.chkShowT1.isChecked())
        self.curveT2.setVisible(self.chkShowT2.isChecked())
        self.curveT3.setVisible(self.chkShowT3.isChecked())
        if hasattr(self, 'curveHeat'):
            self.curveHeat.setVisible(self.chkShowHeat.isChecked())
        if hasattr(self, 'curveFan'):
            self.curveFan.setVisible(self.chkShowFan.isChecked())


    def on_save_telemetry_csv(self):
        """Export collected live telemetry to CSV. If empty, informs the user."""
        if not hasattr(self, 'teleRows') or not self.teleRows:
            QtWidgets.QMessageBox.information(self, "No Telemetry", "No telemetry data to save yet.")
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save Telemetry CSV", "telemetry.csv", "CSV Files (*.csv)"
        )
        if not path:
            return
        import csv
        try:
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["timestamp_iso", "t_sec", "set_c", "actual_c", "coldj_c", "heat", "fan", "mode", "temp0_c", "temp1_c", "temp2_c", "temp3_c"]) 
                for row in self.teleRows:
                    w.writerow(row)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to save telemetry: {e}")
            return
        self.statusBar().showMessage(f"Saved telemetry: {path}", 4000)


def main():
    app = QtWidgets.QApplication(sys.argv)
    # nicer default font for table
    app.setFont(QtGui.QFont("Helvetica", 11))
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
