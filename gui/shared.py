# shared.py
from __future__ import annotations
from typing import Optional, List, Deque, Callable
from collections import deque

from PyQt6 import QtCore
import serial
import serial.tools.list_ports as list_ports

# ---------- Small ASCII protocol ----------
class AsciiProtocol:
    def __init__(self, eol: bytes = b"\n"):
        self.eol = eol
        self.buf = bytearray()

    def encode(self, line: str) -> bytes:
        return line.encode("utf-8") + self.eol

    def feed(self, chunk: bytes) -> List[str]:
        out: List[str] = []
        self.buf.extend(chunk)
        while True:
            i = self.buf.find(self.eol)
            if i < 0:
                break
            line = self.buf[:i]
            del self.buf[: i + len(self.eol)]
            out.append(line.decode(errors="replace"))
        return out


# ---------- Serial worker thread ----------
class SerialWorker(QtCore.QThread):
    rx_text = QtCore.pyqtSignal(str)
    rx_raw = QtCore.pyqtSignal(bytes)
    status = QtCore.pyqtSignal(str)
    error = QtCore.pyqtSignal(str)
    connected_changed = QtCore.pyqtSignal(bool)

    def __init__(self, port: str, baud: int, proto: AsciiProtocol, parent=None):
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self.proto = proto
        self._ser: Optional[serial.Serial] = None
        self._run = False
        self._lock = QtCore.QMutex()
        self._tx: Deque[bytes] = deque()

    @QtCore.pyqtSlot(bytes)
    def send(self, data: bytes):
        with QtCore.QMutexLocker(self._lock):
            self._tx.append(data)

    def run(self):
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.05, write_timeout=0.5)
        except Exception as e:
            self.error.emit(f"Open failed: {e}")
            self.connected_changed.emit(False)
            return
        self.connected_changed.emit(True)
        self.status.emit(f"Opened {self.port} @ {self.baud}")
        self._run = True
        try:
            while self._run:
                # write
                pkt = None
                with QtCore.QMutexLocker(self._lock):
                    if self._tx:
                        pkt = self._tx.popleft()
                if pkt:
                    try: self._ser.write(pkt)
                    except Exception as e: self.error.emit(f"Write error: {e}")

                # read
                try:
                    chunk = self._ser.read(4096)
                except Exception as e:
                    self.error.emit(f"Read error: {e}")
                    break
                if chunk:
                    self.rx_raw.emit(chunk)
                    for line in self.proto.feed(chunk):
                        self.rx_text.emit(line)
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
        self._run = False


def list_serial_ports() -> List[str]:
    return [p.device for p in list_ports.comports()]