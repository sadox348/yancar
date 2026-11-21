"""Thread-safe pyserial transport used by the ROS 2 node."""

import threading
from typing import Optional

import serial


class SerialTransport:
    def __init__(self, port: str, baud: int, timeout: float = 0.05):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._serial: Optional[serial.Serial] = None
        self._lock = threading.Lock()

    @property
    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def open(self) -> None:
        with self._lock:
            if self.is_open:
                return
            self._serial = serial.Serial(
                self.port, self.baud, timeout=self.timeout)

    def close(self) -> None:
        with self._lock:
            if self._serial is not None:
                self._serial.close()
                self._serial = None

    def write(self, frame: str) -> None:
        with self._lock:
            if not self.is_open:
                raise serial.SerialException("serial port is not open")
            self._serial.write(frame.encode("ascii"))
            self._serial.flush()

    def read_line(self) -> str:
        serial_port = self._serial
        if serial_port is None or not serial_port.is_open:
            raise serial.SerialException("serial port is not open")
        return serial_port.readline().decode("ascii", errors="replace").strip()

