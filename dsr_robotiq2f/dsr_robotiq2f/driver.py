"""Robotiq 2F-85 Modbus RTU driver (no ROS dependency, unit-testable).

Registers (Robotiq manual): output FC16 @ 1000 (3 words), input FC04 @ 2000 (3 words).
Position byte: 0 = fully open, 255 = fully closed.
The gripper watchdog raises fault 9 if it hears nothing for ~1 s, so the caller
must poll status continuously (the ROS node does this at 50 Hz).
"""
from dataclasses import dataclass
import threading

OUT_REG = 1000
IN_REG = 2000

FAULTS = {
    0: 'no fault', 5: 'action delayed, activation must be completed first',
    7: 'activation bit must be set prior to action', 8: 'max temperature exceeded',
    9: 'no communication for at least 1 s (watchdog)',
    10: 'under minimum operating voltage', 11: 'automatic release in progress',
    12: 'internal fault', 13: 'activation fault', 14: 'overcurrent',
    15: 'automatic release completed',
}
OBJ = {0: 'moving', 1: 'stopped, object detected while opening',
       2: 'stopped, object detected while closing', 3: 'at requested position'}


@dataclass
class Status:
    act: int
    gto: int
    sta: int      # 3 = activation complete
    obj: int
    fault: int
    pos_req: int
    pos: int
    current: int  # raw, ~10 mA units

    @property
    def activated(self):
        return self.sta == 3

    @property
    def object_detected(self):
        return self.obj in (1, 2)

    @property
    def moving(self):
        return self.obj == 0 and self.gto == 1


def decode_status(regs):
    b = b''.join(int(r).to_bytes(2, 'big') for r in regs)
    return Status(act=b[0] & 1, gto=(b[0] >> 3) & 1, sta=(b[0] >> 4) & 3,
                  obj=(b[0] >> 6) & 3, fault=b[2], pos_req=b[3], pos=b[4], current=b[5])


def encode_command(pos, speed, force, act=1, gto=1):
    clamp = lambda v: max(0, min(255, int(v)))
    return [((act & 1) | ((gto & 1) << 3)) << 8, clamp(pos), (clamp(speed) << 8) | clamp(force)]


class Robotiq2F:
    def __init__(self, port, slave=9, baudrate=115200, timeout=0.2, client=None):
        self.slave = slave
        self._lock = threading.Lock()
        if client is None:
            from pymodbus.client import ModbusSerialClient
            client = ModbusSerialClient(port=port, baudrate=baudrate, bytesize=8,
                                        parity='N', stopbits=1, timeout=timeout)
        self._c = client

    def connect(self):
        return bool(self._c.connect())

    def close(self):
        self._c.close()

    def status(self):
        with self._lock:
            r = self._c.read_input_registers(IN_REG, 3, slave=self.slave)
        if r.isError():
            raise IOError(f'Modbus read failed: {r}')
        return decode_status(r.registers)

    def _write(self, words):
        with self._lock:
            r = self._c.write_registers(OUT_REG, words, slave=self.slave)
        if r.isError():
            raise IOError(f'Modbus write failed: {r}')

    def reset(self):
        self._write(encode_command(0, 0, 0, act=0, gto=0))

    def request_activate(self):
        self._write(encode_command(0, 0, 0, act=1, gto=0))

    def move(self, pos, speed, force):
        self._write(encode_command(pos, speed, force))

    def stop(self):
        """Halt motion: clear rGTO while keeping the gripper activated."""
        self._write(encode_command(0, 0, 0, act=1, gto=0))
