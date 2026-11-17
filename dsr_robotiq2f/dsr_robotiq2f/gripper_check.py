"""Standalone bench check (no ROS graph needed): python -m / ros2 run dsr_robotiq2f gripper_check.

Activates the gripper (moves the fingers through full range once) and does a gentle
open / half / close / open cycle while polling continuously (keeps the watchdog alive).
"""
import argparse
import time

from dsr_robotiq2f.driver import FAULTS, Robotiq2F

DEFAULT_PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C90002140-if00'


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default=DEFAULT_PORT)
    ap.add_argument('--slave', type=int, default=9)
    ap.add_argument('--speed', type=int, default=100)
    ap.add_argument('--force', type=int, default=60)
    ap.add_argument('--status-only', action='store_true', help='read status, send nothing')
    a = ap.parse_args(argv)
    g = Robotiq2F(a.port, a.slave)
    if not g.connect():
        raise SystemExit(f'cannot open {a.port}')

    def wait(t):
        t0 = time.time()
        while time.time() - t0 < t:
            time.sleep(0.05)
            s = g.status()
        return s

    s = g.status()
    print('status:', s, '|', FAULTS.get(s.fault, s.fault))
    if a.status_only:
        return
    g.reset(); wait(0.5)
    g.request_activate()
    t0 = time.time()
    while time.time() - t0 < 15 and not g.status().activated:
        time.sleep(0.2)
    s = g.status()
    assert s.activated and s.fault == 0, f'activation failed: {s}'
    print('activated:', s)
    for name, pos in (('open', 0), ('half', 128), ('close', 255), ('open', 0)):
        g.move(pos, a.speed, a.force)
        s = wait(3.0)
        print(f'{name:5s} req={pos:3d} -> pos={s.pos:3d} obj={s.obj} fault={s.fault} cur={s.current}')
        assert s.fault == 0, FAULTS.get(s.fault)
    print('PASS')


if __name__ == '__main__':
    main()
