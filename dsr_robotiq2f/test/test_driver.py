from dsr_robotiq2f.driver import Robotiq2F, decode_status, encode_command


class R:
    def __init__(self, regs=None):
        self.registers = regs or []
    def isError(self):
        return False


class FakeClient:
    def __init__(self):
        self.written = []
    def connect(self):
        return True
    def close(self):
        pass
    def read_input_registers(self, addr, count, slave):
        assert (addr, count, slave) == (2000, 3, 9)
        return R([0x3100, 0x0000, 0x7A1F])
    def write_registers(self, addr, words, slave):
        self.written.append((addr, words, slave))
        return R()


def test_decode():
    s = decode_status([0x3100, 0x0900, 0x03E4])  # act, sta=3, fault 9, req 0, pos 3, cur 228
    assert s.act == 1 and s.activated and s.fault == 9 and s.pos == 3 and s.current == 228


def test_decode_object():
    s = decode_status([0xB900, 0x0000, 0x8010])
    assert s.obj == 2 and s.object_detected and s.gto == 1


def test_encode():
    assert encode_command(255, 100, 60) == [0x0900, 255, (100 << 8) | 60]
    assert encode_command(300, -5, 999) == [0x0900, 255, (0 << 8) | 255]
    assert encode_command(0, 0, 0, act=0, gto=0) == [0, 0, 0]
    assert encode_command(0, 0, 0, act=1, gto=0)[0] == 0x0100


def test_driver_io():
    c = FakeClient()
    g = Robotiq2F('x', 9, client=c)
    assert g.status().pos == 0x7A
    g.move(10, 20, 30)
    g.stop()
    assert c.written[0] == (1000, [0x0900, 10, (20 << 8) | 30], 9)
    assert c.written[1][1][0] == 0x0100
