import sys
import threading
from types import SimpleNamespace

import pytest

from hardware.stepper import Stepper


class Pi:
    connected = True
    def __init__(self):
        self.strength = 8
        self.chains = []
        self.writes = []
        self.deleted = []
        self.before_create = lambda: None
        self.failure = False
    def set_pad_strength(self, pad, strength):
        assert pad == 0
        self.strength = strength
        return 0
    def get_pad_strength(self, pad): return self.strength
    def set_mode(self, *args): return 0
    def write(self, *args):
        self.writes.append(args)
        return 0
    def wave_tx_stop(self): return 0
    def wave_clear(self): return 0
    def wave_add_generic(self, pulses): return 2
    def wave_create(self):
        self.before_create()
        return 1
    def wave_chain(self, chain):
        if self.failure: raise RuntimeError('daemon failure')
        self.chains.append(chain)
        return 0
    def wave_tx_busy(self): return False
    def wave_delete(self, wid): self.deleted.append(wid)
    def stop(self): pass


@pytest.fixture
def motor(monkeypatch):
    pi = Pi()
    monkeypatch.setitem(sys.modules, 'pigpio', SimpleNamespace(
        pi=lambda: pi, OUTPUT=1, pulse=lambda *args: args))
    cfg = dict(step_pin=18, dir_pin=4, enable_pin=21,
               enable_active_low=True, drive_strength_ma=12)
    return Stepper(cfg), pi


def test_configures_drive_strength_and_disabled_output(motor):
    _, pi = motor
    assert pi.strength == 12
    assert pi.writes[-1] == (21, 1)


def test_preset_stop_never_starts_wave(motor):
    stepper, pi = motor
    stop = threading.Event(); stop.set()
    stepper.move(104, 347, stop)
    assert not pi.chains


def test_stop_during_wave_build_never_starts_wave(motor):
    stepper, pi = motor
    stop = threading.Event()
    pi.before_create = stop.set
    stepper.move(104, 347, stop)
    assert not pi.chains
    assert pi.deleted == [1]
    assert pi.writes[-1] == (21, 1)


def test_stop_while_waiting_for_start_lock(motor):
    stepper, pi = motor
    stop = threading.Event()
    with stepper._lock:
        thread = threading.Thread(target=stepper.move, args=(104, 347, stop))
        thread.start()
        stop.set()
    thread.join(1)
    assert not thread.is_alive()
    assert not pi.chains


def test_wave_failure_disables_and_cleans_up(motor):
    stepper, pi = motor
    pi.failure = True
    with pytest.raises(RuntimeError, match='daemon failure'):
        stepper.move(104, 347, threading.Event())
    assert pi.writes[-1] == (21, 1)
    assert pi.deleted == [1]


def test_finite_move(motor):
    stepper, pi = motor
    stepper.move(-104, 347, threading.Event())
    assert pi.chains == [[255, 0, 1, 255, 1, 104, 0]]
    assert (4, 0) in pi.writes
    assert pi.writes[-1] == (21, 1)
