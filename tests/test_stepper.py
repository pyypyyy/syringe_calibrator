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
    def wave_delete(self, wid): self.deleted.append(wid); return 0
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


@pytest.mark.parametrize('operation', ['wave_tx_stop', 'write'])
def test_shutdown_checks_negative_results_but_attempts_all_outputs(motor, operation):
    stepper, pi = motor
    original = getattr(pi, operation)
    def fail(*args):
        original(*args)
        return -5
    setattr(pi, operation, fail)
    with pytest.raises(RuntimeError, match='pigpio'):
        stepper.disable()
    assert pi.writes[-2:] == [(18, 0), (21, 1)]


def test_busy_error_is_not_success_or_an_infinite_wait(motor):
    stepper, pi = motor
    results = iter([-5, 0])
    pi.wave_tx_busy = lambda: next(results)
    with pytest.raises(RuntimeError, match='pigpio'):
        stepper.move(104, 347, threading.Event())
    assert pi.deleted == [1]
    assert pi.writes[-1] == (21, 1)


def test_wave_delete_error_propagates(motor):
    stepper, pi = motor
    pi.wave_delete = lambda wid: -5
    with pytest.raises(RuntimeError, match='delete wave'):
        stepper.move(104, 347, threading.Event())


def test_wave_timeout_stops_and_disables(motor, monkeypatch):
    stepper, pi = motor
    clock = iter([0, 0, 100])
    monkeypatch.setattr('hardware.stepper.time.monotonic', lambda: next(clock))
    busy = iter([1, 1, 0])
    pi.wave_tx_busy = lambda: next(busy)
    with pytest.raises(RuntimeError, match='timed out'):
        stepper.move(104, 347, threading.Event())
    assert pi.deleted == [1]
    assert pi.writes[-1] == (21, 1)


@pytest.mark.parametrize('steps', [1, -256, 12480, 65535])
def test_chain_count_and_configured_pulse_timing(motor, steps):
    stepper, pi = motor
    captured = []
    pi.wave_add_generic = lambda pulses: captured.extend(pulses) or 2
    stepper.move(steps, 208 * 1000 / 60, threading.Event())
    chain = pi.chains[-1]
    assert chain[-2] + 256 * chain[-1] == abs(steps)
    assert captured == [(1 << 18, 0, 144), (0, 1 << 18, 144)]


@pytest.mark.parametrize('operation', ['write', 'wave_clear', 'wave_add_generic', 'wave_create', 'wave_chain'])
def test_shutdown_serializes_with_every_startup_stage(motor, operation):
    stepper, pi = motor
    entered, release, cancelled = threading.Event(), threading.Event(), threading.Event()
    stop = threading.Event()
    original = getattr(pi, operation)
    def block(*args):
        entered.set()
        assert release.wait(2)
        return original(*args)
    setattr(pi, operation, block)
    errors = []
    def move():
        try: stepper.move(104, 347, stop)
        except Exception as exc: errors.append(exc)
    def shutdown():
        stop.set(); cancelled.set(); stepper.disable()
    motion = threading.Thread(target=move); motion.start()
    assert entered.wait(1)
    shutdown_thread = threading.Thread(target=shutdown); shutdown_thread.start()
    assert cancelled.wait(1)
    release.set()
    motion.join(1); shutdown_thread.join(1)
    assert not motion.is_alive() and not shutdown_thread.is_alive()
    assert not errors
    assert pi.writes[-1] == (21, 1)
    assert len(pi.chains) == (1 if operation == 'wave_chain' else 0)
    # A cancelled queued move cannot send a second chain after shutdown.
    chains = list(pi.chains)
    stepper.move(104, 347, stop)
    assert pi.chains == chains


@pytest.mark.parametrize('phase', ['positioning', 'measurement'])
@pytest.mark.parametrize('emergency', [False, True])
def test_controller_stop_cancels_production_stepper_before_wave_start(motor, tmp_path, phase, emergency):
    import time
    from pathlib import Path
    import yaml
    from calibration.controller import CalibrationController
    from storage.runs import RunStore
    stepper, pi = motor
    cfg = yaml.safe_load((Path(__file__).parents[1]/'config.yaml').read_text())
    cfg['calibration']['settle_s'] = 0
    controller = CalibrationController(SimpleNamespace(voltage=lambda:1), lambda:(1.5,50), stepper, RunStore(tmp_path), cfg)
    plan = controller.create_trial_plan(.1)
    if phase == 'measurement':
        controller.position_reader = lambda:(2.4,plan.start_volume_ml)
    entered, release = threading.Event(), threading.Event()
    def block():
        entered.set(); assert release.wait(2)
    pi.before_create = block
    errors = []
    def execute():
        try:
            if phase == 'positioning': controller.move_to_volume(51)
            else: controller.execute_trial(plan,.1,1,time.monotonic())
        except Exception as exc: errors.append(exc)
    worker = threading.Thread(target=execute); worker.start()
    assert entered.wait(1)
    stopper = threading.Thread(target=controller.stop, kwargs={'emergency':emergency})
    stopper.start()
    assert controller._stop.wait(1)
    release.set(); stopper.join(1); worker.join(1)
    assert not stopper.is_alive() and not worker.is_alive()
    assert len(errors) == 1 and isinstance(errors[0], InterruptedError)
    assert not pi.chains and pi.deleted == [1]
    assert pi.writes[-1] == (21,1)


@pytest.mark.parametrize('operation', ['wave_clear', 'wave_add_generic', 'wave_create', 'wave_chain'])
def test_negative_startup_results_disable_motor(motor, operation):
    stepper, pi = motor
    setattr(pi, operation, lambda *args:-5)
    with pytest.raises(RuntimeError, match='pigpio'):
        stepper.move(104,347,threading.Event())
    assert pi.writes[-1] == (21,1)
    assert pi.deleted == ([1] if operation == 'wave_chain' else [])


def test_drive_strength_verification_failure_starts_disabled(motor):
    stepper, pi = motor
    pi.writes.clear()
    pi.get_pad_strength = lambda pad:8
    with pytest.raises(RuntimeError, match='drive-strength verification'):
        Stepper(stepper.cfg)
    assert (21,1) in pi.writes and (21,0) not in pi.writes
