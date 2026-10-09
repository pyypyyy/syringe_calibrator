"""Production reader/controller with a deterministic test-only syringe/ADC clock.

No time/quality/stroke settings are accelerated or relaxed. The scheduled motion
thread replaces the physical actuator only; conversion and polling time advance
in instrument seconds. Real threading and pigpio startup races are tested elsewhere.
"""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from calibration.controller import CalibrationController, MotionSafetyError, State
from hardware.softpot import PositionReader, SoftPotMapping
from storage.runs import RunStore


class Clock:
    now = 0.0
    def advance(self, seconds): self.now += seconds
    def monotonic(self): return self.now


class Stop:
    def __init__(self, clock): self.clock = clock; self.cancelled = False
    def is_set(self): return self.cancelled
    def set(self): self.cancelled = True
    def clear(self): self.cancelled = False
    def wait(self, seconds):
        self.clock.advance(seconds)
        return self.cancelled


class Syringe:
    def __init__(self, clock, noise_v, conversion_s, fault):
        self.clock, self.noise_v, self.conversion_s, self.fault = clock, noise_v, conversion_s, fault
        self.rng = np.random.default_rng(120)
        self.origin = 50.0
        self.started = self.until = 0.0
        self.velocity = 0.0
        self.async_move = False
        self.disabled = True
        self.positions = []
        self.moves = 0

    @property
    def position(self):
        elapsed = min(self.clock.now, self.until) - self.started
        return self.origin + max(0, elapsed) * self.velocity

    def disable(self):
        self.origin = self.position
        self.started = self.until = self.clock.now
        self.velocity = 0.0
        self.disabled = True

    def move(self, steps, hz, stop):
        if stop.is_set(): return
        self.origin = self.position
        self.started = self.clock.now
        period = round(1_000_000 / hz) / 1_000_000
        self.until = self.started + abs(steps) * period
        self.velocity = (1 if steps > 0 else -1) / (208 * period)
        if self.fault == 'wrong_direction': self.velocity *= -1
        if self.fault == 'stuck': self.velocity = 0
        self.disabled = False
        self.moves += 1
        if not self.async_move:
            self.clock.advance(self.until - self.started)
            self.disable()

    def voltage(self, channel):
        self.clock.advance(self.conversion_s)
        if channel == 1:
            flowing = self.clock.now < self.until
            return .2 + .8 * abs(self.velocity) * 60 / 1000 * flowing
        if self.fault == 'disconnected': raise OSError('I2C disconnected')
        if self.fault == 'invalid': return float('nan')
        if self.fault == 'out_of_range': return 3.3
        self.positions.append(self.position)
        return self.position * .03 + self.rng.normal(0, self.noise_v)


def make_instrument(tmp_path, monkeypatch, noise_v=.0015, conversion_s=.009, fault=None):
    cfg = yaml.safe_load((Path(__file__).parents[1]/'config.yaml').read_text())
    clock = Clock()
    syringe = Syringe(clock, noise_v, conversion_s, fault)
    reader = PositionReader(syringe, 2, SoftPotMapping([
        {'mean_voltage_v':v*.03, 'volume_ml':v} for v in (0,50,100)
    ]), cfg['safety'])
    controller = CalibrationController(SimpleNamespace(voltage=lambda:syringe.voltage(1)),
        reader, syringe, RunStore(tmp_path), cfg)
    controller._stop = Stop(clock)
    class ScheduledMotion:
        def __init__(self, target, **kwargs): self.target = target
        def start(self):
            syringe.async_move = True
            try: self.target()
            finally: syringe.async_move = False
        def is_alive(self): return not syringe.disabled and clock.now < syringe.until
        def join(self, timeout): pass
    monkeypatch.setattr('calibration.controller.time', SimpleNamespace(monotonic=clock.monotonic))
    monkeypatch.setattr('calibration.controller.threading', SimpleNamespace(Thread=ScheduledMotion))
    return controller, syringe, clock


@pytest.mark.parametrize('target', [.02, .5, 1.0])
@pytest.mark.parametrize('conversion_s', [.009, .02])
def test_configured_measurement_and_return_with_production_filter(tmp_path, monkeypatch, target, conversion_s):
    controller, syringe, clock = make_instrument(tmp_path, monkeypatch, conversion_s=conversion_s)
    plan = controller.create_trial_plan(target)
    summary, samples = controller.execute_trial(plan, target, 1, clock.now)
    assert summary['accepted'], summary['rejection_reasons']
    assert summary['reference_flow_lpm'] == pytest.approx(target, rel=.015)
    assert abs(syringe.position - plan.end_volume_ml) <= .5
    assert any(sample['analysis_sample'] for sample in samples)
    reached = controller.move_to_volume(plan.start_volume_ml, State.RETURNING)
    assert abs(reached - plan.start_volume_ml) <= .5
    assert abs(syringe.position - plan.start_volume_ml) <= .5
    assert syringe.disabled and min(syringe.positions) > 0 and max(syringe.positions) < 100


@pytest.mark.parametrize('fault, expected', [
    ('disconnected','disconnected'), ('invalid','calibrated range'),
    ('out_of_range','calibrated range'), ('stuck','stall'),
    ('wrong_direction','opposite direction'),
])
def test_production_feedback_faults_disable_motion(tmp_path, monkeypatch, fault, expected):
    controller, syringe, clock = make_instrument(tmp_path, monkeypatch, noise_v=0, fault=fault)
    plan = controller.create_trial_plan(.1)
    with pytest.raises(MotionSafetyError, match=expected):
        controller.execute_trial(plan, .1, 1, clock.now)
    assert syringe.disabled


@pytest.mark.parametrize('noise_v', [.0015, .006])
def test_all_configured_targets_repeats_and_models_with_adc_delay_and_noise(tmp_path, monkeypatch, noise_v):
    controller, syringe, clock = make_instrument(tmp_path, monkeypatch, noise_v=noise_v)
    controller._run('AIR', controller.config['calibration']['target_flows_lpm'], 3)
    status = controller.status()
    assert status['state'] == 'COMPLETE', status
    analysis = controller.store.read_json(status['run_id'], 'analysis.json')
    assert analysis['accepted_trials'] == 21
    assert analysis['usable_calibration_points'] == 7
    selected = analysis['selected_model']
    assert selected['name'] in controller.config['analysis']['candidate_models']
    assert selected['monotonic']
    # 6 mV noise corresponds to 0.2 ml before filtering, not exact reference
    # flow. Bound prediction error to 0.5% of this instrument's full scale.
    assert selected['cv_rmse_lpm'] < .005
    assert np.polyval(selected['coefficients'], .2 + .8*.5) == pytest.approx(.5, abs=.01)
    assert analysis['bootstrap']['accepted_iterations'] > 0
    assert syringe.disabled
    assert min(syringe.positions) > 0 and max(syringe.positions) < 100


def test_slow_adc_cannot_make_short_high_flow_trial_pass_quality(tmp_path, monkeypatch):
    controller, syringe, clock = make_instrument(tmp_path, monkeypatch, conversion_s=.05)
    plan = controller.create_trial_plan(1.0)
    summary, samples = controller.execute_trial(plan, 1.0, 1, clock.now)
    assert not summary['accepted']
    assert 'sample count below minimum' in summary['rejection_reasons']
    assert syringe.disabled
