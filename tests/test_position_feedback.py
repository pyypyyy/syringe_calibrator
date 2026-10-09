import pytest
import yaml
from pathlib import Path

from calibration.controller import CalibrationController, MotionSafetyError
from hardware.softpot import PositionReader, SoftPotMapping
from storage.runs import RunStore


class Plant:
    position = 50.0
    def voltage(self, channel): return self.position * .03
    def disable(self): pass
    def move(self, steps, hz, stop): self.position += steps / 208


def setup(tmp_path):
    cfg = yaml.safe_load((Path(__file__).parents[1] / 'config.yaml').read_text())
    plant = Plant()
    mapping = SoftPotMapping([{'mean_voltage_v':v*.03, 'volume_ml':v} for v in (0,50,100)])
    reader = PositionReader(plant, 2, mapping, cfg['safety'])
    controller = CalibrationController(None, reader, plant, RunStore(tmp_path), cfg)
    return plant, reader, controller


def test_positioning_uses_fresh_feedback_in_both_directions(tmp_path):
    plant, reader, controller = setup(tmp_path)
    for _ in range(5): reader()
    for target in (57.5, 25, 80, 50):
        reported = controller.move_to_volume(target)
        assert reported == pytest.approx(plant.position)
        assert abs(plant.position-target) <= .5


def test_fast_stroke_endpoint_refresh_removes_filter_lag(tmp_path):
    plant, reader, controller = setup(tmp_path)
    for i in range(73):
        plant.position = 80-i*(1000/60)*.05
        reader()
    assert reader()[1]-plant.position > .5
    assert controller._safe_position(fresh=True)[1] == pytest.approx(20)


def test_fresh_samples_still_reject_out_of_range(tmp_path):
    plant, reader, controller = setup(tmp_path)
    plant.position = 110
    with pytest.raises(MotionSafetyError, match='Invalid SoftPot'):
        controller._safe_position(fresh=True)


def test_stop_flag_precedes_hardware_disable(tmp_path):
    plant, reader, controller = setup(tmp_path)
    observed = []
    plant.disable = lambda: observed.append(controller._stop.is_set())
    controller.stop(emergency=True)
    assert observed == [True]


def test_motion_thread_failure_reaches_controller(tmp_path):
    plant, reader, controller = setup(tmp_path)
    def fail(*args): raise RuntimeError('wave unavailable')
    plant.move = fail
    plan = controller.create_trial_plan(.1)
    import time
    with pytest.raises(MotionSafetyError, match='wave unavailable'):
        controller._watch_measurement(plan, time.monotonic())
