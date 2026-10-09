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


def test_raw_mechanical_limit_cannot_be_hidden_by_median(tmp_path):
    plant, reader, controller = setup(tmp_path)
    reader._safe_high = 80
    plant.position = 79
    reader.fresh()
    plant.position = 81
    with pytest.raises(MotionSafetyError, match='mechanical range'):
        controller._safe_position()
    assert controller._stop.is_set()


@pytest.mark.parametrize('field', ['mean_voltage_v', 'volume_ml'])
@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_mapping_rejects_nonfinite_calibration(field, value):
    points = [{'mean_voltage_v':v*.03, 'volume_ml':v} for v in (0,50,100)]
    points[-1][field] = value
    with pytest.raises(ValueError):
        SoftPotMapping(points)


def test_watcher_cancels_before_disable_and_blocks_restart_of_lingering_motion(tmp_path):
    import threading
    import time
    from dataclasses import replace
    plant, reader, controller = setup(tmp_path)
    entered, release = threading.Event(), threading.Event()
    def hang(*args):
        entered.set()
        assert release.wait(5)
    plant.move = hang
    cancelled_at_disable = []
    plant.disable = lambda: cancelled_at_disable.append(controller._stop.is_set())
    plan = replace(controller.create_trial_plan(.1), expected_duration_s=-2)
    try:
        with pytest.raises(MotionSafetyError, match='timed out'):
            controller._watch_measurement(plan, time.monotonic())
        assert entered.is_set()
        assert cancelled_at_disable and all(cancelled_at_disable)
        with pytest.raises(RuntimeError, match='already running'):
            controller.start('AIR', [.1], 1)
        assert controller._stop.is_set()
    finally:
        release.set(); controller._motion.join(1)


def test_simultaneous_starts_claim_only_one_worker(tmp_path):
    import threading
    plant, reader, controller = setup(tmp_path)
    release, entered = threading.Event(), threading.Event()
    def run(*args):
        entered.set(); assert release.wait(2)
    controller._run = run
    barrier = threading.Barrier(3)
    results = []
    def start():
        barrier.wait()
        try:
            controller.start('AIR',[.1],1); results.append('started')
        except RuntimeError:
            results.append('conflict')
    first = threading.Thread(target=start); second = threading.Thread(target=start)
    first.start(); second.start(); barrier.wait()
    try:
        assert entered.wait(1)
        first.join(1); second.join(1)
        assert sorted(results) == ['conflict','started']
        with pytest.raises(RuntimeError, match='during calibration'):
            controller.install_position_reader(reader)
    finally:
        release.set(); controller._worker.join(1)


def test_stop_does_not_overwrite_worker_terminal_status(tmp_path):
    from types import SimpleNamespace
    from calibration.controller import State
    plant, reader, controller = setup(tmp_path)
    controller._worker = SimpleNamespace(is_alive=lambda:True)
    plant.disable = lambda: controller._set(state=State.ABORTED)
    controller.stop(emergency=True)
    assert controller.status()['state'] == 'ABORTED'


def test_start_clears_previous_run_status_before_worker_executes(tmp_path, monkeypatch):
    from calibration.controller import State
    plant, reader, controller = setup(tmp_path)
    controller._set(state=State.COMPLETE, run_id='previous', message='old')
    class PendingWorker:
        def __init__(self, **kwargs): pass
        def start(self): pass
        def is_alive(self): return True
    monkeypatch.setattr('calibration.controller.threading.Thread', PendingWorker)
    controller.start('AIR',[.1],1)
    status = controller.status()
    assert status['state'] == 'IDLE'
    assert status['run_id'] is None and status['message'] == ''


def test_failed_shutdown_is_displayed_as_failure(tmp_path):
    plant, reader, controller = setup(tmp_path)
    def fail(): raise RuntimeError('daemon lost')
    plant.disable = fail
    with pytest.raises(RuntimeError, match='daemon lost'):
        controller.stop(emergency=True)
    assert controller._stop.is_set()
    assert controller.status()['state'] == 'FAILED'
    assert 'shutdown failed' in controller.status()['message']


def test_measurement_commands_measured_start_to_planned_endpoint(tmp_path):
    import time
    plant, reader, controller = setup(tmp_path)
    plan = controller.create_trial_plan(1)
    plant.position = 79.55
    controller.move_to_volume = lambda *args:79.55
    controller.config['calibration']['settle_s'] = 0
    def watch(measured_plan, started):
        assert measured_plan.start_volume_ml == 79.55
        assert measured_plan.end_volume_ml == 20
        assert measured_plan.stroke_ml == pytest.approx(59.55)
        assert measured_plan.expected_duration_s == pytest.approx(3.573)
        plant.position = 20
        return [{}, {}, {}]
    controller._watch_measurement = watch
    controller.analyze_trial = lambda *args: {'accepted':True}
    summary, _ = controller.execute_trial(plan,1,1,time.monotonic())
    assert summary['accepted']


def test_run_creation_failure_reports_failed_and_disables_motor(tmp_path):
    plant, reader, controller = setup(tmp_path)
    disabled = []
    plant.disable = lambda:disabled.append(True)
    def fail(*args): raise OSError('data directory not writable')
    controller.store.create = fail
    controller._run('AIR',[.1],1)
    assert disabled
    assert controller.status()['state'] == 'FAILED'
    assert 'not writable' in controller.status()['message']


def test_final_shutdown_failure_cannot_leave_aborted_as_terminal_state(tmp_path):
    plant, reader, controller = setup(tmp_path)
    def abort(*args): raise InterruptedError('cancelled')
    def fail(): raise RuntimeError('daemon unavailable')
    controller.capture_zero = abort
    plant.disable = fail
    controller._run('AIR',[.1],1)
    status = controller.status()
    assert status['state'] == 'FAILED'
    assert 'shutdown failed' in status['message']
    assert controller.store.read_json(status['run_id'],'run.json')['completion_state'] == 'FAILED'


def test_stationary_drift_during_settling_prevents_measurement_start(tmp_path):
    import time
    plant, reader, controller = setup(tmp_path)
    plan = controller.create_trial_plan(.1)
    controller._stop.wait = lambda seconds:setattr(plant,'position',82) or False
    controller._watch_measurement = lambda *args:pytest.fail('must not start measurement after drift')
    with pytest.raises(MotionSafetyError, match='changed during settling'):
        controller.execute_trial(plan,.1,1,time.monotonic())
