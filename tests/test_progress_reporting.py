"""Observability must use existing feedback and leave operational completion honest."""
from types import SimpleNamespace

import pytest

from calibration.controller import State
from test_controller_integration import make_controller, wait_terminal


@pytest.mark.parametrize('state', [State.POSITIONING, State.MEASURING, State.RETURNING])
@pytest.mark.parametrize('start,end', [(60., 40.), (40., 60.)])
def test_motion_progress_uses_feedback_and_suppresses_only_display_jitter(tmp_path, state, start, end):
    controller, _, _ = make_controller(tmp_path)
    controller._set(state=state, operation_start_position_ml=start,
                    operation_target_position_ml=end, position_ml=(start + end) / 2)
    assert controller.status()['operation_progress'] == .5
    controller._set(position_ml=start + (end - start) * .49)
    status = controller.status()
    assert status['operation_progress'] == .5
    assert status['position_ml'] == start + (end - start) * .49
    controller._set(position_ml=end + (end - start) * .1)
    assert controller.status()['operation_progress'] == 1
    controller._set(position_ml=None)
    assert controller.status()['operation_progress'] is None
    controller._set(state=State.SETTLING, operation_duration_s=5)
    assert controller.status()['operation_start_position_ml'] is None
    controller._set(state=State.POSITIONING, operation_start_position_ml=end,
                    operation_target_position_ml=start, position_ml=end)
    assert controller.status()['operation_progress'] == 0


def test_status_does_not_read_any_sensors_or_move_motor(tmp_path):
    controller, _, stepper = make_controller(tmp_path)
    def forbidden(*args):
        pytest.fail('status accessed hardware')
    controller.position_reader = forbidden
    controller.flow_sensor.voltage = forbidden
    stepper.move = stepper.disable = forbidden
    controller._set(state=State.MEASURING, operation_start_position_ml=60,
                    operation_target_position_ml=40)
    for _ in range(20):
        status = controller.status()
        assert status['operation_progress'] is None
        assert status['position_ml'] is None
        assert status['sensor_voltage_v'] is None
    controller._set(state=State.ANALYZING)
    assert controller.status()['operation_progress'] is None


@pytest.mark.parametrize('state', [State.SETTLING, State.ZERO_BEFORE, State.ZERO_AFTER])
def test_time_progress_and_zero_phase_reset(tmp_path, monkeypatch, state):
    controller, _, _ = make_controller(tmp_path)
    clock = SimpleNamespace(now=100.)
    monkeypatch.setattr('calibration.controller.time', SimpleNamespace(monotonic=lambda: clock.now))
    controller._set(state=state, operation_duration_s=4)
    clock.now += 1
    status = controller.status()
    assert status['operation_progress'] == .25
    assert status['operation_elapsed_s'] == 1
    assert status['operation_remaining_s'] == 3
    if state != State.SETTLING:
        controller._zero_phase('recording', 10)
        assert controller.status()['operation_progress'] == 0
        assert controller.status()['operation_phase'] == 'recording'
    controller._set(state=State.FAILED, message='read failed')
    frozen = controller.status()
    clock.now += 20
    assert controller.status()['operation_elapsed_s'] == frozen['operation_elapsed_s']
    assert controller.status()['operation_progress'] == frozen['operation_progress']
    assert frozen['failure_operation'] == state.value


@pytest.mark.parametrize('terminal', [State.FAILED, State.ABORTED])
def test_overall_milestones_remain_below_complete_and_freeze(tmp_path, terminal):
    controller, _, _ = make_controller(tmp_path)
    controller._set(total=2, state=State.ZERO_BEFORE)
    assert controller.status()['progress'] == 0
    controller._set(zero_before_completed=True, state=State.RETURNING, completed=1)
    assert controller.status()['progress'] == 2 / 5
    controller._set(completed=2, state=State.ZERO_AFTER)
    assert controller.status()['progress'] == 3 / 5
    controller._set(zero_after_completed=True, state=State.ANALYZING)
    assert controller.status()['progress'] == 4 / 5
    controller._set(state=terminal)
    assert controller.status()['progress'] == 4 / 5
    controller._set(state=State.COMPLETE)
    assert controller.status()['progress'] == 1
    controller._set(state=State.FAILED, message='shutdown failed')
    assert controller.status()['progress'] == 1


@pytest.mark.parametrize('return_failure', [False, True])
def test_real_workflow_counts_only_after_return_and_retains_last_trial(tmp_path, return_failure):
    controller, _, _ = make_controller(tmp_path)
    original_move = controller.move_to_volume
    snapshots = []
    def move(target, state=State.POSITIONING):
        if state == State.RETURNING:
            snapshots.append(controller.status())
            if return_failure:
                controller._set(state=state)
                raise RuntimeError('return failed')
        elif snapshots:
            # Previous summary remains visible until the next return succeeds.
            assert controller.status()['last_completed_trial']['repeat'] == len(snapshots)
        return original_move(target, state)
    controller.move_to_volume = move
    controller.start('AIR', [.1], 2)
    status = wait_terminal(controller)
    assert snapshots[0]['completed'] == 0
    assert snapshots[0]['last_completed_trial'] is None
    assert snapshots[0]['progress'] == 1 / 5
    if return_failure:
        assert status['state'] == 'FAILED'
        assert status['failure_operation'] == 'RETURNING'
        assert status['completed'] == 0 and status['last_completed_trial'] is None
    else:
        assert snapshots[1]['completed'] == 1
        last = status['last_completed_trial']
        assert status['completed'] == 2
        assert last['repeat'] == 2
        assert last['measured_stroke_ml'] > 0
        saved = controller.store.read_json(status['run_id'], 'analysis.json')['trials'][-1]
        for key in ('reference_flow_lpm', 'sensor_voltage_v', 'accepted', 'rejection_reasons'):
            assert last[key] == saved[key]
        # The intentionally single-target run cannot produce a model.
        assert status['state'] == 'FAILED' and status['failure_operation'] == 'ANALYZING'
        assert status['progress'] == 4 / 5


@pytest.mark.parametrize('emergency', [False, True])
def test_stop_preserves_operation_context_and_cancellation_order(tmp_path, emergency):
    controller, _, stepper = make_controller(tmp_path)
    controller._worker = SimpleNamespace(is_alive=lambda: True)
    controller._set(state=State.MEASURING, total=3, completed=1,
                    current_target_lpm=.1, repeat=2, position_ml=48)
    observed = []
    stepper.disable = lambda: observed.append((controller._stop.is_set(), controller._emergency.is_set()))
    before = controller.status()['progress']
    controller.stop(emergency=emergency)
    status = controller.status()
    assert observed == [(True, emergency)]
    assert status['state'] == 'STOPPING'
    assert status['failure_operation'] == 'MEASURING'
    controller._set(state=State.RETURNING)
    assert controller.status()['state'] == 'STOPPING'
    controller._set(state=State.ABORTED)
    status = controller.status()
    assert status['progress'] == before
    assert status['position_ml'] == 48 and status['repeat'] == 2


def test_real_zero_reports_settling_and_recording_without_extra_acquisitions(tmp_path):
    controller, _, _ = make_controller(tmp_path)
    phases, readings = [], []
    original_phase = controller._zero_phase
    def phase(name, duration):
        original_phase(name, duration)
        phases.append((controller.status()['operation_phase'], duration))
    controller._zero_phase = phase
    def voltage():
        readings.append(1)
        assert controller.status()['operation_phase'] == 'recording'
        return .2
    controller.flow_sensor.voltage = voltage
    result = controller.capture_zero(State.ZERO_BEFORE)
    assert [name for name, _ in phases] == ['settling', 'recording']
    assert result['sample_count'] == len(readings) > 0
    assert controller.status()['sensor_voltage_v'] == .2


def test_new_run_clears_previous_trial_operation_and_milestones(tmp_path, monkeypatch):
    controller, _, _ = make_controller(tmp_path)
    controller._set(state=State.COMPLETE, completed=2, total=2,
                    zero_before_completed=True, zero_after_completed=True,
                    last_completed_trial={'repeat': 2}, operation_start_position_ml=50,
                    operation_target_position_ml=40, failure_operation='RETURNING')
    class PendingWorker:
        def __init__(self, **kwargs): pass
        def start(self): pass
        def is_alive(self): return True
    monkeypatch.setattr('calibration.controller.threading.Thread', PendingWorker)
    controller.start('CO2', [.02, .05], 3)
    status = controller.status()
    assert status['progress'] == 0 and status['completed'] == 0
    assert status['remaining'] == 6 and status['repeats_total'] == 3
    assert status['target_count'] == 2
    for key in ('last_completed_trial', 'operation_start_position_ml',
                'operation_target_position_ml', 'failure_operation', 'operation_progress'):
        assert status[key] is None


def test_rejected_last_trial_summary_is_an_independent_snapshot(tmp_path):
    controller, _, _ = make_controller(tmp_path)
    original_analyze = controller.analyze_trial
    def reject(*args):
        summary = original_analyze(*args)
        summary.update(accepted=False, rejection_reasons=['test rejection'])
        return summary
    controller.analyze_trial = reject
    controller.start('AIR', [.1], 1)
    status = wait_terminal(controller)
    last = status['last_completed_trial']
    assert last['accepted'] is False
    assert last['rejection_reasons'] == ['test rejection']
    assert status['completed'] == 1 and status['rejected'] == 1
    last['rejection_reasons'].append('consumer mutation')
    assert controller.status()['last_completed_trial']['rejection_reasons'] == ['test rejection']
