"""Preload guarantees concern commanded directions and measured feedback, not step totals."""
from dataclasses import replace
from types import SimpleNamespace

import pytest

from calibration.controller import MotionSafetyError, PositioningBudget, State
from test_position_feedback import setup
from test_controller_integration import make_controller, wait_terminal


def instrument(tmp_path, initial=50):
    plant, reader, controller = setup(tmp_path)
    plant.position = initial
    controller.config['calibration']['settle_s'] = 0
    commands = []
    move = plant.move
    def record(steps, hz, stop):
        status = controller.status()
        commands.append({'steps': steps, 'before': plant.position,
                         'state': status['state'], 'phase': status['operation_phase'],
                         'attempt': status['positioning_attempt']})
        move(steps, hz, stop)
    plant.move = record
    return plant, reader, controller, commands


@pytest.mark.parametrize('initial', [20, 68.75, 71.75, 85])
def test_same_direction_approach_from_arbitrary_valid_position(tmp_path, initial):
    plant, reader, controller, commands = instrument(tmp_path, initial)
    plan = controller.create_trial_plan(.05)
    assert (plan.start_volume_ml, plan.end_volume_ml) == pytest.approx((68.75, 31.25))
    reached, diagnostic = controller.prepare_trial(plan, .05, 1)
    approach = [c for c in commands if c['phase'] == 'directional_approach']
    assert approach and all(c['steps'] < 0 for c in approach)
    assert commands[-1]['steps'] < 0
    assert approach[0]['before'] > plan.start_volume_ml
    assert abs(reached - plan.start_volume_ml) <= .5
    assert diagnostic['configured_preload_ml'] == 3
    assert diagnostic['preload_target_ml'] == 71.75
    assert abs(diagnostic['preload_reached_ml'] - 71.75) <= .5
    assert diagnostic['approach_attempts'] == 1
    assert diagnostic['settling_displacement_ml'] == pytest.approx(0)
    assert diagnostic['commanded_steps'] <= diagnostic['pulse_limit']
    assert controller.config['axis']['microsteps_per_ml'] == 208
    assert controller.status()['completed'] == 0 and controller.status()['progress'] == 0


@pytest.mark.parametrize('always_overshoot', [False, True])
def test_overshoot_restarts_at_preload_without_reverse_final_correction(tmp_path, always_overshoot):
    plant, reader, controller, commands = instrument(tmp_path)
    plan = controller.create_trial_plan(.05)
    run_id = controller.store.create('AIR', {})
    controller._set(run_id=run_id)
    move = plant.move
    overshoots = []
    def overshoot(steps, hz, stop):
        phase = controller.status()['operation_phase']
        before = plant.position
        move(steps, hz, stop)
        if (phase == 'directional_approach' and before - plan.start_volume_ml <= .6
                and (always_overshoot or not overshoots)):
            plant.position -= .8
            overshoots.append(1)
    plant.move = overshoot
    if always_overshoot:
        with pytest.raises(MotionSafetyError, match='failed after 2 attempts'):
            controller.prepare_trial(plan, .05, 1)
    else:
        reached, diagnostics = controller.prepare_trial(plan, .05, 1)
        assert abs(reached - plan.start_volume_ml) <= .5
        assert diagnostics['approach_attempts'] == 2
        assert diagnostics['warnings']
        assert commands[-1]['steps'] < 0
    assert all(c['steps'] < 0 for c in commands if c['phase'] == 'directional_approach')
    assert any(c['steps'] > 0 and c['attempt'] == 2 and c['phase'] == 'pre_positioning' for c in commands)
    saved = (controller.store.read_json(run_id, 'trials/flow_0.050_repeat_1_positioning.json')
             if always_overshoot else diagnostics)
    assert saved['approach_attempts'] == 2
    assert saved['status'] == ('FAILED' if always_overshoot else 'READY')
    assert saved['commanded_steps'] <= saved['pulse_limit']
    if always_overshoot:
        assert 'overshot' in saved['error']
        assert saved['last_measured_position_ml'] is not None


def test_in_tolerance_crossing_is_accepted_without_reverse(tmp_path):
    plant, reader, controller, commands = instrument(tmp_path)
    plan = controller.create_trial_plan(.05)
    move = plant.move
    crossed = []
    def overshoot(steps, hz, stop):
        before = plant.position
        phase = controller.status()['operation_phase']
        move(steps, hz, stop)
        if phase == 'directional_approach' and before - plan.start_volume_ml <= .6 and not crossed:
            plant.position -= .2
            crossed.append(1)
    plant.move = overshoot
    reached, diagnostic = controller.prepare_trial(plan, .05, 1)
    assert abs(reached - plan.start_volume_ml) <= .5
    assert diagnostic['approach_attempts'] == 1
    assert diagnostic['warnings']
    assert commands[-1]['steps'] < 0


@pytest.mark.parametrize('phase', ['pre_positioning', 'directional_approach'])
@pytest.mark.parametrize('emergency', [False, True])
def test_stop_during_each_positioning_phase_cancels_before_next_chunk(tmp_path, phase, emergency):
    plant, reader, controller, commands = instrument(tmp_path)
    controller._worker = SimpleNamespace(is_alive=lambda: True)
    plan = controller.create_trial_plan(.05)
    run_id = controller.store.create('AIR', {})
    controller._set(run_id=run_id, total=1)
    move = plant.move
    disable_observations = []
    plant.disable = lambda: disable_observations.append(controller._stop.is_set())
    def cancel(steps, hz, stop):
        move(steps, hz, stop)
        if controller.status()['operation_phase'] == phase:
            controller.stop(emergency=emergency)
    plant.move = cancel
    with pytest.raises(InterruptedError):
        controller.prepare_trial(plan, .05, 1)
    command_count = len(commands)
    with pytest.raises(InterruptedError):
        controller.move_to_volume(71.75)
    assert len(commands) == command_count
    status = controller.status()
    assert status['state'] == 'STOPPING' and status['operation_phase'] == phase
    assert status['completed'] == 0 and status['progress'] == 0
    assert controller._emergency.is_set() == emergency
    assert disable_observations[-1] is True
    saved = controller.store.read_json(run_id, 'trials/flow_0.050_repeat_1_positioning.json')
    assert saved['status'] == 'ABORTED' and saved['phase'] == phase


@pytest.mark.parametrize('fault', ['stationary', 'opposite', 'invalid', 'noisy_progress'])
def test_feedback_faults_have_finite_travel_and_disable_driver(tmp_path, fault):
    plant, reader, controller, commands = instrument(tmp_path)
    plan = controller.create_trial_plan(.05)
    disabled = []
    plant.disable = lambda: disabled.append(True)
    move = plant.move
    def bad_move(steps, hz, stop):
        before = plant.position
        move(steps, hz, stop)
        if fault == 'stationary': plant.position = before
        elif fault == 'opposite': plant.position = before - steps / 208
        elif fault == 'invalid': plant.position = 150
        elif fault == 'noisy_progress': plant.position = before + .01
    plant.move = bad_move
    with pytest.raises(MotionSafetyError):
        controller.prepare_trial(plan, .05, 1)
    assert disabled
    assert len(commands) < 200
    assert sum(abs(c['steps']) for c in commands) < 10000


@pytest.mark.parametrize('low,high', [(0, 72), (30, 100), (25, 75)])
def test_no_motion_when_preload_or_stroke_has_insufficient_clearance(tmp_path, low, high):
    plant, reader, controller, commands = instrument(tmp_path)
    reader.min_calibrated_volume_ml = low
    reader.max_calibrated_volume_ml = high
    # Explicitly use the normal full-range plan: validation must reject, not clamp it.
    plan = replace(controller.create_trial_plan(.02), start_volume_ml=68.75,
                   end_volume_ml=31.25, stroke_ml=37.5)
    if low == 25:
        # A longer stroke within the mapping leaves no room for its preload.
        plan = replace(plan, start_volume_ml=73, end_volume_ml=27, stroke_ml=46)
    with pytest.raises(MotionSafetyError, match='Insufficient room'):
        controller.prepare_trial(plan, .05, 1)
    assert commands == []


@pytest.mark.parametrize('key,value', [
    ('directional_preload_ml', 0), ('directional_preload_ml', float('nan')),
    ('directional_preload_ml', 1), ('preload_clearance_ml', 0),
    ('directional_preload_attempts', 0), ('directional_preload_attempts', 4),
    ('directional_preload_attempts', True), ('directional_preload_attempts', 1.5),
])
def test_invalid_preload_settings_rejected_before_motion(tmp_path, key, value):
    plant, reader, controller, commands = instrument(tmp_path)
    controller.config['calibration'][key] = value
    with pytest.raises(MotionSafetyError): controller.create_trial_plan(.05)
    assert commands == []


def test_legacy_configuration_uses_provisional_defaults(tmp_path):
    plant, reader, controller, commands = instrument(tmp_path)
    for name in ('directional_preload_ml', 'directional_preload_attempts', 'preload_clearance_ml'):
        controller.config['calibration'].pop(name)
    plan = controller.create_trial_plan(.05)
    _, diagnostics = controller.prepare_trial(plan, .05, 1)
    assert diagnostics['configured_preload_ml'] == 3
    assert diagnostics['preload_target_ml'] == 71.75


@pytest.mark.parametrize('gas', ['AIR', 'CO2'])
def test_repeats_and_flow_transitions_return_to_next_preload_and_keep_measurement_direction(tmp_path, gas):
    controller, plant, stepper = make_controller(tmp_path)
    commands, returns, measurements = [], [], []
    original_move = stepper.move
    def record(steps, hz, stop):
        status = controller.status()
        commands.append((status['state'], status['operation_phase'], steps))
        return original_move(steps, hz, stop)
    stepper.move = record
    original_position = controller.move_to_volume
    def position(target, state=State.POSITIONING, **kwargs):
        if state == State.RETURNING:
            returns.append((controller.status()['completed'], target))
        return original_position(target, state, **kwargs)
    controller.move_to_volume = position
    original_watch = controller._watch_measurement
    def watch(plan, started):
        assert commands[-1][1] == 'directional_approach'
        assert commands[-1][2] < 0
        assert controller.status()['completed'] == len(measurements)
        measurements.append(plan)
        return original_watch(plan, started)
    controller._watch_measurement = watch
    # Keep this test about physical sequence and collected samples; existing full
    # integration tests separately verify successful models at all default flows.
    controller.start(gas, [.1, .2], 2)
    status = wait_terminal(controller)
    assert status['completed'] == 4
    assert len(measurements) == len(returns) == 4
    plans = [controller.create_trial_plan(t) for t in [.1, .2]]
    expected_returns = [controller.preload_position(plans[0])] + [controller.preload_position(plans[1])] * 3
    assert [t for _, t in returns] == expected_returns
    assert [c for c, _ in returns] == [0, 1, 2, 3]
    assert all(steps < 0 for _, phase, steps in commands if phase == 'directional_approach')
    analysis = controller.store.read_json(status['run_id'], 'analysis.json')
    for trial in analysis['trials']:
        assert trial['positioning']['status'] == 'READY'
        assert trial['raw_sample_count'] > 3
        raw = (controller.store.path(status['run_id']) / 'trials' / (trial['trial_id'] + '.csv')).read_text()
        assert 'MEASURING' in raw and 'POSITIONING' not in raw
    assert stepper.disabled


def test_progress_resets_between_preposition_and_approach_without_completing_trial(tmp_path):
    plant, reader, controller, commands = instrument(tmp_path)
    controller._set(total=3, completed=1, zero_before_completed=True)
    snapshots = []
    move = plant.move
    def record(steps, hz, stop):
        move(steps, hz, stop)
        snapshots.append(controller.status())
    plant.move = record
    plan = controller.create_trial_plan(.05)
    controller.prepare_trial(plan, .05, 2)
    pre = [s for s in snapshots if s['operation_phase'] == 'pre_positioning']
    approach = [s for s in snapshots if s['operation_phase'] == 'directional_approach']
    assert pre and approach
    assert approach[0]['operation_progress'] == 0
    assert all(s['progress'] == 2 / 6 and s['completed'] == 1 for s in snapshots)
    assert approach[0]['preload_position_ml'] == 71.75
    assert approach[0]['measurement_start_position_ml'] == 68.75
    assert approach[0]['positioning_attempt'] == 1
    assert 'Directional approach' in approach[0]['operation_description']
    assert approach[0]['operation_direction'] == 'Decreasing volume'


def test_no_false_approach_when_fresh_feedback_jumps_directly_to_start(tmp_path):
    plant, reader, controller, commands = instrument(tmp_path, 71.75)
    plan = controller.create_trial_plan(.05)
    original = reader.fresh
    def jump():
        if controller.status()['operation_phase'] == 'directional_approach':
            plant.position = plan.start_volume_ml
        return original()
    reader.fresh = jump
    with pytest.raises(MotionSafetyError, match='without a movement command'):
        controller.prepare_trial(plan, .05, 1)
    assert not any(c['phase'] == 'directional_approach' for c in commands)


def test_total_budget_refuses_a_chunk_before_issuing_it(tmp_path):
    plant, reader, controller, commands = instrument(tmp_path)
    with pytest.raises(MotionSafetyError, match='budget exhausted'):
        controller.move_to_volume(70, budget=PositioningBudget(1))
    assert commands == []


def test_failed_planning_is_persisted_without_issuing_motor_commands(tmp_path):
    plant, reader, controller, commands = instrument(tmp_path)
    reader.min_calibrated_volume_ml = 25
    reader.max_calibrated_volume_ml = 75
    controller.flow_sensor = SimpleNamespace(voltage=lambda: .2)
    controller.config['calibration'].update(zero_duration_s=.01, sample_interval_s=.001)
    controller._run('AIR', [.1], 1)
    status = controller.status()
    assert status['state'] == 'FAILED'
    assert status['failure_operation'] == 'POSITIONING'
    assert status['operation_phase'] == 'planning'
    assert commands == [] and status['completed'] == 0
    saved = controller.store.read_json(status['run_id'], 'trials/flow_0.100_repeat_1_positioning.json')
    assert saved['phase'] == 'planning' and saved['commanded_steps'] == 0
    assert 'Insufficient room' in saved['error']


def test_settling_displacement_cannot_exceed_existing_tolerance_even_if_target_error_passes(tmp_path):
    plant, reader, controller, commands = instrument(tmp_path)
    plan = controller.create_trial_plan(.05)
    move = plant.move
    crossed = []
    def overshoot(steps, hz, stop):
        before = plant.position
        phase = controller.status()['operation_phase']
        move(steps, hz, stop)
        if phase == 'directional_approach' and before - plan.start_volume_ml <= .6 and not crossed:
            plant.position = plan.start_volume_ml - .4
            crossed.append(True)
    plant.move = overshoot
    controller._stop.wait = lambda seconds: setattr(plant, 'position', plan.start_volume_ml + .4) or False
    with pytest.raises(MotionSafetyError, match='changed during settling'):
        controller.prepare_trial(plan, .05, 1)
    assert abs(plant.position - plan.start_volume_ml) < .5
    assert commands[-1]['steps'] < 0


def test_preloading_does_not_hide_existing_measurement_endpoint_error(tmp_path):
    plant, reader, controller, commands = instrument(tmp_path)
    plan = controller.create_trial_plan(.05)
    def watch(measured_plan, started):
        assert commands[-1]['phase'] == 'directional_approach'
        assert commands[-1]['steps'] < 0
        plant.position = measured_plan.end_volume_ml - .8
        return [{}, {}, {}]
    controller._watch_measurement = watch
    controller.analyze_trial = lambda *args: pytest.fail('endpoint failure must precede fitting')
    with pytest.raises(MotionSafetyError, match='measurement end position was not reached'):
        controller.execute_trial(plan, .05, 1, 0)
    assert controller.config['calibration']['position_tolerance_ml'] == .5
    assert controller.config['axis']['microsteps_per_ml'] == 208


def test_successful_diagnostics_do_not_delay_between_settling_verification_and_stroke(tmp_path):
    plant, reader, controller, commands = instrument(tmp_path)
    run_id = controller.store.create('AIR', {})
    controller._set(run_id=run_id)
    plan = controller.create_trial_plan(.05)
    order = []
    write = controller.store.write_json
    def save(run, name, data):
        order.append('persist')
        return write(run, name, data)
    controller.store.write_json = save
    def watch(measured_plan, started):
        assert order == []
        order.append('measure')
        plant.position = measured_plan.end_volume_ml
        return [{}, {}, {}]
    controller._watch_measurement = watch
    controller.analyze_trial = lambda *args: {'accepted': True}
    controller.execute_trial(plan, .05, 1, 0)
    assert order == ['measure', 'persist']
    saved = controller.store.read_json(run_id, 'trials/flow_0.050_repeat_1_positioning.json')
    assert saved['status'] == 'READY'
