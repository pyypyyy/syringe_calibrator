"""End-to-end controller tests using a shared, moving test-only syringe plant."""
import csv
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import pytest
import yaml

from calibration.controller import CalibrationController
from storage.runs import RunStore


ROOT = Path(__file__).parents[1]
DEFAULT_TARGETS = [.02, .05, .10, .25, .50, .75, 1.00]


@dataclass
class FakePlant:
    position_ml: float = 50.0
    flow_lpm: float = 0.0
    motor_enabled: bool = False
    motion_started: float = 0.0
    motion_origin_ml: float = 50.0
    velocity_ml_s: float = 0.0
    motion_duration_s: float = 0.0
    continuous_feedback: bool = True
    positions: list[float] = field(default_factory=lambda: [50.0])
    lock: threading.Lock = field(default_factory=threading.Lock)


class FakeStepper:
    def __init__(self, plant, microsteps_per_ml, fault=None):
        self.plant = plant
        self.microsteps_per_ml = microsteps_per_ml
        self.fault = fault
        self.disabled = True
        self.disable_calls = 0
        self.move_calls = 0
        self.stop_time = None

    def disable(self):
        self.disabled = True
        self.disable_calls += 1
        self.stop_time = time.monotonic()
        with self.plant.lock:
            self.plant.motor_enabled = False
            self.plant.flow_lpm = 0.0

    def move(self, steps, frequency_hz, stop_event):
        self.move_calls += 1
        self.disabled = False
        amount_ml = steps / self.microsteps_per_ml
        duration = abs(steps) / frequency_hz
        with self.plant.lock:
            self.plant.motor_enabled = True
            self.plant.flow_lpm = abs(frequency_hz / self.microsteps_per_ml * 60 / 1000)
            start_position = self.plant.position_ml
        started = time.monotonic()
        with self.plant.lock:
            self.plant.motion_started = started
            self.plant.motion_origin_ml = start_position
            self.plant.velocity_ml_s = amount_ml / duration if duration else 0.0
            self.plant.motion_duration_s = duration
            self.plant.continuous_feedback = self.fault is None
        while True:
            if stop_event.is_set() or self.disabled:
                break
            elapsed = time.monotonic() - started
            fraction = min(1.0, elapsed / duration) if duration else 1.0
            with self.plant.lock:
                if self.fault == "stall":
                    position = start_position
                elif self.fault == "wrong_direction":
                    position = start_position - amount_ml * fraction
                elif self.fault == "out_of_range":
                    position = 200.0
                    fraction = 1.0
                else:
                    position = start_position + amount_ml * fraction
                self.plant.position_ml = position
                self.plant.positions.append(self.plant.position_ml)
            # Keep feedback quantization far below the controller's sampling
            # interval so OLS measures the commanded physical velocity.
            if fraction >= 1.0 or stop_event.wait(min(.0001, duration / 5)):
                break
        with self.plant.lock:
            self.plant.motor_enabled = False
            self.plant.flow_lpm = 0.0


class FakePositionReader:
    min_calibrated_volume_ml = 0.0
    max_calibrated_volume_ml = 100.0

    def __init__(self, plant):
        self.plant = plant

    def __call__(self):
        with self.plant.lock:
            if self.plant.motor_enabled and self.plant.continuous_feedback:
                elapsed = min(
                    time.monotonic() - self.plant.motion_started,
                    self.plant.motion_duration_s,
                )
                self.plant.position_ml = (
                    self.plant.motion_origin_ml + self.plant.velocity_ml_s * elapsed
                )
                self.plant.positions.append(self.plant.position_ml)
            position = self.plant.position_ml
        return position / 100 * 3.0, position


class FakeFlowSensor:
    def __init__(self, plant):
        self.plant = plant

    def voltage(self):
        with self.plant.lock:
            flow = self.plant.flow_lpm
        return .2 + .8 * flow


def make_controller(tmp_path, fault=None):
    config = yaml.safe_load((ROOT / "config.yaml").read_text())
    config["axis"]["microsteps_per_ml"] = 1000
    config["calibration"].update({
        "positioning_speed_lpm": 500,
        "positioning_chunk_ml": .1,
        "position_tolerance_ml": .003,
        "sample_interval_s": .002,
        "settle_s": 0,
        "zero_duration_s": .012,
        "min_stroke_ml": .02,
        "max_stroke_ml": 5,
        "target_duration_s": .2,
        "min_duration_s": .16,
        "max_duration_s": .24,
    })
    config["quality"].update({
        "minimum_samples": 10,
        "minimum_duration_s": .04,
        "minimum_zero_samples": 3,
        "planned_data_acceptable_fraction": .65,
        "repeat_flow_zero_mad_tolerance_lpm": .01,
    })
    config["analysis"]["bootstrap_iterations"] = 50
    config["safety"].update({
        "stall_timeout_s": .04,
        "minimum_position_change_ml": .002,
        "wrong_direction_tolerance_ml": .015,
    })
    plant = FakePlant()
    stepper = FakeStepper(plant, config["axis"]["microsteps_per_ml"], fault)
    controller = CalibrationController(
        FakeFlowSensor(plant), FakePositionReader(plant), stepper,
        RunStore(tmp_path), config,
    )
    return controller, plant, stepper


def wait_terminal(controller, timeout=10):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if controller.status()["state"] in {"COMPLETE", "FAILED", "ABORTED"}:
            controller._worker.join(timeout=1)
            return controller.status()
        time.sleep(.002)
    pytest.fail(f"controller did not terminate: {controller.status()}")


def test_full_default_calibration_runs_real_controller_state_machine(tmp_path):
    controller, plant, stepper = make_controller(tmp_path)
    controller.start("AIR", DEFAULT_TARGETS, 3)
    status = wait_terminal(controller)
    assert status["state"] == "COMPLETE", status
    analysis = controller.store.read_json(status["run_id"], "analysis.json")
    assert analysis["status"] in {"GOOD", "WARNING"}
    assert analysis["total_trials"] == analysis["accepted_trials"] == 21
    assert analysis["usable_calibration_points"] == 7
    assert all(row["quality"] == "GOOD" for row in analysis["target_diagnostics"])
    assert analysis["selected_model"]["name"] == "linear"
    assert analysis["bootstrap"]["accepted_iterations"] > 0
    for trial in analysis["trials"]:
        assert trial["accepted"], trial["rejection_reasons"]
        assert trial["reference_flow_lpm"] == pytest.approx(trial["target_flow_lpm"], rel=.035)
        assert trial["stable_region_start_position_ml"] <= trial["stroke_start_ml"] + .05
        assert trial["stable_region_end_position_ml"] >= trial["stroke_end_ml"] - .05
    assert min(plant.positions) >= 0 and max(plant.positions) <= 100
    expected_states = {"ZERO_BEFORE", "POSITIONING", "SETTLING", "MEASURING",
                       "RETURNING", "ZERO_AFTER", "ANALYZING", "COMPLETE"}
    assert expected_states <= set(controller.state_history)
    run_path = controller.store.path(status["run_id"])
    for name in ("analysis.json", "trial_summary.csv", "calibration_points.csv",
                 "zero_before.json", "zero_after.json"):
        assert (run_path / name).exists()
    raw_files = list((run_path / "trials").glob("*.csv"))
    assert len(raw_files) == 21
    with raw_files[0].open() as source:
        assert "analysis_sample" in next(csv.reader(source))
    model = analysis["selected_model"]
    slope, intercept = model["coefficients"]
    assert slope * (.2 + .8 * .5) + intercept == pytest.approx(.5, abs=.015)
    assert stepper.move_calls > 21


@pytest.mark.parametrize(
    "fault, expected",
    [("stall", "stall"), ("wrong_direction", "opposite direction"),
     ("out_of_range", "outside safe calibrated range")],
)
def test_motion_safety_faults_fail_real_workflow(tmp_path, fault, expected):
    controller, _, stepper = make_controller(tmp_path, fault)
    controller.start("AIR", [.5], 1)
    status = wait_terminal(controller)
    assert status["state"] == "FAILED"
    assert expected.lower() in status["message"].lower()
    assert stepper.disabled and stepper.move_calls
    run = controller.store.read_json(status["run_id"], "run.json")
    assert run["completion_state"] == "FAILED"
    assert not (controller.store.path(status["run_id"]) / "analysis.json").exists()


def test_normal_abort_preserves_partial_data_without_successful_model(tmp_path):
    controller, _, stepper = make_controller(tmp_path)
    controller.start("AIR", DEFAULT_TARGETS, 3)
    deadline = time.monotonic() + 3
    while controller.status()["completed"] < 1 and time.monotonic() < deadline:
        time.sleep(.002)
    controller.stop()
    status = wait_terminal(controller)
    assert status["state"] == "ABORTED"
    assert stepper.disabled and not controller._worker.is_alive()
    run = controller.store.read_json(status["run_id"], "run.json")
    assert run["completion_state"] == "ABORTED"
    summary = controller.store.path(status["run_id"]) / "trial_summary.csv"
    assert summary.exists() and summary.read_text()
    assert not (controller.store.path(status["run_id"]) / "analysis.json").exists()


def test_emergency_stop_interrupts_active_measurement(tmp_path):
    controller, _, stepper = make_controller(tmp_path)
    controller.start("AIR", DEFAULT_TARGETS, 3)
    deadline = time.monotonic() + 3
    while controller.status()["state"] != "MEASURING" and time.monotonic() < deadline:
        time.sleep(.001)
    completed = controller.status()["completed"]
    controller.stop(emergency=True)
    stop_time = stepper.stop_time
    status = wait_terminal(controller)
    assert status["state"] == "ABORTED"
    assert stepper.disabled and stop_time is not None
    assert status["completed"] == completed
    assert controller.store.read_json(status["run_id"], "run.json")["completion_state"] == "ABORTED"
    assert not (controller.store.path(status["run_id"]) / "analysis.json").exists()
