from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import asdict, dataclass, replace
from copy import deepcopy
from enum import Enum

import numpy as np

from analysis.model_selection import compare_models, select_empirical_model
from analysis.quality import (
    robust_repeat_outliers,
    target_quality,
    trial_rejection_reasons,
    zero_rejection_reasons,
)
from analysis.reference_flow import estimate_reference_flow
from analysis.uncertainty import bootstrap_band
from analysis.omron_reference import omron_reference
from .calibration_plan import StrokePlan, select_stroke, directional_preload_position

log = logging.getLogger(__name__)


class State(str, Enum):
    IDLE = "IDLE"
    HARDWARE_ERROR = "HARDWARE_ERROR"
    ZERO_BEFORE = "ZERO_BEFORE"
    POSITIONING = "POSITIONING"
    SETTLING = "SETTLING"
    MEASURING = "MEASURING"
    RETURNING = "RETURNING"
    ZERO_AFTER = "ZERO_AFTER"
    ANALYZING = "ANALYZING"
    COMPLETE = "COMPLETE"
    STOPPING = "STOPPING"
    ABORTED = "ABORTED"
    FAILED = "FAILED"


class MotionSafetyError(RuntimeError):
    """Raised when position feedback makes continued motion unsafe."""


class DirectionalApproachOvershoot(MotionSafetyError):
    """A bounded retry from preload may recover; never correct in reverse."""


@dataclass
class PositioningBudget:
    limit_steps: int
    used_steps: int = 0

    def reserve(self, steps):
        if self.used_steps + abs(steps) > self.limit_steps:
            raise MotionSafetyError("Positioning pulse/travel budget exhausted (possible stall or inconsistent feedback)")
        self.used_steps += abs(steps)


@dataclass
class Status:
    state: State = State.IDLE
    run_id: str | None = None
    gas: str | None = None
    current_target_lpm: float | None = None
    repeat: int = 0
    completed: int = 0
    total: int = 0
    accepted: int = 0
    rejected: int = 0
    elapsed_s: float = 0
    position_ml: float | None = None
    sensor_voltage_v: float | None = None
    message: str = ""
    repeats_total: int = 0
    target_index: int = 0
    target_count: int = 0
    zero_before_completed: bool = False
    zero_after_completed: bool = False
    analysis_completed: bool = False
    preload_position_ml: float | None = None
    measurement_start_position_ml: float | None = None
    positioning_attempt: int = 0
    positioning_attempts_total: int = 0
    operation_start_position_ml: float | None = None
    operation_target_position_ml: float | None = None
    operation_duration_s: float | None = None
    operation_phase: str | None = None
    failure_operation: str | None = None
    last_completed_trial: dict | None = None


class CalibrationController:
    """Own the complete hardware workflow; hardware objects are injected for tests."""

    def __init__(self, flow_sensor, position_reader, stepper, store, config):
        self.flow_sensor = flow_sensor
        self.position_reader = position_reader
        self.stepper = stepper
        self.store = store
        self.config = config
        self._lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        self._status = Status()
        self._stop = threading.Event()
        self._emergency = threading.Event()
        self._worker = None
        self._motion = None
        self.state_history: list[str] = []
        self._operation_started = time.monotonic()
        self._operation_elapsed = 0.0
        self._operation_progress = None

    def status(self):
        """Snapshot only: never acquire hardware or estimate completion from speed.

        Overall milestones have equal weight: zero-before, each returned trial,
        zero-after, and durable successful analysis. Failed/aborted runs retain
        completed milestones. Motion display uses a high-water mark to suppress
        SoftPot jitter; the measured position itself is never smoothed here.
        """
        with self._lock:
            current = self._status
            status = asdict(current)
            state = current.state
            status["state"] = state.value
            milestones = (current.completed + int(current.zero_before_completed)
                          + int(current.zero_after_completed) + int(current.analysis_completed))
            status["progress"] = milestones / (current.total + 3) if current.total else 0
            status["remaining"] = max(0, current.total - current.completed)
            terminal = state in (State.COMPLETE, State.FAILED, State.ABORTED, State.STOPPING,
                                 State.IDLE, State.HARDWARE_ERROR)
            elapsed = self._operation_elapsed if terminal else time.monotonic() - self._operation_started
            status["operation_elapsed_s"] = max(0.0, elapsed)
            duration = current.operation_duration_s
            status["operation_remaining_s"] = (max(0.0, duration - elapsed)
                                                if duration is not None else None)
            progress = None
            start, target, position = (current.operation_start_position_ml,
                                       current.operation_target_position_ml, current.position_ml)
            if not terminal:
                if state in (State.POSITIONING, State.MEASURING, State.RETURNING):
                    if all(v is not None and np.isfinite(v) for v in (start, target, position)):
                        if target != start:
                            progress = min(1.0, max(0.0, (position - start) / (target - start)))
                        elif position == target:
                            progress = 1.0
                elif duration is not None:
                    progress = min(1.0, max(0.0, elapsed / duration)) if duration > 0 else 1.0
                if progress is not None:
                    self._operation_progress = max(self._operation_progress or 0.0, progress)
            status["operation_progress"] = (self._operation_progress
                                            if terminal or progress is not None else None)
            status["operation_direction"] = (
                "Increasing volume" if target > start else "Decreasing volume" if target < start else "Stationary"
            ) if start is not None and target is not None else None
            descriptions = {
                State.IDLE: "Ready to start calibration",
                State.ZERO_BEFORE: "Measuring zero-flow sensor voltage before calibration",
                State.ZERO_AFTER: "Measuring zero-flow sensor voltage after calibration",
                State.POSITIONING: "Moving piston to starting position",
                State.SETTLING: "Allowing the system to settle — piston stopped",
                State.MEASURING: "Measuring sensor response" + (
                    f" at {current.current_target_lpm:.3f} L/min" if current.current_target_lpm is not None else ""),
                State.RETURNING: "Returning piston to starting position",
                State.ANALYZING: "Calculating calibration curve and uncertainty",
                State.COMPLETE: "Calibration completed successfully",
                State.STOPPING: "Stop requested — stopping motor and terminating calibration",
                State.ABORTED: "Calibration stopped by user",
                State.FAILED: "Calibration failed",
                State.HARDWARE_ERROR: "Hardware is not ready",
            }
            if state == State.POSITIONING:
                descriptions[state] = {
                    "planning": "Validating directional preload and stroke limits",
                    "pre_positioning": "Pre-positioning: moving piston to preload position",
                    "directional_approach": "Directional approach: moving to measurement start",
                }.get(current.operation_phase, descriptions[state])
            if state == State.RETURNING:
                descriptions[state] = "Returning piston to preload position"
            status["operation_description"] = descriptions[state]
            return status

    def _set(self, **kwargs):
        with self._lock:
            old = self._status.state
            state = kwargs.get("state", old)
            # Keep Stop visible while the worker unwinds. Hardware cancellation
            # remains owned by the existing event and stepper shutdown path.
            if old == State.STOPPING and state not in (State.ABORTED, State.FAILED, State.COMPLETE):
                kwargs = {key: value for key, value in kwargs.items()
                          if key != "state" and not key.startswith("operation_")}
                state = old
            if state != old:
                if old not in (State.STOPPING, State.FAILED, State.ABORTED, State.COMPLETE):
                    self._operation_elapsed = time.monotonic() - self._operation_started
                if state == State.FAILED and self._status.failure_operation is None:
                    self._status.failure_operation = old.value
                if state not in (State.FAILED, State.ABORTED, State.STOPPING, State.COMPLETE):
                    self._operation_started = time.monotonic()
                    self._operation_progress = None
                    self._status.operation_start_position_ml = None
                    self._status.operation_target_position_ml = None
                    self._status.operation_duration_s = None
                    self._status.operation_phase = None
            if state == State.COMPLETE:
                self._status.analysis_completed = True
            for key, value in kwargs.items():
                setattr(self._status, key, value)
            if "state" in kwargs and kwargs["state"] != old:
                self.state_history.append(kwargs["state"].value)

    def _zero_phase(self, phase, duration):
        with self._lock:
            if self._status.state == State.STOPPING:
                return
            self._status.operation_phase = phase
            self._status.operation_duration_s = duration
            self._operation_started = time.monotonic()
            self._operation_progress = None

    def start(self, gas, targets, repeats):
        if (gas not in ("AIR", "CO2") or not targets or repeats < 1
                or not isinstance(repeats, int)
                or any(not np.isfinite(t) or t <= 0 for t in targets)):
            raise ValueError("invalid calibration request")
        with self._lifecycle_lock:
            if ((self._worker and self._worker.is_alive())
                    or (self._motion and self._motion.is_alive())):
                raise RuntimeError("calibration already running")
            self._stop.clear()
            self._emergency.clear()
            self.state_history.clear()
            with self._lock:
                self._status = Status(gas=gas, total=len(targets) * repeats,
                                      repeats_total=repeats, target_count=len(targets))
                self._operation_started = time.monotonic()
                self._operation_elapsed = 0.0
                self._operation_progress = None
            self._worker = threading.Thread(
                target=self._run, args=(gas, targets, repeats), daemon=True
            )
            self._worker.start()

    def install_position_reader(self, position_reader):
        """Atomically activate a newly validated SoftPot mapping while idle."""
        with self._lifecycle_lock:
            if ((self._worker and self._worker.is_alive())
                    or (self._motion and self._motion.is_alive())):
                raise RuntimeError("cannot replace SoftPot mapping during calibration")
            self.position_reader = position_reader

    def stop(self, emergency=False):
        """Request cancellation; emergency shutdown occurs before bookkeeping."""
        # A new start cannot clear cancellation before shutdown finishes.
        with self._lifecycle_lock:
            self._stop.set()
            if emergency:
                self._emergency.set()
            try:
                self.stepper.disable()
            except Exception as exc:
                self._set(state=State.FAILED, message=f"Motor shutdown failed: {exc}")
                raise
            with self._lock:
                # The worker may have reached a terminal state during disable().
                # Do not overwrite that outcome with a stale STOPPING update.
                if self._worker and self._worker.is_alive() and self._status.state not in (
                        State.ABORTED, State.FAILED, State.COMPLETE):
                    if self._status.state != State.STOPPING:
                        self._operation_elapsed = time.monotonic() - self._operation_started
                        self._status.failure_operation = self._status.state.value
                    self._status.state = State.STOPPING
                    self._status.message = "Emergency stop" if emergency else "Stop requested"
                    self.state_history.append(State.STOPPING.value)

    def _check_stopped(self):
        if self._stop.is_set():
            raise InterruptedError("calibration stopped by user")

    def _safe_position(self, fresh=False) -> tuple[float, float]:
        """Read authoritative feedback and validate the calibrated/mechanical range."""
        try:
            reader = getattr(self.position_reader, "fresh", self.position_reader) if fresh else self.position_reader
            raw_voltage, volume = reader()
        except Exception as exc:
            self._stop.set()
            self.stepper.disable()
            raise MotionSafetyError(f"Invalid SoftPot position feedback: {exc}") from exc
        low, high = self._position_bounds()
        if not np.isfinite(volume) or not low <= volume <= high:
            self._stop.set()
            self.stepper.disable()
            raise MotionSafetyError(
                f"SoftPot position {volume!r} ml is outside safe calibrated range {low:g}–{high:g} ml."
            )
        self._set(position_ml=float(volume))
        return float(raw_voltage), float(volume)

    def _position_bounds(self) -> tuple[float, float]:
        safety = self.config.get("safety", {})
        calibrated_low = getattr(self.position_reader, "min_calibrated_volume_ml", safety.get("min_volume_ml", 0))
        calibrated_high = getattr(self.position_reader, "max_calibrated_volume_ml", safety.get("max_volume_ml", 100))
        low = max(float(safety.get("min_volume_ml", 0)), float(calibrated_low))
        high = min(float(safety.get("max_volume_ml", 100)), float(calibrated_high))
        if high <= low:
            raise MotionSafetyError("mechanical and calibrated SoftPot ranges do not overlap")
        return low, high

    def create_trial_plan(self, target_flow_lpm: float) -> StrokePlan:
        low, high = self._position_bounds()
        plan = select_stroke(
            target_flow_lpm,
            self.config.get("calibration", {}),
            minimum_volume_ml=low,
            maximum_volume_ml=high,
        )
        self.preload_position(plan)
        return plan

    def preload_position(self, plan):
        low, high = self._position_bounds()
        try:
            return directional_preload_position(plan, self.config.get("calibration", {}), low, high)
        except ValueError as exc:
            raise MotionSafetyError(str(exc)) from exc

    def _validate_target(self, target_ml: float):
        low, high = self._position_bounds()
        if not low <= target_ml <= high:
            raise MotionSafetyError(
                f"Commanded position {target_ml:.3f} ml is outside safe calibrated range {low:g}–{high:g} ml."
            )

    def move_to_volume(self, target_ml: float, state: State = State.POSITIONING,
                       *, approach_direction=None, budget=None) -> float:
        """Feedback chunks with finite pulse/travel bounds and existing watchdogs.

        approach_direction fixes the command sign. Crossing the target within the
        existing tolerance is accepted without reversal; crossing beyond it raises
        a retryable error. General pre-position/return moves may still correct.
        """
        try:
            self._check_stopped()
            self._validate_target(target_ml)
            self._set(state=state, operation_target_position_ml=target_ml)
            calibration = self.config.get("calibration", {})
            safety = self.config.get("safety", {})
            allowed_error = float(calibration.get("position_tolerance_ml", 0.5))
            tolerance = allowed_error / 2
            chunk_ml = float(calibration.get("positioning_chunk_ml", 0.5))
            speed = float(calibration.get("positioning_speed_lpm", 0.1))
            microsteps = float(self.config.get("axis", {}).get("microsteps_per_ml", 208))
            if (not all(math.isfinite(v) and v > 0 for v in (allowed_error, chunk_ml, speed, microsteps))
                    or approach_direction not in (None, -1, 1)):
                raise MotionSafetyError("Invalid bounded positioning configuration")
            frequency = speed * 1000 / 60 * microsteps
            minimum_change = float(safety.get("minimum_position_change_ml", 0.1))
            stall_timeout = float(safety.get("stall_timeout_s", 2.0))
            wrong_tolerance = float(safety.get("wrong_direction_tolerance_ml", 0.5))
            _, current = self._safe_position(fresh=True)
            self._set(operation_start_position_ml=current, operation_target_position_ml=target_ml)
            local_budget = PositioningBudget(math.ceil(
                (abs(target_ml - current) + 2 * chunk_ml + 2 * allowed_error) * microsteps))
            progress_position, progress_time = current, time.monotonic()
            opposite_total = 0.0
            while True:
                self._check_stopped()
                if approach_direction is not None:
                    remaining = (target_ml - current) * approach_direction
                    if remaining < -allowed_error:
                        raise DirectionalApproachOvershoot(
                            f"Directional approach overshot start {target_ml:.3f} ml: "
                            f"measured {current:.3f} ml; tolerance {allowed_error:g} ml"
                        )
                    # Aim inside a quarter of the unchanged acceptance tolerance,
                    # leaving more headroom for stationary noise without reversal.
                    if remaining <= allowed_error / 4:
                        if not local_budget.used_steps:
                            raise MotionSafetyError("Directional approach ended without a movement command")
                        return current
                    expected_sign = approach_direction
                else:
                    if abs(target_ml - current) <= tolerance:
                        return current
                    expected_sign = 1 if target_ml > current else -1
                amount = min(chunk_ml, abs(target_ml - current), 65535 / microsteps)
                steps = round(expected_sign * amount * microsteps)
                if not steps:
                    raise MotionSafetyError("Positioning cannot resolve target within tolerance")
                local_budget.reserve(steps)
                if budget is not None:
                    budget.reserve(steps)
                # Check after bookkeeping too: never issue another chunk after cancellation.
                self._check_stopped()
                before = current
                self.stepper.move(steps, frequency, self._stop)
                self._check_stopped()
                _, current = self._safe_position(fresh=True)
                delta = current - before
                if delta * expected_sign < 0:
                    opposite_total += abs(delta)
                else:
                    opposite_total = max(0.0, opposite_total - abs(delta))
                if opposite_total > wrong_tolerance:
                    direction = "increasing" if expected_sign > 0 else "decreasing"
                    raise MotionSafetyError(
                        "SoftPot position moved persistently in the opposite direction. "
                        f"Expected {direction} volume; observed {current - progress_position:+.3f} ml."
                    )
                if abs(current - progress_position) >= minimum_change:
                    progress_position, progress_time = current, time.monotonic()
                elif time.monotonic() - progress_time >= stall_timeout:
                    raise MotionSafetyError(
                        f"Motion stall detected: SoftPot position changed less than {minimum_change:g} ml "
                        f"in {stall_timeout:g} seconds."
                    )
        finally:
            self.stepper.disable()

    def _positioning_phase(self, phase, attempt, attempts, preload, start):
        self._set(state=State.POSITIONING, operation_phase=phase,
                  preload_position_ml=preload, measurement_start_position_ml=start,
                  positioning_attempt=attempt, positioning_attempts_total=attempts)
        with self._lock:
            if self._status.state != State.STOPPING:
                self._operation_started = time.monotonic()
                self._operation_progress = None
                self._status.operation_start_position_ml = None
                self._status.operation_target_position_ml = None

    def prepare_trial(self, plan, target, repeat):
        """Preload, approach in one direction, settle, then verify fresh feedback.

        Only an out-of-tolerance overshoot is retried. Other feedback faults and
        cancellation fail immediately. No flow samples are collected here.
        """
        cfg = self.config.get("calibration", {})
        diagnostics = {
            "target_flow_lpm": target, "repeat": repeat,
            "configured_preload_ml": cfg.get("directional_preload_ml", 3.0),
            "measurement_start_ml": plan.start_volume_ml,
            "preload_target_ml": None, "preload_reached_ml": None,
            "start_after_approach_ml": None, "start_after_settling_ml": None,
            "settling_displacement_ml": None, "attempts": [], "warnings": [],
            "status": "POSITIONING", "phase": "pre_positioning",
        }
        budget = None
        try:
            self._check_stopped()
            self._set(state=State.POSITIONING, operation_phase="pre_positioning",
                      measurement_start_position_ml=plan.start_volume_ml)
            preload = self.preload_position(plan)
            diagnostics["preload_target_ml"] = preload
            attempts = cfg.get("directional_preload_attempts", 2)
            tolerance = float(cfg.get("position_tolerance_ml", 0.5))
            chunk = float(cfg.get("positioning_chunk_ml", 0.5))
            microsteps = float(self.config.get("axis", {}).get("microsteps_per_ml", 208))
            if not all(math.isfinite(v) and v > 0 for v in (chunk, microsteps)):
                raise MotionSafetyError("Invalid bounded positioning configuration")
            self._positioning_phase("pre_positioning", 1, attempts, preload, plan.start_volume_ml)
            _, initial = self._safe_position(fresh=True)
            distance = float(diagnostics["configured_preload_ml"])
            # Shared cap covers initial transit and every permitted recovery leg.
            travel_limit = (abs(preload - initial) + (2 * attempts - 1) * distance
                            + 2 * attempts * (chunk + tolerance))
            budget = PositioningBudget(math.ceil(travel_limit * microsteps))
            diagnostics["pulse_limit"] = budget.limit_steps
            diagnostics["travel_limit_ml"] = budget.limit_steps / microsteps
            for attempt in range(1, attempts + 1):
                self._check_stopped()
                diagnostics["phase"] = "pre_positioning"
                self._positioning_phase("pre_positioning", attempt, attempts, preload, plan.start_volume_ml)
                reached_preload = self.move_to_volume(preload, budget=budget)
                diagnostics["preload_reached_ml"] = reached_preload
                entry = {"attempt": attempt, "preload_reached_ml": reached_preload}
                diagnostics["attempts"].append(entry)
                if (plan.start_volume_ml - reached_preload) * plan.direction <= tolerance:
                    raise MotionSafetyError("Preload did not leave room for a directional approach")
                diagnostics["phase"] = "directional_approach"
                self._positioning_phase("directional_approach", attempt, attempts, preload, plan.start_volume_ml)
                try:
                    reached = self.move_to_volume(plan.start_volume_ml,
                        approach_direction=plan.direction, budget=budget)
                except DirectionalApproachOvershoot as exc:
                    entry["error"] = str(exc)
                    diagnostics["warnings"].append(str(exc))
                    if attempt == attempts:
                        raise MotionSafetyError(f"Directional approach failed after {attempts} attempts: {exc}") from exc
                    continue
                entry["start_after_approach_ml"] = reached
                diagnostics["start_after_approach_ml"] = reached
                if abs(reached - plan.start_volume_ml) > tolerance:
                    raise MotionSafetyError("measurement start position was not reached within tolerance")
                if (reached - plan.start_volume_ml) * plan.direction > 0:
                    diagnostics["warnings"].append("Approach crossed target within tolerance; no reverse correction")
                break
            diagnostics["phase"] = "settling"
            self._set(state=State.SETTLING,
                      operation_duration_s=float(cfg.get("settle_s", 2.0)))
            if self._stop.wait(float(cfg.get("settle_s", 2.0))):
                self._check_stopped()
            self._check_stopped()
            _, settled = self._safe_position(fresh=True)
            diagnostics["start_after_settling_ml"] = settled
            diagnostics["settling_displacement_ml"] = settled - reached
            if (abs(settled - plan.start_volume_ml) > tolerance
                    or abs(settled - reached) > tolerance):
                raise MotionSafetyError(
                    "measurement start position changed during settling: "
                    f"approach {reached:.3f}, settled {settled:.3f}, target {plan.start_volume_ml:.3f} ml"
                )
            if abs(settled - reached) > tolerance / 2:
                diagnostics["warnings"].append(
                    f"Position shifted {settled - reached:+.3f} ml during settling (within tolerance)"
                )
            self._check_stopped()
            diagnostics["status"] = "READY"
            return settled, diagnostics
        except Exception as exc:
            diagnostics["status"] = "ABORTED" if isinstance(exc, InterruptedError) else "FAILED"
            diagnostics["error"] = str(exc)
            self.stepper.disable()
            raise
        finally:
            diagnostics["commanded_steps"] = budget.used_steps if budget else 0
            diagnostics["approach_attempts"] = len(diagnostics["attempts"])
            with self._lock:
                run_id = self._status.run_id
                diagnostics["last_measured_position_ml"] = self._status.position_ml
            # Successful diagnostics are persisted after measurement, never in
            # the gap between fresh settling verification and starting the stroke.
            if run_id and "error" in diagnostics:
                try:
                    self._save_positioning_diagnostics(target, repeat, diagnostics)
                except Exception:
                    log.exception("could not persist failed positioning diagnostics")

    def _save_positioning_diagnostics(self, target, repeat, diagnostics):
        with self._lock:
            run_id = self._status.run_id
        if run_id:
            self.store.write_json(run_id,
                f"trials/flow_{target:.3f}_repeat_{repeat}_positioning.json", diagnostics)

    def capture_zero(self, state: State) -> dict:
        self._set(state=state)
        self.stepper.disable()
        calibration = self.config.get("calibration", {})
        self._zero_phase("settling", float(calibration.get("settle_s", 2.0)))
        if self._stop.wait(float(calibration.get("settle_s", 2.0))):
            self._check_stopped()
        self._zero_phase("recording", float(calibration.get("zero_duration_s", 5.0)))
        values = []
        started = time.monotonic()
        end = started + float(calibration.get("zero_duration_s", 5.0))
        interval = float(calibration.get("sample_interval_s", 0.05))
        while time.monotonic() < end:
            self._check_stopped()
            voltage = self.flow_sensor.voltage()
            values.append(voltage)
            self._set(sensor_voltage_v=float(voltage))
            self._stop.wait(interval)
        duration = time.monotonic() - started
        return {
            "mean_voltage_v": float(np.mean(values)),
            "standard_deviation_v": float(np.std(values, ddof=1)) if len(values) > 1 else 0.0,
            "sample_count": len(values),
            "duration_s": duration,
        }

    def _watch_measurement(self, plan: StrokePlan, started: float) -> list[dict]:
        cfg = self.config.get("calibration", {})
        safety = self.config.get("safety", {})
        microsteps = float(self.config.get("axis", {}).get("microsteps_per_ml", 208))
        steps = round(plan.direction * plan.stroke_ml * microsteps)
        hz = plan.target_flow_lpm * 1000 / 60 * microsteps
        motion_errors = []
        def run_motion():
            try:
                self.stepper.move(steps, hz, self._stop)
            except Exception as exc:
                motion_errors.append(exc)
        motion = threading.Thread(target=run_motion, daemon=True)
        self._motion = motion
        samples = []
        motion.start()
        last_progress = plan.start_volume_ml
        progress_time = time.monotonic()
        opposite = 0.0
        previous = plan.start_volume_ml
        try:
            while motion.is_alive():
                self._check_stopped()
                now = time.monotonic()
                if now - started > plan.expected_duration_s + 2.0:
                    raise MotionSafetyError("Stepper motion timed out")
                raw, volume = self._safe_position()
                sampled = time.monotonic()
                voltage = float(self.flow_sensor.voltage())
                samples.append({
                    "elapsed_s": sampled - started,
                    "softpot_voltage_v": raw,
                    "filtered_volume_ml": volume,
                    "flow_sensor_voltage_v": voltage,
                    "motion_phase": "MEASURING",
                    "analysis_sample": False,
                })
                self._set(sensor_voltage_v=voltage)
                delta = volume - previous
                if delta * plan.direction < 0:
                    opposite += abs(delta)
                else:
                    opposite = max(0.0, opposite - abs(delta))
                wrong_limit = float(safety.get("wrong_direction_tolerance_ml", 0.5))
                if opposite > wrong_limit:
                    raise MotionSafetyError(
                        "SoftPot position moved persistently in the opposite direction. "
                        f"Expected decreasing volume; observed {volume - plan.start_volume_ml:+.3f} ml."
                    )
                minimum = float(safety.get("minimum_position_change_ml", 0.1))
                if abs(volume - last_progress) >= minimum:
                    last_progress, progress_time = volume, now
                elif now - progress_time >= float(safety.get("stall_timeout_s", 2.0)):
                    raise MotionSafetyError(
                        f"Motion stall detected: SoftPot position changed less than {minimum:g} ml "
                        f"in {float(safety.get('stall_timeout_s', 2.0)):g} seconds."
                    )
                previous = volume
                # The interval is between acquisitions, not an extra delay on
                # top of two ADC conversions. Slow conversions still reduce
                # sample count and remain subject to the existing quality gates.
                next_sample = now + float(cfg.get("sample_interval_s", 0.05))
                self._stop.wait(max(0.0, next_sample - time.monotonic()))
        except Exception:
            self._stop.set()
            self.stepper.disable()
            raise
        finally:
            motion.join(timeout=2.0)
            self.stepper.disable()
        if motion.is_alive():
            self._stop.set()
            raise MotionSafetyError("Stepper motion thread did not terminate")
        if motion_errors:
            raise MotionSafetyError(f"Stepper motion failed: {motion_errors[0]}") from motion_errors[0]
        return samples

    def execute_trial(self, plan: StrokePlan, target: float, repeat: int, started_run: float) -> tuple[dict, list[dict]]:
        self._set(current_target_lpm=target, repeat=repeat)
        reached, positioning = self.prepare_trial(plan, target, repeat)
        try:
            actual_stroke = (plan.end_volume_ml - reached) * plan.direction
            if actual_stroke <= 0:
                raise MotionSafetyError("measurement start is past the planned endpoint")
            plan = replace(plan, start_volume_ml=reached, stroke_ml=actual_stroke,
                           expected_duration_s=actual_stroke / (target * 1000 / 60))
            self._set(state=State.MEASURING, operation_start_position_ml=plan.start_volume_ml,
                      operation_target_position_ml=plan.end_volume_ml)
            trial_started = time.monotonic()
            samples = self._watch_measurement(plan, trial_started)
            # A stop can terminate the motion thread between watcher iterations.
            # Preserve cancellation semantics instead of misreporting short partial
            # sampling as a hardware failure.
            self._check_stopped()
            if len(samples) < 3:
                raise MotionSafetyError("measurement produced fewer than three samples")
            _, final_position = self._safe_position(fresh=True)
            endpoint_tolerance = max(
                float(self.config.get("calibration", {}).get("position_tolerance_ml", 0.5)),
                float(self.config.get("calibration", {}).get("positioning_chunk_ml", 0.5)),
            )
            if abs(final_position - plan.end_volume_ml) > endpoint_tolerance:
                raise MotionSafetyError(
                    f"measurement end position was not reached: expected {plan.end_volume_ml:.3f} ml, "
                    f"observed {final_position:.3f} ml"
                )
            summary = self.analyze_trial(plan, target, repeat, samples)
            # Authoritative stationary endpoint readings, not commanded displacement.
            summary["measured_stroke_ml"] = abs(final_position - reached)
            summary["positioning"] = positioning
            self._set(elapsed_s=time.monotonic() - started_run)
            return summary, samples
        except Exception as exc:
            positioning["trial_error"] = str(exc)
            raise
        finally:
            try:
                self._save_positioning_diagnostics(target, repeat, positioning)
            except Exception:
                if "trial_error" not in positioning:
                    raise
                log.exception("could not persist positioning diagnostics after trial failure")

    def analyze_trial(self, plan, target, repeat, samples):
        window = self.config.get("analysis_window", {})
        start_fraction = float(window.get("discard_start_fraction", 0.1))
        end_fraction = float(window.get("discard_end_fraction", 0.1))
        count = len(samples)
        first = min(count - 1, int(np.floor(count * start_fraction)))
        last = max(first + 1, int(np.ceil(count * (1 - end_fraction))))
        stable = samples[first:last]
        if len(stable) < 3:
            stable = samples
        stable_ids = {id(sample) for sample in stable}
        for sample in samples:
            sample["analysis_sample"] = id(sample) in stable_ids
        regression = estimate_reference_flow(
            [sample["elapsed_s"] for sample in stable],
            [sample["filtered_volume_ml"] for sample in stable],
        )
        voltages = [sample["flow_sensor_voltage_v"] for sample in stable]
        expected_sign = plan.direction
        stable_fraction = max(0.0, 1.0 - start_fraction - end_fraction)
        planned_duration = plan.expected_duration_s * stable_fraction
        interval = float(self.config.get("calibration", {}).get("sample_interval_s", 0.05))
        planned_samples = planned_duration / interval
        quality = self.config.get("quality", {})
        acceptable = float(quality.get("planned_data_acceptable_fraction", 0.75))
        required_duration = min(float(quality.get("minimum_duration_s", 20.0)), planned_duration * acceptable)
        required_samples = max(3, min(int(quality.get("minimum_samples", 100)), int(np.floor(planned_samples * acceptable))))
        summary = {
            "trial_id": f"flow_{target:.3f}_repeat_{repeat}",
            "target_flow_lpm": target,
            "repeat": repeat,
            "reference_flow_lpm": regression.flow_lpm,
            "sensor_voltage_v": float(np.mean(voltages)),
            "sensor_voltage_sd_v": float(np.std(voltages, ddof=1)) if len(voltages) > 1 else 0.0,
            "stroke_ml": plan.stroke_ml,
            "stroke_start_ml": plan.start_volume_ml,
            "stroke_end_ml": plan.end_volume_ml,
            "raw_sample_count": count,
            "stable_region_sample_count": len(stable),
            "stable_region_start_s": stable[0]["elapsed_s"],
            "stable_region_end_s": stable[-1]["elapsed_s"],
            "stable_region_start_position_ml": stable[0]["filtered_volume_ml"],
            "stable_region_end_position_ml": stable[-1]["filtered_volume_ml"],
            "planned_stable_duration_s": planned_duration,
            "actual_stable_duration_s": float(stable[-1]["elapsed_s"] - stable[0]["elapsed_s"]),
            "minimum_acceptable_duration_s": required_duration,
            "planned_stable_sample_count": planned_samples,
            "actual_stable_sample_count": len(stable),
            "minimum_acceptable_sample_count": required_samples,
            **regression.to_dict(),
            "direction_valid": regression.slope_ml_s * expected_sign > 0,
            "expected_slope_sign": expected_sign,
            "position_in_range": True,
            "softpot_valid": True,
            "rejection_category": None,
        }
        reasons = trial_rejection_reasons(summary, {**quality, "minimum_samples": required_samples,
                                                     "minimum_duration_s": required_duration})
        summary["accepted"] = not reasons
        summary["rejection_reasons"] = reasons
        if reasons:
            summary["rejection_category"] = "trial_qc"
        return summary

    def evaluate_target_repeats(self, summaries, target, requested):
        indexes = [
            index for index, row in enumerate(summaries)
            if row["target_flow_lpm"] == target and row["accepted"]
        ]
        if indexes:
            quality = self.config.get("quality", {})
            sensor = robust_repeat_outliers(
                [summaries[index]["sensor_voltage_v"] for index in indexes],
                zero_mad_absolute_tolerance=float(
                    quality.get("repeat_voltage_zero_mad_tolerance_v", 0.002)
                ),
            )
            flow = robust_repeat_outliers(
                [summaries[index]["reference_flow_lpm"] for index in indexes],
                zero_mad_absolute_tolerance=float(
                    quality.get("repeat_flow_zero_mad_tolerance_lpm", 0.005)
                ),
            )
            for index, bad in zip(indexes, sensor | flow):
                if bad:
                    summaries[index]["accepted"] = False
                    summaries[index]["rejection_category"] = "repeat_outlier"
                    summaries[index]["rejection_reasons"].append(
                        "repeat outlier: median/MAD modified z-score > 3.5"
                    )
        accepted = sum(row["accepted"] for row in summaries if row["target_flow_lpm"] == target)
        return {
            "target_flow_lpm": target,
            "requested_repeats": requested,
            "accepted_repeats": accepted,
            "rejected_repeats": requested - accepted,
            "quality": target_quality(accepted),
        }

    @staticmethod
    def _calibration_points(summaries, target_diagnostics):
        points = []
        quality_by_target = {row["target_flow_lpm"]: row["quality"] for row in target_diagnostics}
        for target in sorted({row["target_flow_lpm"] for row in summaries}):
            rows = [row for row in summaries if row["target_flow_lpm"] == target and row["accepted"]]
            if not rows:
                continue
            flows = [row["reference_flow_lpm"] for row in rows]
            volts = [row["sensor_voltage_v"] for row in rows]
            points.append({
                "target_flow_lpm": target,
                "mean_reference_flow_lpm": float(np.mean(flows)),
                "reference_flow_sd_lpm": float(np.std(flows, ddof=1)) if len(rows) > 1 else 0.0,
                "mean_sensor_voltage_v": float(np.mean(volts)),
                "sensor_voltage_sd_v": float(np.std(volts, ddof=1)) if len(rows) > 1 else 0.0,
                "accepted_repeats": len(rows),
                "target_quality": quality_by_target[target],
            })
        return points

    def finalize_run(self, run_id, gas, summaries, targets_info, before, after):
        self._set(state=State.ANALYZING)
        accepted = [row for row in summaries if row["accepted"]]
        points = self._calibration_points(summaries, targets_info)
        usable = [point for point in points if point["target_quality"] == "GOOD"]
        usable_targets = {point["target_flow_lpm"] for point in usable}
        model_trials = [row for row in accepted if row["target_flow_lpm"] in usable_targets]
        failures, warnings = [], []
        if len(usable) < 3:
            failures.append("too few usable calibration flow levels")
        if any(row["quality"] == "WEAK" for row in targets_info):
            warnings.append("one or more targets have only one accepted repeat and were excluded")
        if any(not row["accepted"] for row in summaries):
            warnings.append("one or more trials were rejected")
        after_zero_reasons = zero_rejection_reasons(after, self.config.get("quality", {}))
        warnings.extend(after_zero_reasons)
        drift = after["mean_voltage_v"] - before["mean_voltage_v"]
        if abs(drift) > float(self.config.get("quality", {}).get("maximum_zero_drift_v", 0.01)):
            warnings.append("zero drift exceeds configured limit")
        selected_dict = None
        scores = []
        band = None
        if not failures:
            model_rows = [
                {
                    "target_flow_lpm": point["target_flow_lpm"],
                    "sensor_voltage_v": point["mean_sensor_voltage_v"],
                    "reference_flow_lpm": point["mean_reference_flow_lpm"],
                }
                for point in usable
            ]
            try:
                scores = compare_models(model_rows, self.config.get("analysis", {}).get("candidate_models"))
                selected = select_empirical_model(scores)
                selected_dict = selected.to_dict()
                band = bootstrap_band(
                    model_trials,
                    selected.name,
                    int(self.config.get("analysis", {}).get("bootstrap_iterations", 2000)),
                )
                if band["rejection_fraction"] > 0.25:
                    warnings.append("a large fraction of bootstrap fits were rejected")
            except Exception as exc:
                failures.append(f"analysis could not select a physically valid model: {exc}")
        status = "FAILED" if failures else "WARNING" if warnings else "GOOD"
        relative_errors = ([abs(e["error_lpm"] / e["actual_lpm"] * 100)
                            for e in selected_dict["errors"] if abs(e["actual_lpm"]) >= 1e-6]
                           if selected_dict else [])
        repeatability = [point["reference_flow_sd_lpm"] for point in usable]
        omron_grid = band["voltage_v"] if band else []
        analysis = {
            "run_id": run_id,
            "gas": gas,
            "status": status,
            "failure_reasons": failures,
            "warnings": warnings,
            "selected_model": selected_dict,
            "candidate_models": [score.to_dict() for score in scores],
            "bootstrap": band,
            "zero_before": before,
            "zero_after": after,
            "zero_drift_v": drift,
            "accepted_trials": len(accepted),
            "rejected_trials": len(summaries) - len(accepted),
            "total_trials": len(summaries),
            "usable_calibration_points": len(usable),
            "bootstrap_usable_target_flows_lpm": sorted(usable_targets),
            "maximum_relative_error_percent": max(relative_errors, default=None),
            "mean_reference_flow_repeatability_sd_lpm": float(np.mean(repeatability)) if repeatability else None,
            "median_reference_flow_repeatability_sd_lpm": float(np.median(repeatability)) if repeatability else None,
            "omron_reference_series": {"voltage_v": omron_grid, "flow_lpm": [float(omron_reference(v)) for v in omron_grid]},
            "target_diagnostics": targets_info,
            "calibration_points": points,
            "valid_voltage_range_v": [min((p["mean_sensor_voltage_v"] for p in usable), default=0), max((p["mean_sensor_voltage_v"] for p in usable), default=0)],
            "valid_flow_range_lpm": [min((p["mean_reference_flow_lpm"] for p in usable), default=0), max((p["mean_reference_flow_lpm"] for p in usable), default=0)],
            "trials": summaries,
        }
        # All artifacts are durable before COMPLETE becomes observable.
        self.store.write_csv(run_id, "calibration_points.csv", points)
        self.store.write_csv(run_id, "trial_summary.csv", summaries)
        self.store.write_json(run_id, "analysis.json", analysis)
        completion = "FAILED" if status == "FAILED" else "COMPLETE"
        self.store.update_run(run_id, {"completion_state": completion, "analysis_status": status,
                                       "message": "; ".join(failures or warnings)})
        self._check_stopped()
        self._set(
            state=State.FAILED if status == "FAILED" else State.COMPLETE,
            accepted=len(accepted),
            rejected=len(summaries) - len(accepted),
            message="; ".join(failures or warnings),
        )

    def _run(self, gas, targets, repeats):
        started = time.monotonic()
        run_id = None
        summaries = []
        try:
            run_id = self.store.create(
                gas,
                {"targets_lpm": targets, "repeats": repeats, "configuration": self.config},
            )
            self._set(run_id=run_id, gas=gas, total=len(targets) * repeats, completed=0, accepted=0, rejected=0)
            before = self.capture_zero(State.ZERO_BEFORE)
            self.store.write_json(run_id, "zero_before.json", before)
            reasons = zero_rejection_reasons(before, self.config.get("quality", {}))
            if reasons:
                raise MotionSafetyError("Unstable zero-before: " + "; ".join(reasons))
            self._set(zero_before_completed=True)
            target_info = []
            self._set(state=State.POSITIONING, operation_phase="planning")
            plans = []
            for target_index, target in enumerate(targets, 1):
                self._set(current_target_lpm=target, target_index=target_index, repeat=1)
                try:
                    plans.append(self.create_trial_plan(target))
                except MotionSafetyError as exc:
                    self.store.write_json(run_id,
                        f"trials/flow_{target:.3f}_repeat_1_positioning.json", {
                            "status": "FAILED", "phase": "planning", "error": str(exc),
                            "target_flow_lpm": target, "repeat": 1,
                            "configured_preload_ml": self.config.get("calibration", {}).get("directional_preload_ml", 3.0),
                            "approach_attempts": 0, "commanded_steps": 0,
                        })
                    raise
            for target_index, (target, plan) in enumerate(zip(targets, plans), 1):
                self._set(current_target_lpm=target, target_index=target_index, repeat=1)
                for repeat in range(1, repeats + 1):
                    self._check_stopped()
                    summary, samples = self.execute_trial(plan, target, repeat, started)
                    summaries.append(summary)
                    self.store.write_csv(run_id, f"trials/{summary['trial_id']}.csv", samples)
                    # Operational completion requires the return, even if trial QC rejected data.
                    next_plan = plans[target_index] if repeat == repeats and target_index < len(plans) else plan
                    preload = self.preload_position(next_plan)
                    self._set(preload_position_ml=preload,
                              measurement_start_position_ml=next_plan.start_volume_ml)
                    self.move_to_volume(preload, State.RETURNING)
                    self._check_stopped()
                    self._set(
                        completed=len(summaries),
                        accepted=sum(row["accepted"] for row in summaries),
                        rejected=sum(not row["accepted"] for row in summaries),
                        last_completed_trial=deepcopy(summary),
                    )
                target_info.append(self.evaluate_target_repeats(summaries, target, repeats))
                self._set(last_completed_trial=deepcopy(summaries[-1]),
                          accepted=sum(row["accepted"] for row in summaries),
                          rejected=sum(not row["accepted"] for row in summaries))
            after = self.capture_zero(State.ZERO_AFTER)
            self.store.write_json(run_id, "zero_after.json", after)
            self._set(zero_after_completed=True)
            self._check_stopped()
            self.finalize_run(run_id, gas, summaries, target_info, before, after)
        except InterruptedError as exc:
            log.info("calibration aborted: %s", exc)
            self._set(state=State.ABORTED, message=str(exc))
            if run_id is not None:
                self.store.write_csv(run_id, "trial_summary.csv", summaries)
                self.store.update_run(run_id, {"completion_state": "ABORTED", "message": str(exc)})
        except Exception as exc:
            log.exception("calibration failed")
            self._set(state=State.FAILED, message=str(exc))
            if run_id is not None:
                self.store.write_csv(run_id, "trial_summary.csv", summaries)
                self.store.update_run(run_id, {"completion_state": "FAILED", "message": str(exc)})
        finally:
            try:
                self.stepper.disable()
            except Exception as exc:
                log.exception("motor shutdown failed")
                message = f"Motor shutdown failed: {exc}"
                self._set(state=State.FAILED, message=message)
                if run_id is not None:
                    self.store.update_run(run_id, {"completion_state": "FAILED", "message": message})
