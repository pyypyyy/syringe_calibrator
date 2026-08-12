import logging
import threading
import time
from dataclasses import asdict, dataclass
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
from .calibration_plan import StrokePlan, select_stroke

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


class CalibrationController:
    """Own the complete hardware workflow; hardware objects are injected for tests."""

    def __init__(self, flow_sensor, position_reader, stepper, store, config):
        self.flow_sensor = flow_sensor
        self.position_reader = position_reader
        self.stepper = stepper
        self.store = store
        self.config = config
        self._lock = threading.Lock()
        self._status = Status()
        self._stop = threading.Event()
        self._emergency = threading.Event()
        self._worker = None
        self.state_history: list[str] = []

    def status(self):
        with self._lock:
            status = asdict(self._status)
            status["state"] = self._status.state.value
            status["progress"] = self._status.completed / self._status.total if self._status.total else 0
            return status

    def _set(self, **kwargs):
        with self._lock:
            old = self._status.state
            for key, value in kwargs.items():
                setattr(self._status, key, value)
            if "state" in kwargs and kwargs["state"] != old:
                self.state_history.append(kwargs["state"].value)

    def start(self, gas, targets, repeats):
        if gas not in ("AIR", "CO2") or not targets or repeats < 1:
            raise ValueError("invalid calibration request")
        with self._lock:
            if self._worker and self._worker.is_alive():
                raise RuntimeError("calibration already running")
            self._stop.clear()
            self._emergency.clear()
            self.state_history.clear()
            self._worker = threading.Thread(
                target=self._run, args=(gas, targets, repeats), daemon=True
            )
            self._worker.start()

    def install_position_reader(self, position_reader):
        """Atomically activate a newly validated SoftPot mapping while idle."""
        with self._lock:
            if self._worker and self._worker.is_alive():
                raise RuntimeError("cannot replace SoftPot mapping during calibration")
            self.position_reader = position_reader

    def stop(self, emergency=False):
        """Request cancellation; emergency shutdown occurs before bookkeeping."""
        if emergency:
            self._emergency.set()
            # Stepper.disable() calls pigpio wave_tx_stop immediately.
            self.stepper.disable()
        self._stop.set()
        self._set(state=State.STOPPING, message="Emergency stop" if emergency else "Stop requested")
        if not emergency:
            self.stepper.disable()

    def _check_stopped(self):
        if self._stop.is_set():
            raise InterruptedError("calibration stopped by user")

    def _safe_position(self) -> tuple[float, float]:
        """Read authoritative feedback and validate the calibrated/mechanical range."""
        try:
            raw_voltage, volume = self.position_reader()
        except Exception as exc:
            self.stepper.disable()
            raise MotionSafetyError(f"Invalid SoftPot position feedback: {exc}") from exc
        low, high = self._position_bounds()
        if not np.isfinite(volume) or not low <= volume <= high:
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
        return select_stroke(
            target_flow_lpm,
            self.config.get("calibration", {}),
            minimum_volume_ml=low,
            maximum_volume_ml=high,
        )

    def _validate_target(self, target_ml: float):
        low, high = self._position_bounds()
        if not low <= target_ml <= high:
            raise MotionSafetyError(
                f"Commanded position {target_ml:.3f} ml is outside safe calibrated range {low:g}–{high:g} ml."
            )

    def move_to_volume(self, target_ml: float, state: State = State.POSITIONING) -> float:
        """Move in feedback-controlled chunks with direction and stall watchdogs."""
        self._validate_target(target_ml)
        self._set(state=state)
        calibration = self.config.get("calibration", {})
        safety = self.config.get("safety", {})
        tolerance = float(calibration.get("position_tolerance_ml", 0.5))
        chunk_ml = float(calibration.get("positioning_chunk_ml", 0.5))
        speed = float(calibration.get("positioning_speed_lpm", 0.1))
        microsteps = float(self.config.get("axis", {}).get("microsteps_per_ml", 208))
        frequency = speed * 1000 / 60 * microsteps
        minimum_change = float(safety.get("minimum_position_change_ml", 0.1))
        stall_timeout = float(safety.get("stall_timeout_s", 2.0))
        wrong_tolerance = float(safety.get("wrong_direction_tolerance_ml", 0.5))
        _, current = self._safe_position()
        expected_sign = 1 if target_ml > current else -1
        progress_position, progress_time = current, time.monotonic()
        opposite_total = 0.0
        while abs(target_ml - current) > tolerance:
            self._check_stopped()
            expected_sign = 1 if target_ml > current else -1
            amount = min(chunk_ml, abs(target_ml - current))
            steps = round(expected_sign * amount * microsteps)
            before = current
            self.stepper.move(steps, frequency, self._stop)
            self._check_stopped()
            _, current = self._safe_position()
            delta = current - before
            if delta * expected_sign < 0:
                opposite_total += abs(delta)
            else:
                opposite_total = max(0.0, opposite_total - abs(delta))
            if opposite_total > wrong_tolerance:
                self.stepper.disable()
                direction = "increasing" if expected_sign > 0 else "decreasing"
                raise MotionSafetyError(
                    "SoftPot position moved persistently in the opposite direction. "
                    f"Expected {direction} volume; observed {current - progress_position:+.3f} ml."
                )
            if abs(current - progress_position) >= minimum_change:
                progress_position, progress_time = current, time.monotonic()
            elif time.monotonic() - progress_time >= stall_timeout:
                self.stepper.disable()
                raise MotionSafetyError(
                    f"Motion stall detected: SoftPot position changed less than {minimum_change:g} ml "
                    f"in {stall_timeout:g} seconds."
                )
        self.stepper.disable()
        return current

    def capture_zero(self, state: State) -> dict:
        self._set(state=state)
        self.stepper.disable()
        calibration = self.config.get("calibration", {})
        if self._stop.wait(float(calibration.get("settle_s", 2.0))):
            self._check_stopped()
        values = []
        started = time.monotonic()
        end = started + float(calibration.get("zero_duration_s", 5.0))
        interval = float(calibration.get("sample_interval_s", 0.05))
        while time.monotonic() < end:
            self._check_stopped()
            values.append(self.flow_sensor.voltage())
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
        motion = threading.Thread(target=self.stepper.move, args=(steps, hz, self._stop), daemon=True)
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
                raw, volume = self._safe_position()
                voltage = float(self.flow_sensor.voltage())
                samples.append({
                    "elapsed_s": now - started,
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
                self._stop.wait(float(cfg.get("sample_interval_s", 0.05)))
        except Exception:
            self.stepper.disable()
            self._stop.set()
            raise
        finally:
            motion.join(timeout=2.0)
            self.stepper.disable()
        return samples

    def execute_trial(self, plan: StrokePlan, target: float, repeat: int, started_run: float) -> tuple[dict, list[dict]]:
        self._validate_target(plan.start_volume_ml)
        self._validate_target(plan.end_volume_ml)
        reached = self.move_to_volume(plan.start_volume_ml, State.POSITIONING)
        tolerance = float(self.config.get("calibration", {}).get("position_tolerance_ml", 0.5))
        if abs(reached - plan.start_volume_ml) > tolerance:
            raise MotionSafetyError("measurement start position was not reached within tolerance")
        self._set(state=State.SETTLING, current_target_lpm=target, repeat=repeat)
        if self._stop.wait(float(self.config.get("calibration", {}).get("settle_s", 2.0))):
            self._check_stopped()
        self._set(state=State.MEASURING)
        trial_started = time.monotonic()
        samples = self._watch_measurement(plan, trial_started)
        # A stop can terminate the motion thread between watcher iterations.
        # Preserve cancellation semantics instead of misreporting short partial
        # sampling as a hardware failure.
        self._check_stopped()
        if len(samples) < 3:
            raise MotionSafetyError("measurement produced fewer than three samples")
        _, final_position = self._safe_position()
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
        self._set(elapsed_s=time.monotonic() - started_run)
        return summary, samples

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
        self._set(
            state=State.FAILED if status == "FAILED" else State.COMPLETE,
            accepted=len(accepted),
            rejected=len(summaries) - len(accepted),
            message="; ".join(failures or warnings),
        )

    def _run(self, gas, targets, repeats):
        started = time.monotonic()
        run_id = self.store.create(
            gas,
            {"targets_lpm": targets, "repeats": repeats, "configuration": self.config},
        )
        summaries = []
        self._set(run_id=run_id, gas=gas, total=len(targets) * repeats, completed=0, accepted=0, rejected=0)
        try:
            before = self.capture_zero(State.ZERO_BEFORE)
            self.store.write_json(run_id, "zero_before.json", before)
            reasons = zero_rejection_reasons(before, self.config.get("quality", {}))
            if reasons:
                raise MotionSafetyError("Unstable zero-before: " + "; ".join(reasons))
            target_info = []
            for target in targets:
                plan = self.create_trial_plan(target)
                for repeat in range(1, repeats + 1):
                    self._check_stopped()
                    summary, samples = self.execute_trial(plan, target, repeat, started)
                    summaries.append(summary)
                    self.store.write_csv(run_id, f"trials/{summary['trial_id']}.csv", samples)
                    self._set(
                        completed=len(summaries),
                        accepted=sum(row["accepted"] for row in summaries),
                        rejected=sum(not row["accepted"] for row in summaries),
                    )
                    # Explicitly return after every repeat. The next trial still
                    # verifies/repositions, so feedback rather than step counts wins.
                    self.move_to_volume(plan.start_volume_ml, State.RETURNING)
                target_info.append(self.evaluate_target_repeats(summaries, target, repeats))
            after = self.capture_zero(State.ZERO_AFTER)
            self.store.write_json(run_id, "zero_after.json", after)
            self.finalize_run(run_id, gas, summaries, target_info, before, after)
        except InterruptedError as exc:
            log.info("calibration aborted: %s", exc)
            self.store.write_csv(run_id, "trial_summary.csv", summaries)
            self.store.update_run(run_id, {"completion_state": "ABORTED", "message": str(exc)})
            self._set(state=State.ABORTED, message=str(exc))
        except Exception as exc:
            log.exception("calibration failed")
            self.store.write_csv(run_id, "trial_summary.csv", summaries)
            self.store.update_run(run_id, {"completion_state": "FAILED", "message": str(exc)})
            self._set(state=State.FAILED, message=str(exc))
        finally:
            self.stepper.disable()
