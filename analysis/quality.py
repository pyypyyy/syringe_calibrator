import numpy as np


def robust_repeat_outliers(values, threshold=3.5, zero_mad_absolute_tolerance=0.002):
    """Return a median/MAD modified-z-score outlier mask.

    With a zero MAD (common for three repeats), differences within the supplied
    physical tolerance are treated as equivalent instead of comparing them to
    floating-point epsilon.
    """
    x = np.asarray(values, float)
    median = np.median(x)
    mad = np.median(np.abs(x - median))
    if mad < 1e-12:
        return np.abs(x - median) > zero_mad_absolute_tolerance
    difference = np.abs(x - median)
    # The MAD score remains the primary rule.  The dimensional tolerance also
    # prevents a microscopically small but non-zero MAD from amplifying harmless
    # instrument-resolution differences into an outlier.
    return ((0.6745 * difference / mad > threshold)
            & (difference > zero_mad_absolute_tolerance))


def trial_rejection_reasons(summary: dict, quality: dict | None = None, **legacy) -> list[str]:
    """Evaluate all independent trial gates and return every failure reason."""
    q = dict(quality or {})
    q.update(legacy)
    minimum_samples = q.get("minimum_samples", 100)
    minimum_duration = q.get("minimum_duration_s", 20.0)
    minimum_r2 = q.get("minimum_reference_r_squared", q.get("minimum_r_squared", 0.98))
    maximum_residual = q.get("maximum_regression_residual_sd_ml", 0.5)
    maximum_sensor_sd = q.get("maximum_sensor_voltage_sd_v", 0.05)
    reasons = []
    if summary.get("sample_count", 0) < minimum_samples:
        reasons.append("sample count below minimum")
    if summary.get("duration_s", 0.0) < minimum_duration:
        reasons.append("measurement duration below minimum")
    if summary.get("r_squared", 0.0) < minimum_r2:
        reasons.append("reference-flow regression R² below minimum")
    if summary.get("residual_standard_deviation_ml", float("inf")) > maximum_residual:
        reasons.append("reference-flow regression residual SD above limit")
    if summary.get("sensor_voltage_sd_v", float("inf")) > maximum_sensor_sd:
        reasons.append("sensor voltage standard deviation above limit")
    if not summary.get("direction_valid", False):
        reasons.append("movement direction invalid")
    if not summary.get("position_in_range", False):
        reasons.append("position outside allowed range")
    if not summary.get("softpot_valid", False):
        reasons.append("SoftPot signal invalid")
    return reasons


def zero_rejection_reasons(zero: dict, quality: dict | None = None) -> list[str]:
    q = quality or {}
    reasons = []
    if zero.get("sample_count", 0) < q.get("minimum_zero_samples", 3):
        reasons.append("zero-flow sample count below minimum")
    if zero.get("standard_deviation_v", float("inf")) > q.get("max_zero_sd_v", 0.01):
        reasons.append("zero-flow voltage is unstable")
    return reasons


def target_quality(accepted_repeats: int) -> str:
    return "GOOD" if accepted_repeats >= 2 else "WEAK" if accepted_repeats == 1 else "UNUSABLE"
