from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class StrokePlan:
    """A complete, physically bounded syringe stroke."""

    target_flow_lpm: float
    start_volume_ml: float
    end_volume_ml: float
    stroke_ml: float
    expected_duration_s: float
    direction: int = -1


def select_stroke(
    target_flow_lpm: float,
    settings: dict,
    available_span_ml: float | None = None,
    *,
    minimum_volume_ml: float | None = None,
    maximum_volume_ml: float | None = None,
) -> StrokePlan:
    """Choose a central, safe stroke whose duration is within configured limits.

    ``available_span_ml`` is retained for compatibility with earlier callers. New
    callers should pass the actual intersection of mechanical and SoftPot bounds.
    """
    if target_flow_lpm <= 0:
        raise ValueError("target flow must be positive")
    low = 0.0 if minimum_volume_ml is None else float(minimum_volume_ml)
    high = low + float(available_span_ml) if maximum_volume_ml is None else float(maximum_volume_ml)
    if high <= low:
        raise ValueError("no safe calibrated syringe range is available")
    span = high - low
    margin = min(float(settings.get("working_margin_ml", 2.0)), span * 0.1)
    usable_low, usable_high = low + margin, high - margin
    usable_span = usable_high - usable_low
    ml_s = target_flow_lpm * 1000.0 / 60.0
    desired = ml_s * float(settings.get("target_duration_s", 45.0))
    duration_low = ml_s * float(settings.get("min_duration_s", 20.0))
    duration_high = ml_s * float(settings.get("max_duration_s", 60.0))
    lower = max(float(settings.get("min_stroke_ml", 2.0)), duration_low)
    upper = min(float(settings.get("max_stroke_ml", 60.0)), duration_high, usable_span)
    if upper <= 0 or upper < lower:
        # At very low flows the minimum stroke can imply a longer duration. It is
        # safer to retain the minimum physical stroke than create no plan.
        lower = float(settings.get("min_stroke_ml", 2.0))
        upper = min(float(settings.get("max_stroke_ml", 60.0)), usable_span)
    if upper < lower:
        raise ValueError("safe calibrated range is too small for the minimum stroke")
    stroke = min(max(desired, lower), upper)
    center = (usable_low + usable_high) / 2.0
    start = center + stroke / 2.0
    end = center - stroke / 2.0
    return StrokePlan(target_flow_lpm, start, end, stroke, stroke / ml_s)


def directional_preload_position(plan, settings, minimum_volume_ml, maximum_volume_ml):
    """Validate the existing stroke without shrinking it to make preload fit.

    Reserve a fixed clearance inside both mechanical and calibrated limits, plus
    the existing start tolerance so an accepted endpoint cannot use that margin.
    Legacy configurations receive the same provisional defaults as config.yaml.
    """
    distance = float(settings.get("directional_preload_ml", 3.0))
    clearance = float(settings.get("preload_clearance_ml", 2.0))
    tolerance = float(settings.get("position_tolerance_ml", 0.5))
    attempts = settings.get("directional_preload_attempts", 2)
    if (not all(math.isfinite(v) for v in (distance, clearance, tolerance))
            or tolerance <= 0 or distance <= 2 * tolerance or clearance <= 0):
        raise ValueError("Preload distance must exceed twice the position tolerance; clearance must be positive")
    if isinstance(attempts, bool) or not isinstance(attempts, int) or not 1 <= attempts <= 3:
        raise ValueError("directional_preload_attempts must be an integer from 1 to 3")
    if plan.direction not in (-1, 1):
        raise ValueError("invalid measurement direction")
    preload = plan.start_volume_ml - plan.direction * distance
    low, high = minimum_volume_ml + clearance, maximum_volume_ml - clearance
    if (not all(math.isfinite(v) for v in (preload, plan.start_volume_ml, plan.end_volume_ml,
                                             minimum_volume_ml, maximum_volume_ml))
            or low >= high
            or min(preload, plan.start_volume_ml, plan.end_volume_ml) - tolerance < low
            or max(preload, plan.start_volume_ml, plan.end_volume_ml) + tolerance > high):
        raise ValueError(
            f"Insufficient room for directional preload: stroke {plan.start_volume_ml:.3f} to "
            f"{plan.end_volume_ml:.3f} ml, preload {preload:.3f} ml; "
            f"clearance range {low:g}–{high:g} ml with {tolerance:g} ml tolerance"
        )
    return preload
