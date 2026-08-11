from dataclasses import dataclass


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
