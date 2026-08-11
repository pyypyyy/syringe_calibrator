from dataclasses import dataclass
@dataclass(frozen=True)
class StrokePlan: target_flow_lpm:float; stroke_ml:float; expected_duration_s:float
def select_stroke(target_flow_lpm, settings, available_span_ml):
    ml_s=target_flow_lpm*1000/60; desired=ml_s*settings["target_duration_s"]
    stroke=max(settings["min_stroke_ml"],min(desired,settings["max_stroke_ml"],available_span_ml))
    duration=stroke/ml_s
    if duration>settings["max_duration_s"]: stroke=max(settings["min_stroke_ml"],ml_s*settings["max_duration_s"])
    return StrokePlan(target_flow_lpm,stroke,stroke/ml_s)
