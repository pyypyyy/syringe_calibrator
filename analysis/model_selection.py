from __future__ import annotations
from dataclasses import asdict, dataclass
import numpy as np
from .curve_fit import MODEL_DEGREES, fit_model
from .omron_reference import omron_reference

# Improvements below ten microlitres/minute are numerical noise for this
# instrument, not evidence that another polynomial parameter is useful.
MINIMUM_ABSOLUTE_CV_RMSE_IMPROVEMENT_LPM = 1e-5

@dataclass
class ModelScore:
    name:str; coefficients:list[float]|None; parameter_count:int; cv_mae_lpm:float; cv_rmse_lpm:float
    cv_max_absolute_error_lpm:float; monotonic:bool; errors:list[dict]; warnings:list[str]
    def to_dict(self): return asdict(self)

def compare_models(repeats, candidates=None):
    """Leave one entire target-flow level out, preventing repeat leakage."""
    candidates=candidates or list(MODEL_DEGREES); levels=sorted({r["target_flow_lpm"] for r in repeats}); scores=[]
    lo=min(r["sensor_voltage_v"] for r in repeats); hi=max(r["sensor_voltage_v"] for r in repeats)
    for name in candidates:
        degree=MODEL_DEGREES[name]; errors=[]; warnings=[]
        for held in levels:
            train=[r for r in repeats if r["target_flow_lpm"] != held]; test=[r for r in repeats if r["target_flow_lpm"] == held]
            grouped=[]
            for level in sorted({r["target_flow_lpm"] for r in train}):
                rows=[r for r in train if r["target_flow_lpm"]==level]
                grouped.append((np.mean([r["sensor_voltage_v"] for r in rows]),np.mean([r["reference_flow_lpm"] for r in rows])))
            try: model=fit_model(name,*zip(*grouped),voltage_range=(lo,hi))
            except (ValueError,FloatingPointError,np.linalg.LinAlgError): continue
            for row in test: errors.append({"target_flow_lpm":held,"actual_lpm":row["reference_flow_lpm"],"error_lpm":float(model.predict(row["sensor_voltage_v"])-row["reference_flow_lpm"])})
        # Final fitting uses one unweighted calibration anchor per flow level,
        # consistent with the CV training folds.
        final_points=[]
        for level in levels:
            rows=[r for r in repeats if r["target_flow_lpm"]==level]
            final_points.append((np.mean([r["sensor_voltage_v"] for r in rows]),np.mean([r["reference_flow_lpm"] for r in rows])))
        try: final=fit_model(name,*zip(*final_points),(lo,hi))
        except (ValueError,FloatingPointError,np.linalg.LinAlgError) as exc: warnings.append(str(exc)); continue
        e=np.array([x["error_lpm"] for x in errors]);
        if not len(e): continue
        grid=np.linspace(lo,hi,500); predicted=np.asarray(final.predict(grid)); physical=final.monotonic
        if np.min(predicted) < -0.05:
            warnings.append("curve predicts significantly negative flow inside calibrated range"); physical=False
        measured_max=max(r["reference_flow_lpm"] for r in repeats)
        if np.max(predicted) > max(2.0, measured_max*3):
            warnings.append("curve has implausible edge predictions"); physical=False
        scores.append(ModelScore(name,final.coefficients,degree+1,float(np.mean(abs(e))),float(np.sqrt(np.mean(e*e))),float(max(abs(e))),physical,errors,warnings+final.warnings))
    oe=[]
    for r in repeats: oe.append({"target_flow_lpm":r["target_flow_lpm"],"actual_lpm":r["reference_flow_lpm"],"error_lpm":float(omron_reference(r["sensor_voltage_v"])-r["reference_flow_lpm"])})
    e=np.array([x["error_lpm"] for x in oe]); scores.append(ModelScore("omron_reference",None,0,float(np.mean(abs(e))),float(np.sqrt(np.mean(e*e))),float(max(abs(e))),True,oe,[]))
    return scores

def select_empirical_model(
    scores,
    improvement_required=0.05,
    absolute_improvement_floor_lpm=MINIMUM_ABSOLUTE_CV_RMSE_IMPROVEMENT_LPM,
):
    eligible=sorted((s for s in scores if s.name!="omron_reference" and s.monotonic),key=lambda s:s.parameter_count)
    if not eligible: raise ValueError("no physically valid empirical model")
    chosen=eligible[0]
    for candidate in eligible[1:]:
        required=0.15 if candidate.name=="polynomial_5" else improvement_required
        relative_ok = candidate.cv_rmse_lpm < chosen.cv_rmse_lpm * (1 - required)
        absolute_ok = (
            chosen.cv_rmse_lpm - candidate.cv_rmse_lpm
            > absolute_improvement_floor_lpm
        )
        if relative_ok and absolute_ok:
            chosen=candidate
    return chosen
