from __future__ import annotations
from dataclasses import asdict, dataclass
import numpy as np
from .curve_fit import MODEL_DEGREES, fit_model
from .omron_reference import omron_reference

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
        try: final=fit_model(name,[r["sensor_voltage_v"] for r in repeats],[r["reference_flow_lpm"] for r in repeats],(lo,hi))
        except (ValueError,FloatingPointError,np.linalg.LinAlgError) as exc: warnings.append(str(exc)); continue
        e=np.array([x["error_lpm"] for x in errors]);
        if not len(e): continue
        scores.append(ModelScore(name,final.coefficients,degree+1,float(np.mean(abs(e))),float(np.sqrt(np.mean(e*e))),float(max(abs(e))),final.monotonic,errors,warnings+final.warnings))
    oe=[]
    for r in repeats: oe.append({"target_flow_lpm":r["target_flow_lpm"],"actual_lpm":r["reference_flow_lpm"],"error_lpm":float(omron_reference(r["sensor_voltage_v"])-r["reference_flow_lpm"])})
    e=np.array([x["error_lpm"] for x in oe]); scores.append(ModelScore("omron_reference",None,0,float(np.mean(abs(e))),float(np.sqrt(np.mean(e*e))),float(max(abs(e))),True,oe,[]))
    return scores

def select_empirical_model(scores, improvement_required=0.05):
    eligible=sorted((s for s in scores if s.name!="omron_reference" and s.monotonic),key=lambda s:s.parameter_count)
    if not eligible: raise ValueError("no physically valid empirical model")
    chosen=eligible[0]
    for candidate in eligible[1:]:
        required=0.15 if candidate.name=="polynomial_5" else improvement_required
        if candidate.cv_rmse_lpm < chosen.cv_rmse_lpm*(1-required): chosen=candidate
    return chosen
