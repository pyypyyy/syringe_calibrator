from dataclasses import asdict, dataclass
import numpy as np

@dataclass(frozen=True)
class ReferenceFlowResult:
    flow_lpm: float; slope_ml_s: float; intercept_ml: float; sample_count: int
    r_squared: float; slope_standard_error: float; residual_standard_deviation_ml: float
    duration_s: float; volume_span_ml: float
    def to_dict(self): return asdict(self)

def estimate_reference_flow(elapsed_s, volume_ml) -> ReferenceFlowResult:
    """OLS fit V=a+bt using every accepted stable-region position sample."""
    t, v = np.asarray(elapsed_s, float), np.asarray(volume_ml, float)
    if len(t) != len(v) or len(t) < 3 or not np.all(np.isfinite(t)) or not np.all(np.isfinite(v)):
        raise ValueError("reference regression requires at least three finite paired samples")
    if np.ptp(t) <= 0: raise ValueError("elapsed time must span a positive interval")
    x = t - t.mean(); slope = float(x @ (v-v.mean()) / (x @ x)); intercept = float(v.mean()-slope*t.mean())
    fitted = intercept+slope*t; residual = v-fitted; ss_res=float(residual@residual); ss_tot=float(((v-v.mean())**2).sum())
    dof=len(t)-2; residual_sd=float(np.sqrt(ss_res/dof)); slope_se=float(residual_sd/np.sqrt(x@x))
    return ReferenceFlowResult(abs(slope)*0.06, slope, intercept, len(t), 1-ss_res/ss_tot if ss_tot else 1.0,
                               slope_se, residual_sd, float(np.ptp(t)), float(np.ptp(v)))
