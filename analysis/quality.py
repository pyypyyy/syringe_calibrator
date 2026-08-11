import numpy as np

def robust_repeat_outliers(values, threshold=3.5):
    """Return outlier mask using median/MAD; threshold 3.5 modified z-score."""
    x=np.asarray(values,float); median=np.median(x); mad=np.median(np.abs(x-median))
    if mad < 1e-12: return np.zeros(len(x),dtype=bool)
    return 0.6745*np.abs(x-median)/mad > threshold

def trial_rejection_reasons(summary, minimum_samples=100, minimum_duration_s=20, minimum_r_squared=.98):
    reasons=[]
    if summary["sample_count"] < minimum_samples: reasons.append("insufficient sample count")
    if summary["duration_s"] < minimum_duration_s: reasons.append("insufficient measurement duration")
    if summary["r_squared"] < minimum_r_squared: reasons.append("poor reference-flow regression")
    if not summary.get("direction_valid",False): reasons.append("movement direction invalid")
    if not summary.get("position_in_range",False): reasons.append("position outside allowed range")
    return reasons
