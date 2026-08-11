import numpy as np
from .curve_fit import fit_model

def bootstrap_band(repeats, model_name, iterations=2000, grid_size=200, seed=None):
    rng=np.random.default_rng(seed); levels=sorted({r["target_flow_lpm"] for r in repeats}); lo=min(r["sensor_voltage_v"] for r in repeats); hi=max(r["sensor_voltage_v"] for r in repeats); grid=np.linspace(lo,hi,grid_size); curves=[]; rejected=0
    for _ in range(iterations):
        points=[]
        for level in levels:
            rows=[r for r in repeats if r["target_flow_lpm"]==level]
            sample=rng.choice(rows,len(rows),replace=True)
            points.append((np.mean([r["sensor_voltage_v"] for r in sample]),np.mean([r["reference_flow_lpm"] for r in sample])))
        try: model=fit_model(model_name,*zip(*points),(lo,hi))
        except (ValueError,FloatingPointError,np.linalg.LinAlgError): rejected+=1; continue
        if not model.monotonic: rejected+=1; continue
        curves.append(model.predict(grid))
    if not curves: raise ValueError("all bootstrap fits were invalid")
    values=np.asarray(curves)
    return {"voltage_v":grid.tolist(),"lower_lpm":np.percentile(values,2.5,axis=0).tolist(),"upper_lpm":np.percentile(values,97.5,axis=0).tolist(),"accepted_iterations":len(curves),"rejected_iterations":rejected}
