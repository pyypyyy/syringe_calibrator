from dataclasses import asdict, dataclass
import numpy as np

MODEL_DEGREES={"linear":1,"quadratic":2,"cubic":3,"polynomial_5":5}
@dataclass(frozen=True)
class FittedModel:
    name:str; degree:int; coefficients:list[float]; monotonic:bool; warnings:list[str]
    def predict(self, voltage): return np.polyval(self.coefficients, voltage)
    def to_dict(self): return asdict(self)

def monotonic_over(coefficients, voltage_range, samples=500):
    derivative=np.polyder(coefficients); grid=np.linspace(*voltage_range,samples)
    return bool(np.all(np.polyval(derivative,grid)>=-1e-9))

def fit_model(name, voltage, flow, voltage_range=None):
    degree=MODEL_DEGREES[name]; x=np.asarray(voltage,float); y=np.asarray(flow,float)
    if len(np.unique(x)) < degree+1: raise ValueError(f"{name} needs {degree+1} distinct points")
    with np.errstate(all="raise"):
        coeff=np.polyfit(x,y,degree).tolist()
    vr=voltage_range or (float(x.min()),float(x.max())); warnings=[]
    mono=monotonic_over(coeff,vr)
    if not mono: warnings.append("curve decreases inside calibrated voltage range")
    if not np.all(np.isfinite(coeff)): warnings.append("non-finite coefficients")
    return FittedModel(name,degree,coeff,mono,warnings)

def predict_checked(model, voltage, valid_range):
    if voltage < valid_range[0] or voltage > valid_range[1]: raise ValueError("voltage is outside calibrated range")
    return float(model.predict(voltage))
