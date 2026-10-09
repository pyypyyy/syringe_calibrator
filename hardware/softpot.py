from collections import deque
import numpy as np
class PositionOutOfRange(ValueError): pass
class SoftPotMapping:
    def __init__(self, points):
        self.points=sorted(points,key=lambda p:p["mean_voltage_v"]); volumes=[p["volume_ml"] for p in self.points]
        delta=np.diff(volumes)
        voltages=np.asarray([p["mean_voltage_v"] for p in self.points],float)
        if len(points)<3 or not (np.all(delta>0) or np.all(delta<0)): raise ValueError("SoftPot mapping must contain at least three monotonic points")
        if np.any(np.diff(voltages)<=1e-6): raise ValueError("SoftPot voltages must be distinct")
        if np.ptp(voltages)<0.25: raise ValueError("SoftPot voltage span is inadequate")
        self.min_calibrated_volume_ml=float(min(volumes)); self.max_calibrated_volume_ml=float(max(volumes))
    def volume(self, voltage):
        x=[p["mean_voltage_v"] for p in self.points]
        if not x[0]<=voltage<=x[-1]: raise PositionOutOfRange("SoftPot voltage outside calibrated range")
        return float(np.interp(voltage,x,[p["volume_ml"] for p in self.points]))
class RobustPositionFilter:
    def __init__(self,mapping,max_jump_ml=5,persistence=3): self.mapping=mapping; self.window=deque(maxlen=5); self.last=None; self.pending=0; self.max_jump=max_jump_ml; self.persistence=persistence
    def update(self,voltage):
        raw=self.mapping.volume(voltage); self.window.append(raw); candidate=float(np.median(self.window))
        if self.last is not None and abs(candidate-self.last)>self.max_jump:
            self.pending+=1
            if self.pending<self.persistence:return raw,self.last
        self.pending=0; self.last=candidate; return raw,candidate


class PositionReader:
    """Streaming filter during motion; fresh stationary samples at move boundaries."""
    def __init__(self, adc, channel, mapping, safety):
        self.adc, self.channel = adc, channel
        self.filter = RobustPositionFilter(mapping, safety["max_position_jump_ml"], safety["jump_persistence"])
        self.min_calibrated_volume_ml = mapping.min_calibrated_volume_ml
        self.max_calibrated_volume_ml = mapping.max_calibrated_volume_ml

    def __call__(self):
        voltage = self.adc.voltage(self.channel)
        _, filtered = self.filter.update(voltage)
        return voltage, filtered

    def fresh(self):
        # Each ADC read performs a new conversion. Validate every raw reading.
        voltages = [self.adc.voltage(self.channel) for _ in range(5)]
        volumes = [self.filter.mapping.volume(v) for v in voltages]
        if max(volumes) - min(volumes) > self.filter.max_jump:
            raise ValueError("Unstable stationary SoftPot feedback")
        self.filter.window.clear()
        self.filter.window.extend(volumes)
        self.filter.last = float(np.median(volumes))
        self.filter.pending = 0
        return float(np.median(voltages)), self.filter.last
