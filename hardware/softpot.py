from collections import deque
import numpy as np
class PositionOutOfRange(ValueError): pass
class SoftPotMapping:
    def __init__(self, points):
        self.points=sorted(points,key=lambda p:p["mean_voltage_v"]); volumes=[p["volume_ml"] for p in self.points]
        delta=np.diff(volumes)
        if len(points)<3 or not (np.all(delta>0) or np.all(delta<0)): raise ValueError("SoftPot mapping must contain at least three monotonic points")
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
