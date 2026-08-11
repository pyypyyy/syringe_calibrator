from datetime import datetime,timezone
import json,time
import numpy as np
from hardware.softpot import SoftPotMapping
class SoftPotCalibrationSession:
    def __init__(self,read_voltage): self.read_voltage=read_voltage; self.points=[]
    def capture(self,volume_ml,duration_s=.75,interval_s=.02):
        readings=[]; end=time.monotonic()+duration_s
        while time.monotonic()<end: readings.append(self.read_voltage()); time.sleep(interval_s)
        point={"volume_ml":float(volume_ml),"mean_voltage_v":float(np.mean(readings)),"standard_deviation_v":float(np.std(readings,ddof=1)),"sample_count":len(readings)}; self.points=[p for p in self.points if p["volume_ml"]!=volume_ml]+[point]; return point
    def save(self,path):
        validation={"errors":[],"warnings":[]}
        if len(self.points)<3: validation["errors"].append("at least three calibration positions are required")
        voltages=[p["mean_voltage_v"] for p in self.points]
        if voltages and np.ptp(voltages)<0.25: validation["errors"].append("SoftPot voltage span is inadequate")
        if len(set(round(v,6) for v in voltages)) != len(voltages): validation["errors"].append("duplicate or degenerate voltages")
        if any(p.get("standard_deviation_v",0)>0.05 for p in self.points): validation["errors"].append("one or more calibration points are excessively noisy")
        try: SoftPotMapping(self.points)
        except ValueError as exc: validation["errors"].append(str(exc))
        if validation["errors"]: raise ValueError("; ".join(validation["errors"]))
        payload={"created_at":datetime.now(timezone.utc).isoformat(),"valid":True,"validation":validation,"points":self.points}
        path.write_text(json.dumps(payload,indent=2)); return payload
