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
        SoftPotMapping(self.points); payload={"created_at":datetime.now(timezone.utc).isoformat(),"valid":True,"points":self.points}; path.write_text(json.dumps(payload,indent=2)); return payload
