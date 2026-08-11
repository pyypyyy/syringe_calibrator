import csv,logging,threading,time
from dataclasses import asdict,dataclass
from enum import Enum
from pathlib import Path
import numpy as np
from analysis.model_selection import compare_models,select_empirical_model
from analysis.quality import robust_repeat_outliers
from analysis.reference_flow import estimate_reference_flow
from analysis.uncertainty import bootstrap_band
from .calibration_plan import select_stroke

log=logging.getLogger(__name__)
class State(str,Enum):
    IDLE="IDLE"; HARDWARE_ERROR="HARDWARE_ERROR"; ZERO_BEFORE="ZERO_BEFORE"; POSITIONING="POSITIONING"; SETTLING="SETTLING"; MEASURING="MEASURING"; RETURNING="RETURNING"; ZERO_AFTER="ZERO_AFTER"; ANALYZING="ANALYZING"; COMPLETE="COMPLETE"; STOPPING="STOPPING"; FAILED="FAILED"
@dataclass
class Status:
    state:State=State.IDLE; run_id:str|None=None; gas:str|None=None; current_target_lpm:float|None=None; repeat:int=0; completed:int=0; total:int=0; accepted:int=0; rejected:int=0; elapsed_s:float=0; position_ml:float|None=None; sensor_voltage_v:float|None=None; message:str=""

class CalibrationController:
    """Single worker and explicit state machine; injected objects are real hardware in production."""
    def __init__(self,flow_sensor,position_reader,stepper,store,config):
        self.flow_sensor=flow_sensor; self.position_reader=position_reader; self.stepper=stepper; self.store=store; self.config=config; self._lock=threading.Lock(); self._status=Status(); self._stop=threading.Event(); self._worker=None
    def status(self):
        with self._lock:return {**asdict(self._status),"state":self._status.state.value,"progress":self._status.completed/self._status.total if self._status.total else 0}
    def _set(self,**kwargs):
        with self._lock:
            for k,v in kwargs.items():setattr(self._status,k,v)
    def start(self,gas,targets,repeats):
        if gas not in ("AIR","CO2") or not targets or repeats<1: raise ValueError("invalid calibration request")
        with self._lock:
            if self._worker and self._worker.is_alive(): raise RuntimeError("calibration already running")
            self._stop.clear(); self._worker=threading.Thread(target=self._run,args=(gas,targets,repeats),daemon=True); self._worker.start()
    def stop(self,emergency=False): self._set(state=State.STOPPING,message="Emergency stop" if emergency else "Stop requested"); self._stop.set(); self.stepper.disable()
    def _zero(self,state):
        self._set(state=state); self.stepper.disable(); time.sleep(self.config["calibration"]["settle_s"]); values=[]; end=time.monotonic()+self.config["calibration"]["zero_duration_s"]
        while time.monotonic()<end and not self._stop.wait(self.config["calibration"]["sample_interval_s"]): values.append(self.flow_sensor.voltage())
        return {"mean_voltage_v":float(np.mean(values)),"standard_deviation_v":float(np.std(values,ddof=1)),"sample_count":len(values)}
    def _run(self,gas,targets,repeats):
        started=time.monotonic(); run_id=self.store.create(gas,{"targets_lpm":targets,"repeats":repeats,"configuration":self.config}); summaries=[]
        self._set(run_id=run_id,gas=gas,total=len(targets)*repeats,completed=0,accepted=0,rejected=0)
        try:
            before=self._zero(State.ZERO_BEFORE); self.store.write_json(run_id,"zero_before.json",before)
            for target in targets:
                for repeat in range(1,repeats+1):
                    if self._stop.is_set(): raise InterruptedError("calibration stopped")
                    plan=select_stroke(target,self.config["calibration"],self.config["safety"]["max_volume_ml"]-self.config["safety"]["min_volume_ml"])
                    self._set(state=State.SETTLING,current_target_lpm=target,repeat=repeat); time.sleep(self.config["calibration"]["settle_s"])
                    self._set(state=State.MEASURING); samples=[]; sample_start=time.monotonic(); steps=round(plan.stroke_ml*self.config["axis"]["microsteps_per_ml"]); hz=target*1000/60*self.config["axis"]["microsteps_per_ml"]
                    motion=threading.Thread(target=self.stepper.move,args=(-steps,hz,self._stop)); motion.start()
                    while motion.is_alive() and not self._stop.is_set():
                        now=time.monotonic(); voltage=self.flow_sensor.voltage(); raw,filtered=self.position_reader(); samples.append({"elapsed_s":now-sample_start,"softpot_voltage_v":raw,"filtered_volume_ml":filtered,"flow_sensor_voltage_v":voltage}); self._set(position_ml=filtered,sensor_voltage_v=voltage,elapsed_s=now-started); self._stop.wait(self.config["calibration"]["sample_interval_s"])
                    motion.join(); reg=estimate_reference_flow([s["elapsed_s"] for s in samples],[s["filtered_volume_ml"] for s in samples]); summary={"trial_id":f"flow_{target:.3f}_repeat_{repeat}","target_flow_lpm":target,"repeat":repeat,"reference_flow_lpm":reg.flow_lpm,"sensor_voltage_v":float(np.mean([s["flow_sensor_voltage_v"] for s in samples])),"sensor_voltage_sd_v":float(np.std([s["flow_sensor_voltage_v"] for s in samples],ddof=1)),"stroke_ml":plan.stroke_ml,**reg.to_dict(),"accepted":True,"rejection_reasons":[]}; summaries.append(summary); self.store.write_csv(run_id,f"trials/{summary['trial_id']}.csv",samples); self._set(completed=len(summaries),accepted=sum(s["accepted"] for s in summaries))
                indexes=[i for i,s in enumerate(summaries) if s["target_flow_lpm"]==target]; mask=robust_repeat_outliers([summaries[i]["sensor_voltage_v"] for i in indexes])|robust_repeat_outliers([summaries[i]["reference_flow_lpm"] for i in indexes])
                for i,bad in zip(indexes,mask):
                    if bad:summaries[i]["accepted"]=False;summaries[i]["rejection_reasons"].append("repeat outlier: median/MAD modified z-score > 3.5")
            after=self._zero(State.ZERO_AFTER); self.store.write_json(run_id,"zero_after.json",after); self._set(state=State.ANALYZING)
            accepted=[s for s in summaries if s["accepted"]]; scores=compare_models(accepted,self.config["analysis"]["candidate_models"]); selected=select_empirical_model(scores); band=bootstrap_band(accepted,selected.name,self.config["analysis"]["bootstrap_iterations"])
            drift=after["mean_voltage_v"]-before["mean_voltage_v"]; warnings=[]
            if abs(drift)>0.01:warnings.append("zero drift exceeds 0.010 V")
            analysis={"run_id":run_id,"gas":gas,"status":"WARNING" if warnings else "GOOD","selected_model":selected.to_dict(),"candidate_models":[s.to_dict() for s in scores],"bootstrap":band,"zero_before":before,"zero_after":after,"zero_drift_v":drift,"accepted_trials":len(accepted),"total_trials":len(summaries),"warnings":warnings,"valid_voltage_range_v":[min(s["sensor_voltage_v"] for s in accepted),max(s["sensor_voltage_v"] for s in accepted)],"valid_flow_range_lpm":[min(s["reference_flow_lpm"] for s in accepted),max(s["reference_flow_lpm"] for s in accepted)],"trials":summaries}
            points=[]
            for target in sorted({s["target_flow_lpm"] for s in accepted}):
                rows=[s for s in accepted if s["target_flow_lpm"]==target]
                points.append({"target_flow_lpm":target,"mean_reference_flow_lpm":float(np.mean([s["reference_flow_lpm"] for s in rows])),"reference_flow_sd_lpm":float(np.std([s["reference_flow_lpm"] for s in rows],ddof=1)) if len(rows)>1 else 0.0,"mean_sensor_voltage_v":float(np.mean([s["sensor_voltage_v"] for s in rows])),"sensor_voltage_sd_v":float(np.std([s["sensor_voltage_v"] for s in rows],ddof=1)) if len(rows)>1 else 0.0,"accepted_repeats":len(rows)})
            self.store.write_csv(run_id,"calibration_points.csv",points); self.store.write_csv(run_id,"trial_summary.csv",summaries); self.store.write_json(run_id,"analysis.json",analysis); self._set(state=State.COMPLETE,accepted=len(accepted),rejected=len(summaries)-len(accepted))
        except Exception as exc:
            log.exception("calibration failed"); self._set(state=State.FAILED,message=str(exc))
        finally:self.stepper.disable()
