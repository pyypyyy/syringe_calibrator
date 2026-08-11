import csv,json
from datetime import datetime,timezone
from pathlib import Path
class RunStore:
    def __init__(self,root): self.root=Path(root); (self.root/"runs").mkdir(parents=True,exist_ok=True); (self.root/"softpot").mkdir(parents=True,exist_ok=True)
    def create(self,gas,metadata):
        run_id=f"flow_{gas.lower()}_{datetime.now(timezone.utc):%Y-%m-%d_%H%M%S}"; path=self.root/"runs"/run_id; (path/"trials").mkdir(parents=True); self.write_json(run_id,"run.json",{"run_id":run_id,"created_at":datetime.now(timezone.utc).isoformat(),"gas":gas,**metadata}); return run_id
    def path(self,run_id):
        path=(self.root/"runs"/run_id).resolve()
        if path.parent != (self.root/"runs").resolve(): raise ValueError("invalid run id")
        return path
    def write_json(self,run_id,name,data): self.path(run_id).joinpath(name).write_text(json.dumps(data,indent=2,allow_nan=False))
    def read_json(self,run_id,name): return json.loads(self.path(run_id).joinpath(name).read_text())
    def update_run(self,run_id,updates):
        data=self.read_json(run_id,"run.json"); data.update(updates); self.write_json(run_id,"run.json",data)
    def write_csv(self,run_id,name,rows):
        rows=list(rows); target=self.path(run_id)/name; target.parent.mkdir(parents=True,exist_ok=True)
        if not rows: target.write_text(""); return
        with target.open("w",newline="") as f: writer=csv.DictWriter(f,fieldnames=rows[0]); writer.writeheader(); writer.writerows(rows)
    def history(self):
        result=[]
        for path in sorted((self.root/"runs").glob("*/run.json"),reverse=True):
            try:
                row=json.loads(path.read_text()); analysis_path=path.parent/"analysis.json"
                if analysis_path.exists():
                    analysis=json.loads(analysis_path.read_text()); selected=analysis.get("selected_model") or {}
                    row.update({"status":analysis.get("status"),"valid_flow_range_lpm":analysis.get("valid_flow_range_lpm"),"selected_model":selected.get("name"),"cv_rmse_lpm":selected.get("cv_rmse_lpm"),"max_error_lpm":selected.get("cv_max_absolute_error_lpm")})
                result.append(row)
            except (OSError,json.JSONDecodeError): continue
        return result
    def latest_softpot(self):
        for path in sorted((self.root/"softpot").glob("softpot_calibration_*.json"),reverse=True):
            try:
                data=json.loads(path.read_text())
                if data.get("valid"): return data
            except (OSError,json.JSONDecodeError): continue
        return None
