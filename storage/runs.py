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
    def write_csv(self,run_id,name,rows):
        rows=list(rows); target=self.path(run_id)/name; target.parent.mkdir(parents=True,exist_ok=True)
        if not rows: target.write_text(""); return
        with target.open("w",newline="") as f: writer=csv.DictWriter(f,fieldnames=rows[0]); writer.writeheader(); writer.writerows(rows)
    def history(self):
        result=[]
        for path in sorted((self.root/"runs").glob("*/run.json"),reverse=True):
            try: result.append(json.loads(path.read_text()))
            except (OSError,json.JSONDecodeError): continue
        return result
    def latest_softpot(self):
        for path in sorted((self.root/"softpot").glob("softpot_calibration_*.json"),reverse=True):
            try:
                data=json.loads(path.read_text())
                if data.get("valid"): return data
            except (OSError,json.JSONDecodeError): continue
        return None
