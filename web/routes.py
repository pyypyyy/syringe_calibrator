from datetime import datetime,timezone
import json
import logging
from pathlib import Path
from flask import Blueprint,abort,jsonify,redirect,render_template,request,send_from_directory,url_for
from calibration.softpot_calibration import SoftPotCalibrationSession

log = logging.getLogger(__name__)

def create_blueprint(store,controller,hardware,adc,config,activate_softpot=None):
    bp=Blueprint("web",__name__); sessions={}; runtime=controller if isinstance(controller,dict) else {"controller":controller}
    @bp.get("/")
    def dashboard(): return render_template("dashboard.html",hardware=hardware,history=store.history()[:1])
    @bp.get("/softpot")
    def softpot(): return render_template("softpot.html",points=sessions.get("current").points if sessions.get("current") else [])
    @bp.post("/api/softpot/capture")
    def capture_softpot():
        if not adc:
            detail=next((error for error in hardware["errors"] if error.startswith("ADS1115:")),None)
            return jsonify(error=detail or "ADS1115 unavailable"),503
        channel=config["ads1115"]["softpot_channel"]
        session=sessions.setdefault("current",SoftPotCalibrationSession(lambda:adc.voltage(channel))); point=session.capture(float(request.json["volume_ml"])); return jsonify(point)
    @bp.post("/api/softpot/save")
    def save_softpot():
        session=sessions.get("current")
        if not session:return jsonify(error="no points captured"),400
        path=store.root/"softpot"/f"softpot_calibration_{datetime.now(timezone.utc):%Y-%m-%d_%H%M%S}.json"
        try:data=session.save(path)
        except ValueError as exc:return jsonify(error=str(exc)),400
        if activate_softpot:
            try:
                activate_softpot(data)
            except Exception:
                # Keep the detailed exception in the instrument log without
                # exposing internals or a traceback to the browser.
                log.exception("SoftPot calibration was saved but activation failed")
                return jsonify(
                    error=("SoftPot calibration was saved, but the position reader "
                           "could not be activated. Check the instrument log."),
                    saved=True,
                ),500
        return jsonify({**data,"message":"SoftPot calibration saved and active."})
    @bp.post("/api/calibration/start")
    def start():
        active=runtime["controller"]
        if not active:return jsonify(error="hardware is not ready",hardware=hardware),503
        data=request.json; active.start(data["gas"],[float(v) for v in data["targets_lpm"]],int(data["repeats"])); return jsonify(active.status()),202
    @bp.post("/api/calibration/stop")
    def stop():
        if runtime["controller"]:runtime["controller"].stop(bool((request.json or {}).get("emergency")))
        return jsonify(ok=True)
    @bp.get("/api/status")
    def status(): return jsonify(runtime["controller"].status() if runtime["controller"] else {"state":"HARDWARE_ERROR","hardware":hardware})
    @bp.get("/calibration")
    def calibration(): return render_template("calibration.html")
    @bp.get("/history")
    def history(): return render_template("history.html",runs=store.history())
    @bp.get("/results/<run_id>")
    def results(run_id):
        try:analysis=store.read_json(run_id,"analysis.json")
        except (FileNotFoundError,ValueError):abort(404)
        return render_template("results.html",analysis=analysis)
    @bp.get("/download/<run_id>/<kind>")
    def download(run_id,kind):
        names={"analysis":"analysis.json","points":"calibration_points.csv","trials":"trial_summary.csv"}
        if kind=="compact":
            a=store.read_json(run_id,"analysis.json"); s=a["selected_model"]; payload={"sensor_model":"Omron D6F-P0010A1","gas":a["gas"],"selected_model_type":s["name"],"coefficients":s["coefficients"],"valid_voltage_range_v":a["valid_voltage_range_v"],"valid_flow_range_lpm":a["valid_flow_range_lpm"],"cv_rmse_lpm":s["cv_rmse_lpm"],"generated_at":datetime.now(timezone.utc).isoformat(),"calibration_run_id":run_id,"bootstrap":{k:a["bootstrap"][k] for k in ("requested_iterations","accepted_iterations","rejected_iterations","rejection_fraction")}}; return jsonify(payload)
        if kind not in names:abort(404)
        return send_from_directory(store.path(run_id),names[kind],as_attachment=True)
    return bp
