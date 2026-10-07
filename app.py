import logging
from pathlib import Path
import yaml
from flask import Flask
from hardware.ads1115 import ADS1115
from hardware.flow_sensor import FlowSensor
from hardware.softpot import PositionReader,SoftPotMapping
from hardware.stepper import Stepper
from storage.runs import RunStore
from web.routes import create_blueprint

def create_app(config_path="config.yaml"):
    config=yaml.safe_load(Path(config_path).read_text()); store=RunStore(config["data_dir"]); hardware={"ready":False,"errors":[]}; controller=None; softpot_session=None
    try:
        stepper=Stepper(config["stepper"]); hardware["stepper"]="Ready"
    except Exception as exc: hardware["errors"].append(f"Stepper/pigpio: {exc}"); stepper=None
    adc_error=None
    try:
        adc=ADS1115(config["ads1115"]); flow=FlowSensor(adc,config["ads1115"]["flow_channel"]); hardware["flow_voltage_v"]=flow.voltage(); hardware["softpot_voltage_v"]=adc.voltage(config["ads1115"]["softpot_channel"])
    except Exception as exc: adc_error=str(exc); hardware["errors"].append(f"ADS1115: {adc_error}"); adc=flow=None
    calibration=store.latest_softpot()
    if calibration and adc:
        try:
            mapping=SoftPotMapping(calibration["points"])
            read_position=PositionReader(adc,config["ads1115"]["softpot_channel"],mapping,config["safety"])
            hardware["softpot_calibrated"]=True
        except Exception as exc: hardware["errors"].append(f"SoftPot calibration: {exc}")
    else: hardware["errors"].append("SoftPot calibration: no valid calibration")
    hardware["ready"]=not hardware["errors"]
    if hardware["ready"]: controller=__import__("calibration.controller",fromlist=["CalibrationController"]).CalibrationController(flow,read_position,stepper,store,config)
    runtime={"controller":controller}
    def activate_softpot(data):
        if not adc:
            detail=f": {adc_error}" if adc_error else ""
            raise RuntimeError(f"ADS1115 unavailable{detail}")
        mapping=SoftPotMapping(data["points"])
        active_reader=PositionReader(adc,config["ads1115"]["softpot_channel"],mapping,config["safety"])
        if runtime["controller"]:
            runtime["controller"].install_position_reader(active_reader)
        elif flow and stepper:
            runtime["controller"]=__import__("calibration.controller",fromlist=["CalibrationController"]).CalibrationController(flow,active_reader,stepper,store,config)
        hardware["softpot_calibrated"]=True
        hardware["errors"]=[e for e in hardware["errors"] if not e.startswith("SoftPot calibration:")]
        hardware["ready"]=not hardware["errors"]
    app=Flask(__name__,template_folder="web/templates",static_folder="web/static"); app.config.update(GASFLOW_CONFIG=config); app.register_blueprint(create_blueprint(store,runtime,hardware,adc,config,activate_softpot)); return app

if __name__=="__main__":
    logging.basicConfig(level=logging.INFO); app=create_app(); cfg=app.config["GASFLOW_CONFIG"]["server"]; app.run(host=cfg["host"],port=cfg["port"],debug=False,use_reloader=False)
