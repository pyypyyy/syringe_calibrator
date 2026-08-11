import logging
from pathlib import Path
import yaml
from flask import Flask
from hardware.ads1115 import ADS1115
from hardware.flow_sensor import FlowSensor
from hardware.softpot import RobustPositionFilter,SoftPotMapping
from hardware.stepper import Stepper
from storage.runs import RunStore
from web.routes import create_blueprint

def create_app(config_path="config.yaml"):
    config=yaml.safe_load(Path(config_path).read_text()); store=RunStore(config["data_dir"]); hardware={"ready":False,"errors":[]}; controller=None; softpot_session=None
    try:
        stepper=Stepper(config["stepper"]); hardware["stepper"]="Ready"
    except Exception as exc: hardware["errors"].append(f"Stepper/pigpio: {exc}"); stepper=None
    try:
        adc=ADS1115(config["ads1115"]); flow=FlowSensor(adc,config["ads1115"]["flow_channel"]); hardware["flow_voltage_v"]=flow.voltage(); hardware["softpot_voltage_v"]=adc.voltage(config["ads1115"]["softpot_channel"])
    except Exception as exc: hardware["errors"].append(f"ADS1115: {exc}"); adc=flow=None
    calibration=store.latest_softpot()
    if calibration and adc:
        try:
            filt=RobustPositionFilter(SoftPotMapping(calibration["points"]),config["safety"]["max_position_jump_ml"],config["safety"]["jump_persistence"])
            def read_position():
                voltage=adc.voltage(config["ads1115"]["softpot_channel"]); raw,filtered=filt.update(voltage); return voltage,filtered
            hardware["softpot_calibrated"]=True
        except Exception as exc: hardware["errors"].append(f"SoftPot calibration: {exc}")
    else: hardware["errors"].append("SoftPot calibration: no valid calibration")
    hardware["ready"]=not hardware["errors"]
    if hardware["ready"]: controller=__import__("calibration.controller",fromlist=["CalibrationController"]).CalibrationController(flow,read_position,stepper,store,config)
    app=Flask(__name__,template_folder="web/templates",static_folder="web/static"); app.config.update(GASFLOW_CONFIG=config); app.register_blueprint(create_blueprint(store,controller,hardware,adc,config)); return app

if __name__=="__main__":
    logging.basicConfig(level=logging.INFO); app=create_app(); cfg=app.config["GASFLOW_CONFIG"]["server"]; app.run(host=cfg["host"],port=cfg["port"],debug=False,use_reloader=False)
