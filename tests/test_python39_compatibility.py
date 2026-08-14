"""Production import and Raspberry Pi SoftPot activation smoke tests."""
import importlib
import pkgutil
from pathlib import Path

import yaml


ROOT = Path(__file__).parents[1]
PRODUCTION_PACKAGES = ("analysis", "calibration", "hardware", "storage", "web")


def production_module_names():
    """Find every importable production module, including future additions."""
    yield "app"
    for package_name in PRODUCTION_PACKAGES:
        package = importlib.import_module(package_name)
        yield package_name
        yield from (
            module.name
            for module in pkgutil.walk_packages(package.__path__, package_name + ".")
        )


def test_complete_production_module_graph_imports():
    imported = {name: importlib.import_module(name) for name in production_module_names()}

    required = {
        "app",
        "calibration.controller",
        "calibration.calibration_plan",
        "calibration.softpot_calibration",
        "analysis.model_selection",
        "analysis.curve_fit",
        "analysis.quality",
        "analysis.reference_flow",
        "analysis.uncertainty",
        "analysis.omron_reference",
        "hardware.ads1115",
        "hardware.flow_sensor",
        "hardware.softpot",
        "hardware.stepper",
        "storage.runs",
        "web.routes",
    }
    assert required <= imported.keys()


def test_softpot_save_activates_controller_with_fake_hardware(tmp_path, monkeypatch):
    app_module = importlib.import_module("app")
    routes = importlib.import_module("web.routes")

    class FakeADC:
        instance = None

        def __init__(self, config):
            self.config = config
            self.softpot_voltage = 0.25
            FakeADC.instance = self

        def voltage(self, channel):
            if channel == self.config["flow_channel"]:
                return 0.2
            return self.softpot_voltage

    class FakeStepper:
        def __init__(self, config):
            self.config = config

        def disable(self):
            pass

        def move(self, steps, frequency_hz, stop_event):
            pass

    class FastSession(routes.SoftPotCalibrationSession):
        def capture(self, volume_ml, duration_s=0.01, interval_s=0.001):
            return super().capture(volume_ml, duration_s, interval_s)

    config = yaml.safe_load((ROOT / "config.yaml").read_text())
    config["data_dir"] = str(tmp_path)
    config_path = tmp_path / "config.yaml"
    config_path.write_text(yaml.safe_dump(config))
    monkeypatch.setattr(app_module, "ADS1115", FakeADC)
    monkeypatch.setattr(app_module, "Stepper", FakeStepper)
    monkeypatch.setattr(routes, "SoftPotCalibrationSession", FastSession)

    flask_app = app_module.create_app(config_path)
    client = flask_app.test_client()
    # Voltage increases as volume decreases, producing a monotonic mapping.
    for volume, voltage in zip((100, 75, 50, 25, 0), (0.25, 1.0, 1.75, 2.5, 3.25)):
        FakeADC.instance.softpot_voltage = voltage
        response = client.post("/api/softpot/capture", json={"volume_ml": volume})
        assert response.status_code == 200, response.get_json()

    response = client.post("/api/softpot/save")

    assert response.status_code == 200, response.get_json()
    assert response.get_json()["message"] == "SoftPot calibration saved and active."
    saved = list((tmp_path / "softpot").glob("softpot_calibration_*.json"))
    assert len(saved) == 1
    assert client.get("/api/status").get_json()["state"] == "IDLE"
