import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest
import yaml

from hardware.ads1115 import ADS1115

ROOT = Path(__file__).parents[1]


@pytest.fixture
def fake_ads_modules(monkeypatch):
    analog_inputs = []

    class FakeADS1115:
        def __init__(self, i2c, address, gain):
            self.i2c = i2c
            self.address = address
            self.gain = gain

    class FakeAnalogIn:
        def __init__(self, adc, channel):
            analog_inputs.append((adc, channel))
            self.voltage = {1: 1.25, 2: 2.5}[channel]

    board = ModuleType("board")
    board.SCL = object()
    board.SDA = object()
    busio = ModuleType("busio")
    busio.I2C = lambda scl, sda: (scl, sda)
    package = ModuleType("adafruit_ads1x15")
    package.__path__ = []
    ads = ModuleType("adafruit_ads1x15.ads1115")
    ads.ADS1115 = FakeADS1115
    analog_in = ModuleType("adafruit_ads1x15.analog_in")
    analog_in.AnalogIn = FakeAnalogIn

    for name, module in {
        "board": board,
        "busio": busio,
        "adafruit_ads1x15": package,
        "adafruit_ads1x15.ads1115": ads,
        "adafruit_ads1x15.analog_in": analog_in,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)

    return SimpleNamespace(analog_inputs=analog_inputs)


@pytest.fixture
def production_config():
    with (ROOT / "config.yaml").open() as config_file:
        return yaml.safe_load(config_file)["ads1115"]


def test_flow_channel_is_a1(production_config, fake_ads_modules):
    adc = ADS1115(production_config)

    assert adc.voltage(production_config["flow_channel"]) == 1.25


def test_softpot_channel_is_a2(production_config, fake_ads_modules):
    adc = ADS1115(production_config)

    assert adc.voltage(production_config["softpot_channel"]) == 2.5
    assert (adc._ads.address, adc._ads.gain) == (0x48, 1)


def test_integer_channels_are_passed_to_analog_in(production_config, fake_ads_modules):
    adc = ADS1115(production_config)

    assert fake_ads_modules.analog_inputs == [(adc._ads, 1), (adc._ads, 2)]
    assert all(isinstance(channel, int) for _, channel in fake_ads_modules.analog_inputs)


@pytest.mark.parametrize("name", ["flow_channel", "softpot_channel"])
@pytest.mark.parametrize("channel", [-1, 4, 1.5, "1", True])
def test_invalid_channel_configuration_is_rejected(name, channel, production_config):
    production_config[name] = channel

    with pytest.raises(ValueError, match=rf"ADS1115 {name} must be an integer from 0 through 3"):
        ADS1115(production_config)
