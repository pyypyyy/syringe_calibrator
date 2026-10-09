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


def test_noncompleting_conversion_times_out_and_releases_adc(production_config, fake_ads_modules, monkeypatch):
    base = sys.modules['adafruit_ads1x15.ads1115'].ADS1115
    monkeypatch.setattr(base, '_conversion_complete', lambda self: False, raising=False)
    adc = ADS1115(production_config)
    class PendingChannel:
        @property
        def voltage(self):
            while not adc._ads._conversion_complete():
                pass
            return 1.0
    adc._channels[2] = PendingChannel()
    times = iter([0, .1, 1.1])
    monkeypatch.setattr('hardware.ads1115.time', SimpleNamespace(monotonic=lambda: next(times)))
    with pytest.raises(TimeoutError, match='conversion timed out'):
        adc.voltage(2)
    assert adc._ads._conversion_deadline is None
    assert adc._lock.acquire(blocking=False)
    adc._lock.release()


def test_concurrent_adc_reads_are_serialized(production_config, fake_ads_modules):
    import threading
    adc = ADS1115(production_config)
    entered, release, other = threading.Event(), threading.Event(), threading.Event()
    class SlowChannel:
        @property
        def voltage(self):
            entered.set()
            assert release.wait(2)
            return 1.0
    class OtherChannel:
        @property
        def voltage(self):
            other.set()
            return 2.0
    adc._channels = {1:SlowChannel(), 2:OtherChannel()}
    first = threading.Thread(target=adc.voltage, args=(1,))
    second = threading.Thread(target=adc.voltage, args=(2,))
    first.start()
    assert entered.wait(1)
    second.start()
    try:
        assert not other.wait(.05)
    finally:
        release.set()
        first.join(1); second.join(1)
    assert other.is_set()


@pytest.mark.parametrize('completes', [True, False])
def test_installed_driver_mux_and_conversion_polling(production_config, monkeypatch, completes):
    # Exercise the actual supported Adafruit driver, substituting only the I2C
    # register transport and host GPIO modules. Core-only installs may skip it.
    ads = pytest.importorskip('adafruit_ads1x15.ads1115')
    from adafruit_ads1x15.ads1x15 import Mode
    pins = []
    def init(self, *args, **kwargs):
        self._gain = kwargs['gain']; self._mode = Mode.SINGLE
        self._data_rate = 128
        self._last_pin_read = None
    monkeypatch.setattr(ads.ADS1115, '__init__', init)
    if hasattr(ads.ADS1115, '_write_config'):
        monkeypatch.setattr(ads.ADS1115, '_write_config', lambda self,pin:pins.append(pin))
    else:
        monkeypatch.setattr(ads.ADS1115, '_write_register',
                            lambda self,reg,value:pins.append((value >> 12) & 7))
    monkeypatch.setattr(ads.ADS1115, '_conversion_complete', lambda self:completes)
    monkeypatch.setattr(ads.ADS1115, 'get_last_result', lambda self,fast:10000)
    monkeypatch.setitem(sys.modules, 'board', SimpleNamespace(SCL=1,SDA=2))
    monkeypatch.setitem(sys.modules, 'busio', SimpleNamespace(I2C=lambda *args:None))
    adc = ADS1115(production_config)
    times = iter([0, .1, 1.1])
    monkeypatch.setattr('hardware.ads1115.time', SimpleNamespace(monotonic=lambda:next(times)))
    if completes:
        # Older releases use 32767 rather than 32768 as the scale divisor.
        assert adc.voltage(2) == pytest.approx(1.25, abs=4.096/32768)
    else:
        with pytest.raises(TimeoutError, match='conversion timed out'):
            adc.voltage(2)
    assert pins == [6]  # Single-ended A2 MUX setting, not differential 2.
