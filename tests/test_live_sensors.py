from types import SimpleNamespace
from flask import Flask

from storage.runs import RunStore
from web.routes import create_blueprint


def test_idle_sensor_reads_are_live_and_fail_explicitly(tmp_path):
    class ADC:
        value = .5
        def voltage(self, channel):
            if self.value is None: raise RuntimeError('disconnected')
            return self.value + channel
    adc = ADC()
    app = Flask(__name__)
    app.register_blueprint(create_blueprint(RunStore(tmp_path), {'controller':None},
        {'errors':[]}, adc, {'ads1115':{'flow_channel':1,'softpot_channel':2}}))
    client = app.test_client()
    assert client.get('/api/sensors').json['flow_voltage_v'] == 1.5
    adc.value = .8
    assert client.get('/api/sensors').json['flow_voltage_v'] == 1.8
    adc.value = None
    assert client.get('/api/sensors').status_code == 503


def test_active_run_polling_does_not_read_adc(tmp_path):
    class ADC:
        def voltage(self, channel): raise AssertionError('extra ADC traffic')
    controller = SimpleNamespace(_worker=SimpleNamespace(is_alive=lambda:True),
        status=lambda:{'sensor_voltage_v':1.2,'position_ml':40})
    app = Flask(__name__)
    app.register_blueprint(create_blueprint(RunStore(tmp_path), {'controller':controller},
        {'errors':[]}, ADC(), {'ads1115':{'flow_channel':1,'softpot_channel':2}}))
    client = app.test_client()
    assert client.get('/api/sensors').json['flow_voltage_v'] == 1.2
    assert client.post('/api/softpot/capture',json={'volume_ml':50}).status_code == 409
