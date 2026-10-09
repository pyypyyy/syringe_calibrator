from types import SimpleNamespace
from flask import Flask
import pytest

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
    assert client.post('/api/softpot/save').status_code == 409


def test_competing_start_returns_json_conflict(tmp_path):
    def start(*args):
        raise RuntimeError('calibration already running')
    controller = SimpleNamespace(start=start)
    app = Flask(__name__)
    app.register_blueprint(create_blueprint(RunStore(tmp_path), controller,
        {'errors': []}, None, {}))
    response = app.test_client().post('/api/calibration/start',
        json={'gas':'AIR', 'targets_lpm':[.1], 'repeats':1})
    assert response.status_code == 409
    assert response.json['error'] == 'calibration already running'


def test_sensor_request_finishes_before_start_can_claim_adc(tmp_path):
    import threading
    reading, release, started = threading.Event(), threading.Event(), threading.Event()
    class ADC:
        def voltage(self, channel):
            reading.set()
            assert release.wait(2)
            assert not started.is_set()
            return 1.0
    class Controller:
        _worker = None
        def status(self): return {'state':'IDLE'}
        def start(self, *args): started.set()
        def stop(self, *args): pass
    app = Flask(__name__)
    app.register_blueprint(create_blueprint(RunStore(tmp_path), Controller(),
        {'errors':[]}, ADC(), {'ads1115':{'flow_channel':1,'softpot_channel':2}}))
    responses = []
    def poll():
        with app.test_client() as client:
            responses.append(client.get('/api/sensors').status_code)
    def start():
        with app.test_client() as client:
            responses.append(client.post('/api/calibration/start',
                json={'gas':'AIR','targets_lpm':[.1],'repeats':1}).status_code)
    reader = threading.Thread(target=poll); reader.start()
    assert reading.wait(1)
    starter = threading.Thread(target=start); starter.start()
    try:
        assert not started.wait(.05)
        # Emergency Stop must not wait for the idle-operation lock.
        assert app.test_client().post('/api/calibration/stop',json={'emergency':True}).status_code == 200
    finally:
        release.set(); reader.join(1); starter.join(1)
    assert sorted(responses) == [200,202]
    assert started.is_set()


def test_save_during_motion_preserves_existing_mapping_file(tmp_path, monkeypatch):
    import web.routes as routes
    class Session:
        points = []
        def __init__(self, read): pass
        def capture(self, volume): return {'volume_ml':volume}
        def save(self, path):
            raise AssertionError('must not persist or activate during motion')
    monkeypatch.setattr(routes, 'SoftPotCalibrationSession', Session)
    worker = SimpleNamespace(is_alive=lambda:False)
    active = SimpleNamespace(_worker=worker)
    app = Flask(__name__)
    app.register_blueprint(create_blueprint(RunStore(tmp_path), active,
        {'errors':[]}, SimpleNamespace(voltage=lambda channel:1),
        {'ads1115':{'flow_channel':1,'softpot_channel':2}},
        lambda data:pytest.fail('must not activate during motion')))
    client = app.test_client()
    assert client.post('/api/softpot/capture',json={'volume_ml':50}).status_code == 200
    previous = tmp_path/'softpot'/'previous.json'
    previous.write_text('existing calibration')
    worker.is_alive = lambda:True
    assert client.post('/api/softpot/save').status_code == 409
    assert previous.read_text() == 'existing calibration'
    assert list(previous.parent.iterdir()) == [previous]
