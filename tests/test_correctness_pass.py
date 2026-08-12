from pathlib import Path
import numpy as np
import pytest
import yaml

from analysis.quality import target_quality
from calibration.calibration_plan import select_stroke
from calibration.controller import CalibrationController
from hardware.softpot import SoftPotMapping
from storage.runs import RunStore

ROOT = Path(__file__).parents[1]

class Reader:
    def __init__(self, low, high):
        self.min_calibrated_volume_ml=low; self.max_calibrated_volume_ml=high
    def __call__(self): return 1.0, (self.min_calibrated_volume_ml+self.max_calibrated_volume_ml)/2
class Sensor:
    def voltage(self): return 1.0
class Stepper:
    def disable(self): pass
    def move(self, *args): pass

def controller(tmp_path, low=0, high=100):
    cfg=yaml.safe_load((ROOT/'config.yaml').read_text())
    return CalibrationController(Sensor(),Reader(low,high),Stepper(),RunStore(tmp_path),cfg)

@pytest.mark.parametrize('low,high',[(0,100),(10,90),(25,75)])
def test_plans_respect_calibrated_softpot_range(tmp_path,low,high):
    c=controller(tmp_path,low,high)
    for target in c.config['calibration']['target_flows_lpm']:
        plan=c.create_trial_plan(target)
        assert low <= plan.end_volume_ml < plan.start_volume_ml <= high

def test_hot_loaded_reader_changes_planner_and_nonoverlap_fails(tmp_path):
    c=controller(tmp_path)
    c.install_position_reader(Reader(25,75))
    assert c._position_bounds()==(25,75)
    c.install_position_reader(Reader(200,300))
    with pytest.raises(Exception,match='do not overlap'): c.create_trial_plan(.1)

def test_mapping_exposes_measured_volume_coverage():
    m=SoftPotMapping([{'mean_voltage_v':0,'volume_ml':75},{'mean_voltage_v':1,'volume_ml':50},{'mean_voltage_v':2,'volume_ml':25}])
    assert (m.min_calibrated_volume_ml,m.max_calibrated_volume_ml)==(25,75)

@pytest.mark.parametrize('count,quality',[(3,'GOOD'),(2,'GOOD'),(1,'WEAK'),(0,'UNUSABLE')])
def test_target_quality_classes(count,quality): assert target_quality(count)==quality

def ideal_samples(plan, interval=.05):
    times=np.arange(0,plan.expected_duration_s+interval/2,interval)
    return [{'elapsed_s':float(t),'filtered_volume_ml':plan.start_volume_ml+plan.direction*plan.target_flow_lpm*1000/60*t,
             'flow_sensor_voltage_v':.3+plan.target_flow_lpm,'softpot_voltage_v':1,'motion_phase':'MEASURING','analysis_sample':False} for t in times]

def test_every_default_flow_passes_physical_duration_and_sample_qc(tmp_path):
    c=controller(tmp_path)
    for target in c.config['calibration']['target_flows_lpm']:
        plan=c.create_trial_plan(target); summary=c.analyze_trial(plan,target,1,ideal_samples(plan))
        assert summary['accepted'], (target,summary['rejection_reasons'])
        assert summary['minimum_acceptable_duration_s'] <= summary['planned_stable_duration_s']
        assert summary['minimum_acceptable_sample_count'] <= summary['planned_stable_sample_count']
    high=c.analyze_trial(c.create_trial_plan(1),1,1,ideal_samples(c.create_trial_plan(1)))
    assert high['minimum_acceptable_sample_count'] < 100

def test_trial_ending_well_before_plan_is_rejected(tmp_path):
    c=controller(tmp_path); plan=c.create_trial_plan(1)
    samples=ideal_samples(plan)[:12]
    summary=c.analyze_trial(plan,1,1,samples)
    assert not summary['accepted']
    assert {'sample count below minimum','measurement duration below minimum'} <= set(summary['rejection_reasons'])

def test_failed_analysis_has_failed_controller_and_durable_artifacts(tmp_path):
    c=controller(tmp_path); run_id=c.store.create('AIR',{})
    zero={'mean_voltage_v':0.1,'standard_deviation_v':0.0,'sample_count':100,'duration_s':5}
    c.finalize_run(run_id,'AIR',[],[],zero,zero)
    assert c.status()['state']=='FAILED'
    assert c.store.read_json(run_id,'run.json')['completion_state']=='FAILED'
    assert c.store.read_json(run_id,'analysis.json')['status']=='FAILED'
    assert (c.store.path(run_id)/'calibration_points.csv').exists()
