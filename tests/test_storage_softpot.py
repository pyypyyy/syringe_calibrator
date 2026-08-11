import json
import pytest
from hardware.softpot import PositionOutOfRange,SoftPotMapping
from storage.runs import RunStore

def test_softpot_interpolation_is_bounded():
    mapping=SoftPotMapping([{"mean_voltage_v":0,"volume_ml":0},{"mean_voltage_v":1,"volume_ml":50},{"mean_voltage_v":2,"volume_ml":100}])
    assert mapping.volume(.5)==25
    with pytest.raises(PositionOutOfRange):mapping.volume(2.1)

def test_run_roundtrip(tmp_path):
    store=RunStore(tmp_path); run_id=store.create("AIR",{"targets_lpm":[.1],"repeats":3}); store.write_json(run_id,"analysis.json",{"selected":"linear"})
    assert store.read_json(run_id,"analysis.json")=={"selected":"linear"}; assert store.history()[0]["run_id"]==run_id
