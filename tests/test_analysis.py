import numpy as np
import pytest
from analysis.reference_flow import estimate_reference_flow
from analysis.omron_reference import OMRON_COEFFICIENTS,omron_reference
from analysis.curve_fit import fit_model,monotonic_over,predict_checked
from analysis.model_selection import ModelScore,compare_models,select_empirical_model
from analysis.quality import robust_repeat_outliers
from analysis.uncertainty import bootstrap_band

def test_reference_flow_exact_and_noisy():
    t=np.linspace(0,10,201); result=estimate_reference_flow(t,100-5*t)
    assert result.flow_lpm==pytest.approx(.3); assert result.r_squared==pytest.approx(1); assert result.sample_count==201
    noisy=estimate_reference_flow(t,100-5*t+np.random.default_rng(2).normal(0,.05,len(t)))
    assert noisy.flow_lpm==pytest.approx(.3,rel=.002); assert noisy.slope_standard_error>0

def test_omron_reference_values_and_typo_regression():
    assert OMRON_COEFFICIENTS[1] == -0.564312
    expected={0.0:-.269996,0.5:-.00053503125,1.0:.093562,2.0:.422082,3.0:2.749012}
    for voltage,flow in expected.items(): assert omron_reference(voltage)==pytest.approx(flow,abs=1e-9)

@pytest.mark.parametrize("name,coeff",[("linear",[.4,.1]),("quadratic",[.1,.2,.05]),("cubic",[.01,.03,.2,.04]),("polynomial_5",[.001,-.002,.003,.02,.2,.03])])
def test_empirical_fits(name,coeff):
    x=np.linspace(.5,3,20); model=fit_model(name,x,np.polyval(coeff,x)); assert model.coefficients==pytest.approx(coeff,rel=1e-8,abs=1e-8)

def make_repeats(fn=lambda u:.4*u-.1):
    return [{"target_flow_lpm":level,"sensor_voltage_v":u+d,"reference_flow_lpm":fn(u+d)} for level,u in enumerate(np.linspace(.5,3,7)) for d in (-.01,0,.01)]

def test_flow_level_cv_and_simple_selection():
    rows=make_repeats(); scores=compare_models(rows); linear=next(s for s in scores if s.name=="linear")
    assert len(linear.errors)==len(rows)
    assert {e["target_flow_lpm"] for e in linear.errors}=={r["target_flow_lpm"] for r in rows}
    assert select_empirical_model(scores).name=="linear"

def test_nonlinear_can_select_complex_model():
    scores=compare_models(make_repeats(lambda u:.03*u**3+.02*u**2+.1*u))
    assert select_empirical_model(scores).name in {"quadratic","cubic"}

def score(name, parameters, rmse):
    return ModelScore(name, [], parameters, rmse, rmse, rmse, True, [], [])

def test_model_selection_ignores_numerical_noise_but_accepts_real_improvement():
    tiny = [score("linear", 2, 1e-8), score("quadratic", 3, .5e-8)]
    assert select_empirical_model(tiny).name == "linear"
    real = [score("linear", 2, .02), score("quadratic", 3, .01)]
    assert select_empirical_model(real).name == "quadratic"

def test_fifth_order_keeps_stronger_complexity_protection():
    small = [score("linear", 2, .02), score("polynomial_5", 6, .018)]
    assert select_empirical_model(small).name == "linear"
    large = [score("linear", 2, .02), score("polynomial_5", 6, .01)]
    assert select_empirical_model(large).name == "polynomial_5"

def test_monotonicity_range_and_outlier():
    assert not monotonic_over([-1,2,0],(0,2))
    model=fit_model("linear",[1,2,3],[0,.5,1])
    with pytest.raises(ValueError):predict_checked(model,.5,(1,3))
    assert robust_repeat_outliers([1,1.01,.99,5]).tolist()==[False,False,False,True]

def test_zero_mad_uses_physical_tolerance():
    assert not robust_repeat_outliers([1, 1, 1.000001]).any()
    assert robust_repeat_outliers([1, 1, 1.3]).tolist() == [False, False, True]
    assert robust_repeat_outliers([1, 1.01, .99, 1.5]).tolist() == [False, False, False, True]

def test_bootstrap_band():
    band=bootstrap_band(make_repeats(),"linear",iterations=100,grid_size=20,seed=3)
    assert band["accepted_iterations"]==100; assert len(band["lower_lpm"])==20
