# Gasflow Calibrator V2

A focused Raspberry Pi laboratory instrument for calibrating an **Omron D6F-P0010A1** analog flow sensor against measured motion of a motor-driven 100 ml syringe. It stores every raw trial, estimates reference flow by regression of syringe volume against time, compares empirical models using leave-one-flow-level-out validation, and reports bootstrap uncertainty.

There is deliberately **no production mock mode**, environmental sensor, temperature/pressure/humidity correction, or silent hardware fallback. Tests alone use synthetic numerical data. If pigpio, the ADC, either ADC channel, or the SoftPot calibration is unavailable, the web UI remains available but calibration is disabled and the exact failure is shown.

## Hardware and wiring

| Function | Connection |
|---|---|
| Stepper STEP | GPIO18 |
| Stepper DIR | GPIO4 |
| Stepper ENABLE | GPIO21, active-low |
| ADS1115 | I2C address `0x48` |
| Omron analog output | ADS1115 A1 |
| Linear SoftPot wiper | ADS1115 A2 |

Use a common signal ground and hardware-appropriate power supplies. Do not power a motor from the Pi rail. The configured defaults assume 208 microsteps/ml and a 100 ml syringe; verify mechanics, direction, end clearances, emergency stop behavior, and SoftPot feedback at low speed before a run. The software is not a substitute for mechanical end stops or supervision.

## Raspberry Pi installation

Enable I2C with `raspi-config`, install and start pigpio, then install the application:

```bash
sudo apt install pigpio python3-pigpio python3-venv
sudo systemctl enable --now pigpiod
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements-pi.txt
python app.py
```

Open `http://<raspberry-pi-address>:5000/` from a device on the same network. The server binds to `0.0.0.0`, uses one process, and explicitly disables Flask debug/reloader operation. For permanent installation, run exactly one supervised process; never configure multiple WSGI workers because hardware has one owner.

## Workflow

1. **SoftPot calibration:** open **SoftPot**, position the syringe at 100, 75, 50, 25, and 0 ml, capture each voltage distribution, then validate/save. Points must be monotonic. Restart once so startup loads the newest valid mapping. Conversion is piecewise-linear and refuses readings outside its calibrated voltage range.
2. **Flow calibration:** select AIR or CO2, targets, and repeats on the dashboard. Zero voltage is captured automatically before and after. For each target the controller chooses a roughly 45-second bounded stroke, samples raw sensor/position values, and derives actual flow from the OLS slope `Q = |b| × 60 / 1000`; commanded speed is only a target.
3. **Progress and safety:** progress exposes the explicit state, live voltage and position. Stop is cooperative; Emergency Stop immediately cancels pulses and disables the driver. Exceptions also disable it in `finally`.
4. **Results:** completion redirects directly to the run result. The selected empirical curve, 95% bootstrap confidence band, measured repeats, cross-validated error graph, Omron comparison, model table, warnings, diagnostics, and exports are available there. History makes every completed run inspectable.

## Understanding the statistics

Linear, quadratic, cubic, and (with enough levels) empirical fifth-order polynomials are fitted. Cross-validation holds out **all repeats of one flow level together**, preventing leakage. Selection prefers a simpler monotonic model unless predictive RMSE meaningfully improves; fifth-order requires at least 15% improvement. Bootstrap intervals resample repeats within each level and exclude invalid/non-monotonic fits. These confidence bands describe fitted calibration uncertainty; repeatability and cross-validated prediction error remain separate quantities.

`analysis/omron_reference.py` contains the Omron D6F-P User's Manual (Cat. A299) representative fifth-order approximation, including the correct `U⁴` coefficient **-0.564312**. It is a fixed manufacturer comparison curve—not the measured calibration of this individual sensor, not a syringe reference, and not automatically selected or exported as the empirical calibration. This distinction is especially important for CO2.

## Data and configuration

`config.yaml` is the single source for pins, ADC channels, mechanics, targets, sampling, stroke constraints, safety thresholds, and bootstrap count. Human-readable artifacts live under `data/softpot/` and `data/runs/<run_id>/`: metadata/configuration, both zero captures, all raw trial CSVs (including rejected trials), trial summaries, and `analysis.json`. Preserve the whole directory for reproducibility.

Development checks:

```bash
pip install -r requirements.txt pytest
pytest
```
