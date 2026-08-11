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

1. **SoftPot calibration:** open **SoftPot**, position the syringe at 100, 75, 50, 25, and 0 ml, capture each voltage distribution, then validate/save. Saving rejects non-monotonic, collapsed, duplicate, or excessively noisy point sets and activates a valid mapping in the running controller. Conversion is piecewise-linear and refuses readings outside its calibrated voltage range.
2. **Flow calibration:** select AIR or CO2, targets, and repeats on the dashboard. Zero voltage is captured and stability-checked before and after. Each planned stroke contains safe start and end positions. Every repeat uses chunked, closed-loop SoftPot positioning, settles, measures toward its end position, and returns to the defined start; accumulated step counts are never treated as position truth.
3. **Progress and safety:** progress exposes `POSITIONING`, `SETTLING`, `MEASURING`, and `RETURNING` as real hardware states. Position range, persistent wrong direction, and lack of meaningful motion are watched during every move. Stop produces an inspectable `ABORTED` run; Emergency Stop first cancels pulses and disables the driver. Safety failures produce `FAILED` and cannot enter normal model fitting.
4. **Measurement integrity:** complete motion samples remain in each raw CSV with `motion_phase` and `analysis_sample` fields. The initial and final transient fractions are excluded from both voltage statistics and the signed position-versus-time OLS regression. Trial QC precedes median/MAD repeat rejection; targets with fewer than two accepted repeats are excluded from fitting.
5. **Results:** artifacts are persisted before `COMPLETE` becomes observable. The result includes explicit status reasons, equation and ranges, target-level points, accepted/rejected repeats, bootstrap band, CV and Omron errors, repeatability, an error table, structured diagnostics, and exports. History combines run metadata with completed analysis while tolerating failed or incomplete runs.

## Understanding the statistics

Linear, quadratic, cubic, and (with enough levels) empirical fifth-order polynomials are fitted to one unweighted aggregate calibration point per usable target. Cross-validation holds out **an entire flow level**, preventing leakage. Selection rejects non-monotonic or physically implausible in-range curves and prefers a simpler model unless predictive RMSE meaningfully improves; fifth-order requires at least 15% improvement. Bootstrap intervals resample repeats within each level, recreate target means, and report invalid/non-monotonic fit rejection. These confidence bands describe fitted calibration uncertainty; repeatability and cross-validated prediction error remain separate quantities.

`analysis/omron_reference.py` contains the Omron D6F-P User's Manual (Cat. A299) representative fifth-order approximation, including the correct `U⁴` coefficient **-0.564312**. It is a fixed manufacturer comparison curve—not the measured calibration of this individual sensor, not a syringe reference, and not automatically selected or exported as the empirical calibration. This distinction is especially important for CO2.

## Data and configuration

`config.yaml` is the single source for pins, ADC channels, mechanics, targets, sampling, stroke constraints, safety thresholds, and bootstrap count. Human-readable artifacts live under `data/softpot/` and `data/runs/<run_id>/`: metadata/configuration, both zero captures, all raw trial CSVs (including rejected trials), trial summaries, and `analysis.json`. Preserve the whole directory for reproducibility.

Development checks:

```bash
pip install -r requirements.txt pytest
pytest
```
