# Gasflow Calibrator V2

The minimum supported Python version is **3.9**. The Raspberry Pi currently
used for the production instrument runs Python **3.9.2**; CI exercises both
Python 3.9 and a newer Python release.

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
2. **Flow calibration:** select AIR or CO2, targets, and repeats on the dashboard. Zero voltage is captured and stability-checked before and after. Each planned stroke contains safe start and end positions. Every repeat uses chunked, closed-loop SoftPot preloading, approaches its start in the measurement direction, settles, measures toward its end position, and returns to the next preload point; accumulated step counts are never treated as position truth.
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

## Current TB6600 wiring and commissioning

The current configuration is for a Raspberry Pi 3B and **direct 3.3 V,
common-cathode** TB6600 inputs. The supplied driver manual accepts 3.3 V
control; the measured PUL input current is approximately 10 mA. Verify DIR
and ENA input currents too. This is not a universal wiring recommendation
for every device sold as TB6600.

| TB6600 terminal | Pi connection |
|---|---|
| PUL+ | GPIO18, physical pin 12 |
| DIR+ | GPIO4, physical pin 7 |
| ENA+ | GPIO21, physical pin 40 |
| PUL−, DIR−, ENA− | Pi GND, e.g. physical pin 6 |
| VCC / power GND | External motor supply positive / negative |

Do not connect the control positives to 5 V in this arrangement. The
optoisolated control return does not require a connection to motor-supply
negative. GPIO4 must not simultaneously be assigned to a 1-Wire overlay.

The separate ADS1115 remains at `0x48`, Omron output on **A1**, SoftPot wiper
on **A2**. The custom Omron PCB is currently bypassed. Pi I2C uses physical
pins 3 (SDA/GPIO2) and 5 (SCL/GPIO3).

`stepper.drive_strength_ma: 12` is applied and read back at application
startup through pigpio. This is a **GPIO bank 0 (GPIO0–27)** setting, not a
per-pin current limiter. It does not change the 3.3 V signal voltage or force
12 mA through a load. Configuration failures disable calibration. Do not run
other motor-control programs alongside this application's pigpio waves.

`enable_active_low: true` is retained: GPIO21 LOW is the requested motor-enabled
state, HIGH is disabled. The optocoupler wiring diagram alone does not establish
whether the driver's ENA input releases or energizes the motor. Verify that
startup/Stop actually releases the motor, and change this setting only if the
physical driver behaves oppositely. Likewise verify `invert_direction` and the
DIP-switch microstep setting against `microsteps_per_ml: 208.0`; that numerical
value is a mechanical calibration, not a universal TB6600 setting.

Start with motor power disconnected. Launch the app and check the live flow
and SoftPot voltages on the dashboard. Calibrate the SoftPot at known syringe
volumes; use only reachable points and never force a mechanical endpoint.
The current UI has no manual jog control. Verify direction, motor enable,
travel clearances and stopping under supervision before a complete calibration.
A single-target run can test motion, but cannot produce a valid fitted model:
model fitting requires at least three usable levels and two accepted repeats
per level. High-flow runs are short with a 100 ml syringe and need physical
validation for pneumatic transients and missed steps.

Stationary positioning and endpoint checks now acquire five fresh ADC samples
and reset the streaming filter. In-motion readings keep the median filter.
Dashboard polling reads the ADC only while idle; during a run it displays the
controller's latest samples, avoiding additional measurement-loop I2C traffic.
Automated tests use simulated hardware; they do not certify physical wiring,
ENA polarity, gas tightness or calibration accuracy.

## Calibration progress interface (Stage 1)

The calibration page keeps the current operation separate from the last completed
trial. It polls only `/api/status`, every 500 ms after the previous request
finishes. No browser request adds ADC samples. Stop and Emergency Stop keep their
existing motor shutdown path. Results remain available through an explicit link,
so completion no longer immediately hides the progress page.

Overall progress is a fraction from 0 to 1. Every milestone has equal weight:
initial zero capture saved and validated, each trial's measurement **and successful
return**, final zero capture saved, and successful analysis with persisted results.
For N trials the denominator is N + 3. A QC-rejected trial still completes its
operational milestone after returning. Failed analysis does not complete the
last milestone. Failed and aborted runs retain completed milestones; progress is
not an elapsed-time estimate. The accepted/rejected counts can change when the
existing target-level repeat outlier checks run.

The existing status fields remain available. Additive `/api/status` fields:

| Field | Meaning |
|---|---|
| `remaining` | Trials that have not completed measurement and return |
| `target_index`, `target_count`, `repeats_total` | One-based target context and requested repeat count; index/repeat 0 means none yet |
| `zero_before_completed`, `zero_after_completed`, `analysis_completed` | Completed overall milestones |
| `operation_description` | Human-readable current activity |
| `operation_progress` | Fraction 0–1, or null when progress is indeterminate |
| `operation_elapsed_s` | Monotonic elapsed time in the operation, or current zero subphase; frozen on stop/termination |
| `operation_duration_s`, `operation_remaining_s` | Configured settling/zero phase duration and remaining time; null for motion/analysis |
| `operation_start_position_ml` | Measured starting position used for motion progress |
| `operation_target_position_ml` | Commanded piston destination, never a measured displacement |
| `operation_direction` | Increasing/decreasing volume, stationary, or null |
| `operation_phase` | `settling` or `recording` for zero capture; `planning`, `pre_positioning`, or `directional_approach` for positioning; otherwise null |
| `failure_operation` | Operation interrupted by stop or failure; null when none |
| `last_completed_trial` | Snapshot of the latest trial that also returned successfully, including existing QC results |

Motion progress uses measured displacement from the operation start toward its
commanded endpoint, clamped to 0–1. A display-only high-water mark prevents small
SoftPot fluctuations from making the bar oscillate. Measured position remains
unchanged, and all safety checks continue using the existing feedback. Bars reset
for each operation; zero capture separately resets between settling and recording.
Analysis is indeterminate. Terminal states stop progress animation. Missing values
appear as a dash, and a lost connection marks retained measurements as stale.

`last_completed_trial.measured_stroke_ml` is an additive saved trial-summary field:
the absolute difference between the existing fresh measured start and endpoint.
The original `stroke_ml`, `stroke_start_ml`, and `stroke_end_ml` retain their
existing meanings. Initial trial acceptance may be revised by repeat outlier QC.
No fitting, uncertainty, sampling, filtering, waveform, or mechanical settings
are changed by this interface.

To verify on the Raspberry Pi under the normal commissioning precautions:

1. Start one application process and begin a calibration with valid hardware and
   SoftPot mapping. Check that the initial zero distinguishes settling from recording.
2. Watch Positioning → Settling → Measuring → Returning on each repeat. Compare
   the measured position with the commanded target; check requested flow, repeat,
   latest Omron voltage, and elapsed operation time.
3. Confirm completed trials increase only after return and that the previous
   trial summary remains visible during the next trial. Overall progress must stay
   below 100% during final zero and analysis; inspect the result link on completion.
4. On separate supervised runs, verify Stop and Emergency Stop show stopping and
   then a terminal outcome without an active animation. Confirm failure details
   identify the interrupted operation, repeat, target flow, and last position.
5. Disconnect the browser's network briefly to verify the stale-data notice and
   automatic reconnection; this does not itself stop the instrument.

Automated tests use test-only hardware. Browser checks with synthetic API responses
validate presentation, not piston motion, motor shutdown latency, physical ADC
performance, wiring, or calibration accuracy. Those require the actual instrument.

## Directional preload before every measurement

Each trial now first moves to a preload point, then approaches its measurement
start **only in the measurement direction**. For a 0.05 L/min stroke, the nominal
sequence is 71.750 ml → 68.750 ml → settle/verify → measure toward 31.250 ml.
This applies to the first repeat, every subsequent repeat, and changes of flow
level. A return goes directly to the next trial's preload point (or the current
one after the final trial); it remains part of the preceding trial's operational
completion milestone. No preload motion counts as a completed trial or enters
the recorded flow/position regression samples.

New `calibration` settings, also used as defaults for older configuration files:

| Setting | Default | Meaning |
|---|---|---|
| `directional_preload_ml` | 3.0 | Distance opposite the measurement direction from its planned start. Provisional; must be validated experimentally. Must exceed twice `position_tolerance_ml`. |
| `directional_preload_attempts` | 2 | Total approach attempts including the first; integer 1–3. |
| `preload_clearance_ml` | 2.0 | Minimum clearance inside the intersection of mechanical and calibrated SoftPot limits. Positive and fixed, never silently reduced to fit a stroke. |

The original stroke is rejected if its endpoints or preload cannot fit with
clearance **and the existing position tolerance**. The planner does not silently
shorten the stroke or change its target duration to create space. Consequently,
a limited SoftPot calibration range may now reject a previously feasible stroke.
Neither `microsteps_per_ml: 208.0` nor endpoint tolerances have been increased.

The final approach uses only commands with the measurement's direction. It aims
inside a quarter of the existing acceptance tolerance to reserve room for sensor
noise. A crossing within the unchanged full tolerance is accepted without a
reverse correction. An out-of-tolerance overshoot disables the driver, returns
to preload, and retries within the configured attempt cap. Other feedback faults,
settling drift beyond tolerance, and cancellation fail immediately. Both the
settled target error and displacement since the approach must pass the existing
position tolerance. There is no correction after settling before measurement.

Each positioning leg has a commanded-step cap equal to the ceiling of
`(initial distance to target + 2 × chunk size + 2 × position tolerance) × microsteps/ml`.
Preparing a trial also shares an aggregate cap across every preload/recovery leg:
`(initial distance to preload + (2A − 1) × preload distance + 2A × (chunk size + position tolerance)) × microsteps/ml`,
rounded up to whole steps, with A the maximum number of attempts. These bounds
count the sum of absolute commanded steps, not just net displacement. Chunks
remain limited by the configured chunk size and pigpio's existing per-command
limit. Existing direction and stall watchdogs remain active; cancellation is
checked before each chunk. The driver is disabled on success or any error. These
caps are software command limits, not independent measurements of physical travel.

Stage 1 status now identifies initial `operation_phase: planning` validation, then distinguishes
`pre_positioning` from
`directional_approach` within POSITIONING. Added fields are
`preload_position_ml`, `measurement_start_position_ml`, `positioning_attempt`, and
`positioning_attempts_total`. Each phase resets its operation bar and timer, while
overall progress retains the same N + 3 milestones. The browser keeps its 500 ms
status polling and adds no ADC reads.

Per-trial diagnostics are saved at
`data/runs/<run_id>/trials/flow_<target>_repeat_<n>_positioning.json`, including
failed or cancelled preparation when storage remains writable. Geometry rejection
before movement also writes a smaller `phase: planning` failure record. The record includes
configured distance, requested and reached preload, pre/post-settling positions,
settling displacement, approach attempts, warnings/errors, last measured position,
and aggregate commanded-step limits/usage. Successful trial summaries also contain
an additive `positioning` object with these fields. Successful records are written
only after measurement, so file I/O does not delay the transition from a freshly
verified settled position to the measurement stroke. Failed measurements retain
these preparation records with an additional `trial_error`; failed preparation
records are saved with the driver disabled. Existing raw CSV columns,
reference flow regression, 10% transient exclusions, QC, repeat rejection, model
selection, and bootstrap calculations are unchanged. The records do not diagnose
whether an endpoint error comes from contact hysteresis, mechanical scaling, or
SoftPot calibration, and preloading is not a demonstrated fix for the observed
0.75–1.0 ml endpoint errors.

### Raspberry Pi update and supervised commissioning

1. Stop the application through its normal supervisor and verify that motion has
   stopped and the driver is disabled. **Disconnect motor power first.** Preserve
   `data/`, the current SoftPot calibration, and any local configuration changes
   before updating the checkout. Do not run two application/motor-control processes.
2. Run `git fetch origin`, then check out `feat/directional-preloading` for PR
   review, or use `git switch main` and `git pull --ff-only origin main` after merge. Reuse the existing virtual environment and dependencies; no new
   packages are required. Compare the new settings with the instrument's existing
   `config.yaml`; retain 208.0 microsteps/ml, GPIO assignments, and TB6600 DIP settings.
3. This Pi requires pigpio's PCM clock: run the **single** daemon with
   `pigpiod -t 0`. If systemd manages it, ensure its existing startup command includes
   `-t 0`; do not launch a second daemon. Check the running daemon's command line
   before starting `python app.py` from the activated virtual environment.
4. With motor power still disconnected, verify UI readiness, real ADC readings,
   mapping coverage, preload targets, and the physical travel available for every
   intended stroke. Confirm independent hardware end-stop/travel protection and an
   effective hardware emergency power cutoff before reconnecting motor power.
5. Reconnect motor power only for attended commissioning with conservative speeds
   and adequate clearance. Check direction and stopping. On separate short,
   supervised runs, check Stop and Emergency Stop during pre-positioning and during
   directional approach. Verify the driver is disabled after both cancellation and
   any positioning failure.
6. Observe at least two repeats at the same flow and a transition to another flow:
   preload → decreasing-volume approach → settle → decreasing-volume measurement.
   Inspect positioning JSON records for attempt count and drift. Verify no final
   reverse correction and no trial count increase until the return finishes.
7. Evaluate the provisional 3.0 ml preload on the real mechanism. Investigate any
   remaining endpoint error independently; do not increase endpoint tolerances or
   auto-adjust microsteps/ml to conceal it. A single-target commissioning run can
   test sequence but cannot produce a valid calibration model.

SoftPot-derived software limits share the same contact hysteresis and calibration
errors they monitor. Commanded-step caps cannot protect independently against
feedback errors, a host interruption, or a driver fault. A pigpio daemon can
outlive the application. Independent hardware travel limits and a hardware power
cutoff remain necessary; this software is **not safe for unattended operation**.
Simulated tests do not establish physical safety, stopping distance, contact
preload adequacy, waveform timing on the Pi, or calibration accuracy.
