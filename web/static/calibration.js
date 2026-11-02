/* Status snapshots only: no additional sensor endpoint or hardware polling. */
const terminalStates = new Set(['COMPLETE', 'FAILED', 'ABORTED', 'HARDWARE_ERROR', 'IDLE']);
const stages = ['POSITIONING', 'SETTLING', 'MEASURING', 'RETURNING'];
const stageNames = {
  POSITIONING: 'Moving piston to starting position', SETTLING: 'Allowing the system to settle',
  MEASURING: 'Measuring flow', RETURNING: 'Returning piston', ZERO_BEFORE: 'Initial zero measurement',
  ZERO_AFTER: 'Final zero measurement', ANALYZING: 'Calculating calibration model',
};
const byId = id => document.getElementById(id);
const text = (id, value) => { byId(id).textContent = value; };
const number = (value, unit, digits = 3) =>
  typeof value === 'number' && Number.isFinite(value) ? `${value.toFixed(digits)} ${unit}` : '—';
let latest = null;
let stopPending = false;
let stopError = "";

function bar(id, value, active) {
  const element = byId(id);
  element.classList.toggle('inactive', !active);
  if (typeof value === 'number' && Number.isFinite(value)) element.value = Math.min(1, Math.max(0, value));
  else if (active) element.removeAttribute('value');
  else element.value = 0;
}

function render(s) {
  latest = s;
  const terminal = terminalStates.has(s.state);
  const stopping = s.state === 'STOPPING' || stopPending;
  const active = !terminal && !stopping;
  // Rounding must not advertise completion before the final milestone.
  const overallPercent = typeof s.progress === 'number' ?
    (s.progress >= 1 ? 100 : Math.min(99.9, s.progress * 100)) : null;
  text('overall-summary', `${s.completed ?? 0} of ${s.total ?? 0} trials completed · ${s.remaining ?? '—'} remaining · ${number(overallPercent, '%', 1)}`);
  bar('progress', s.progress ?? 0, false);
  text('trial-context', `Target ${s.target_index || '—'} of ${s.target_count || '—'} · Requested ${number(s.current_target_lpm, 'L/min')} · Repeat ${s.repeat || '—'} of ${s.repeats_total || '—'}`);
  text('operation-description', stopping ? 'Stop requested — stopping motor and terminating calibration.' :
    s.operation_description || (s.state === 'HARDWARE_ERROR' ? 'Hardware is not ready' : stageNames[s.state] || s.state));
  text('operation-phase', s.operation_phase === 'planning' ? 'Checking all planned strokes before issuing motor commands.' :
    s.operation_phase === 'pre_positioning' ? 'Pre-positioning only — no trial data is being recorded.' :
    s.operation_phase === 'directional_approach' ? 'Approaching in the measurement direction — no trial data is being recorded.' :
    s.operation_phase === 'settling' ? 'Zero measurement: waiting for the stopped system to settle.' :
    s.operation_phase === 'recording' && active ? 'Recording zero-flow sensor voltage.' :
    s.state === 'MEASURING' && active ? 'Sensor measurements are actively being recorded.' : '');
  if (!active && s.operation_phase) text('operation-phase', `Last phase: ${s.operation_phase.replaceAll('_', ' ')}`);
  bar('operation-progress', s.operation_progress, active);
  text('operation-percent', typeof s.operation_progress === 'number' ? number(s.operation_progress * 100, '%', 1) : active ? 'In progress' : '—');
  byId('operation-panel').className = terminal ? `outcome-${s.state.toLowerCase()}` : '';
  const activeIndex = stages.indexOf(s.state);
  document.querySelectorAll('[data-stage]').forEach((element, index) => {
    const done = activeIndex >= 0 && index < activeIndex || ['ZERO_AFTER', 'ANALYZING', 'COMPLETE'].includes(s.state);
    element.className = active && index === activeIndex ? 'active' : done ? 'done' : '';
    if (active && index === activeIndex) element.setAttribute('aria-current', 'step');
    else element.removeAttribute('aria-current');
  });
  text('position', number(s.position_ml, 'ml'));
  text('voltage', number(s.sensor_voltage_v, 'V'));
  text('flow', number(s.current_target_lpm, 'L/min'));
  text('start-position', number(s.operation_start_position_ml, 'ml'));
  text('target-position', number(s.operation_target_position_ml, 'ml'));
  text('preload-position', number(s.preload_position_ml, 'ml'));
  text('measurement-start-position', number(s.measurement_start_position_ml, 'ml'));
  text('positioning-attempt', s.positioning_attempt ? `${s.positioning_attempt} of ${s.positioning_attempts_total || '—'}` : '—');
  text('direction', s.operation_direction || '—');
  text('elapsed', number(s.operation_elapsed_s, 's', 1));
  text('remaining-time', active ? number(s.operation_remaining_s, 's', 1) : '—');
  const hardwareErrors = s.hardware && Array.isArray(s.hardware.errors) ? s.hardware.errors.join('; ') : '';
  text('message', stopError || s.message || hardwareErrors);
  text('failure-context', ['FAILED', 'ABORTED', 'STOPPING'].includes(s.state) ?
    `Operation: ${stageNames[s.failure_operation] || s.failure_operation || '—'} · Requested flow: ${number(s.current_target_lpm, 'L/min')} · Repeat: ${s.repeat || '—'} · Last measured position: ${number(s.position_ml, 'ml')}` : '');
  byId('stop').disabled = terminal || stopping;
  byId('emergency').disabled = terminal;
  byId('results-link').hidden = s.state !== 'COMPLETE' || !s.run_id;
  if (s.run_id) byId('results-link').href = `/results/${encodeURIComponent(s.run_id)}`;
  const last = s.last_completed_trial;
  byId('last-empty').hidden = !!last;
  byId('last-trial').hidden = !last;
  if (last) {
    text('last-target', `${number(last.target_flow_lpm, 'L/min')} / ${last.repeat ?? '—'}`);
    text('last-reference', number(last.reference_flow_lpm, 'L/min'));
    text('last-voltage', number(last.sensor_voltage_v, 'V'));
    text('last-stroke', number(last.measured_stroke_ml, 'ml'));
    text('last-quality', last.accepted === true ? 'Accepted' : last.accepted === false ? 'Rejected' : '—');
    text('last-reason', Array.isArray(last.rejection_reasons) && last.rejection_reasons.length ? last.rejection_reasons.join('; ') : '—');
  }
}

async function timedFetch(url, options = {}) {
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 4000);
  try { return await fetch(url, {...options, signal: controller.signal}); }
  finally { clearTimeout(timeout); }
}

async function poll() {
  try {
    const response = await timedFetch('/api/status', {cache: 'no-store'});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const status = await response.json();
    if (!status || typeof status.state !== 'string') throw new Error('Invalid status response');
    if (terminalStates.has(status.state) || status.state === 'STOPPING') stopPending = false;
    render(status);
    text('connection', 'Connected to instrument');
    byId('connection').className = 'muted';
  } catch (error) {
    text('connection', 'Connection lost — displayed measurements are stale. Retrying…');
    byId('connection').className = 'bad';
    bar('operation-progress', latest ? latest.operation_progress : null, false);
  } finally {
    setTimeout(poll, 500);
  }
}

async function requestStop(emergency) {
  stopError = "";
  stopPending = true;
  if (latest) render(latest);
  text('message', emergency ? 'Emergency stop requested — waiting for instrument confirmation.' : 'Stop requested — waiting for instrument confirmation.');
  try {
    const response = await timedFetch('/api/calibration/stop', {
      method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({emergency}),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
  } catch (error) {
    stopPending = false;
    if (latest) render(latest);
    stopError = `Stop could not be confirmed: ${error.message}. Retry or use the instrument's physical stop.`;
    text('message', stopError);
  }
}
byId('stop').addEventListener('click', () => requestStop(false));
byId('emergency').addEventListener('click', () => requestStop(true));
poll();
