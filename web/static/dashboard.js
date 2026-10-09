document.querySelector('#start')?.addEventListener('click',async()=>{const body={gas:gas.value,targets_lpm:targets.value.split(',').map(Number),repeats:+repeats.value};const r=await fetch('/api/calibration/start',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});if(r.ok)location='/calibration';else alert((await r.json()).error)});

async function refreshSensors() {
  try {
    const response = await fetch('/api/sensors', {cache: 'no-store'});
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || 'Sensor read failed');
    const volts = value => typeof value === 'number' ? `${value.toFixed(4)} V` : 'Unavailable';
    document.querySelector('#flow-voltage').textContent = volts(data.flow_voltage_v);
    document.querySelector('#softpot-voltage').textContent = volts(data.softpot_voltage_v);
    document.querySelector('#sensor-status').textContent = data.source === 'live' ? 'Live' : 'Calibration in progress';
  } catch (error) {
    document.querySelector('#flow-voltage').textContent = 'Unavailable';
    document.querySelector('#softpot-voltage').textContent = 'Unavailable';
    document.querySelector('#sensor-status').textContent = error.message;
  } finally {
    setTimeout(refreshSensors, 1000);
  }
}
refreshSensors();
