import threading
class Stepper:
    def __init__(self, config):
        import pigpio
        self.pi=pigpio.pi()
        if not self.pi.connected: raise RuntimeError("pigpio daemon unavailable")
        self.cfg=config; self._lock=threading.Lock(); self.pi.set_mode(config["step_pin"],pigpio.OUTPUT); self.pi.set_mode(config["dir_pin"],pigpio.OUTPUT); self.pi.set_mode(config["enable_pin"],pigpio.OUTPUT); self.disable()
    def disable(self):
        with self._lock: self.pi.wave_tx_stop(); self.pi.write(self.cfg["step_pin"],0); self.pi.write(self.cfg["enable_pin"],1 if self.cfg["enable_active_low"] else 0)
    def move(self, steps, frequency_hz, stop_event):
        import pigpio
        if not steps:return
        direction=(steps>0)^bool(self.cfg.get("invert_direction")); period=max(2,int(1_000_000/frequency_hz)); count=abs(steps)
        with self._lock:
            self.pi.write(self.cfg["dir_pin"],int(direction)); self.pi.write(self.cfg["enable_pin"],0 if self.cfg["enable_active_low"] else 1)
            self.pi.wave_clear(); self.pi.wave_add_generic([pigpio.pulse(1<<self.cfg["step_pin"],0,period//2),pigpio.pulse(0,1<<self.cfg["step_pin"],period-period//2)]); wid=self.pi.wave_create(); self.pi.wave_chain([255,0,wid,255,1,count&255,(count>>8)&255])
        while self.pi.wave_tx_busy() and not stop_event.wait(.02): pass
        self.disable(); self.pi.wave_delete(wid)
    def close(self): self.disable(); self.pi.stop()
