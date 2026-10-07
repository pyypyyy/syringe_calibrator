"""Finite pigpio moves for the directly connected, optoisolated TB6600."""
import math
import threading


class Stepper:
    def __init__(self, config):
        import pigpio
        self.cfg = config
        self._lock = threading.Lock()
        self._move_lock = threading.Lock()
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon unavailable")
        try:
            pins = [config[name] for name in ("step_pin", "dir_pin", "enable_pin")]
            if len(set(pins)) != 3 or any(not isinstance(p, int) or not 0 <= p <= 27 for p in pins):
                raise ValueError("Stepper requires three distinct GPIOs in bank 0 (0–27)")
            strength = config.get("drive_strength_ma", 12)
            if strength not in range(2, 17, 2):
                raise ValueError("drive_strength_ma must be an even value from 2 to 16")
            self._check(self.pi.set_pad_strength(0, strength), "set GPIO drive strength")
            if self.pi.get_pad_strength(0) != strength:
                raise RuntimeError("GPIO drive-strength verification failed")
            # Set the output latch before enabling the output to avoid an enable glitch.
            self._check(self.pi.write(config["enable_pin"], self._disabled_level), "disable driver")
            self._check(self.pi.write(config["step_pin"], 0), "clear STEP")
            for pin in pins:
                self._check(self.pi.set_mode(pin, pigpio.OUTPUT), "configure GPIO")
            self.disable()
        except Exception:
            self.pi.stop()
            raise

    @property
    def _disabled_level(self):
        return 1 if self.cfg["enable_active_low"] else 0

    @staticmethod
    def _check(result, operation):
        if result < 0:
            raise RuntimeError(f"pigpio could not {operation}: error {result}")
        return result

    def disable(self):
        with self._lock:
            try:
                self.pi.wave_tx_stop()
            finally:
                try:
                    self.pi.write(self.cfg["step_pin"], 0)
                finally:
                    self.pi.write(self.cfg["enable_pin"], self._disabled_level)

    def move(self, steps, frequency_hz, stop_event):
        import pigpio
        if not steps or stop_event.is_set():
            return
        if not isinstance(steps, int) or abs(steps) > 65535:
            raise ValueError("A move must contain 1–65535 integer steps")
        if not math.isfinite(frequency_hz) or not 0 < frequency_hz <= 100000:
            raise ValueError("Invalid step frequency")
        direction = (steps > 0) ^ bool(self.cfg.get("invert_direction"))
        period = max(2, round(1_000_000 / frequency_hz))
        count = abs(steps)
        with self._move_lock:
            wid = None
            try:
                with self._lock:
                    if stop_event.is_set():
                        return
                    self._check(self.pi.write(self.cfg["dir_pin"], int(direction)), "set direction")
                    self._check(self.pi.write(self.cfg["enable_pin"], 1 - self._disabled_level), "enable driver")
                    # Allow direction/enable to settle, interruptibly, before the first pulse.
                    if stop_event.wait(.001):
                        return
                    self._check(self.pi.wave_clear(), "clear waves")
                    self._check(self.pi.wave_add_generic([
                        pigpio.pulse(1 << self.cfg["step_pin"], 0, period // 2),
                        pigpio.pulse(0, 1 << self.cfg["step_pin"], period - period // 2),
                    ]), "build wave")
                    wid = self._check(self.pi.wave_create(), "create wave")
                    if stop_event.is_set():
                        return
                    self._check(self.pi.wave_chain([255, 0, wid, 255, 1, count & 255, count >> 8]), "start wave")
                while self.pi.wave_tx_busy() and not stop_event.wait(.005):
                    pass
            finally:
                self.disable()
                if wid is not None:
                    self.pi.wave_delete(wid)

    def close(self):
        self.disable()
        self.pi.stop()
