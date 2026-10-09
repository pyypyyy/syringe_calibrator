import threading
import time

class ADS1115:
    def __init__(self, config):
        self._lock = threading.Lock()
        channel_names = ("flow_channel", "softpot_channel")
        channels = []
        for name in channel_names:
            channel = config[name]
            if isinstance(channel, bool) or not isinstance(channel, int) or not 0 <= channel <= 3:
                raise ValueError(
                    f"ADS1115 {name} must be an integer from 0 through 3; got {channel!r}"
                )
            channels.append(channel)

        import board
        import busio
        import adafruit_ads1x15.ads1115 as ads
        from adafruit_ads1x15.analog_in import AnalogIn

        class TimedADS1115(ads.ADS1115):
            # The upstream single-shot driver polls this method indefinitely.
            # Bound that polling without replacing its conversion/MUX logic.
            _conversion_deadline = None

            def _conversion_complete(self):
                if (self._conversion_deadline is not None
                        and time.monotonic() >= self._conversion_deadline):
                    raise TimeoutError("ADS1115 conversion timed out")
                return super()._conversion_complete()

        self._ads = TimedADS1115(
            busio.I2C(board.SCL, board.SDA),
            address=int(config["address"]),
            gain=config.get("gain", 1),
        )
        self._channels = {channel: AnalogIn(self._ads, channel) for channel in channels}

    def voltage(self, channel):
        with self._lock:
            self._ads._conversion_deadline = time.monotonic() + 1.0
            try:
                return float(self._channels[channel].voltage)
            finally:
                self._ads._conversion_deadline = None
