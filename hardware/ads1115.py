class ADS1115:
    def __init__(self, config):
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

        self._ads = ads.ADS1115(
            busio.I2C(board.SCL, board.SDA),
            address=int(config["address"]),
            gain=config.get("gain", 1),
        )
        self._channels = {channel: AnalogIn(self._ads, channel) for channel in channels}

    def voltage(self, channel):
        return float(self._channels[channel].voltage)
