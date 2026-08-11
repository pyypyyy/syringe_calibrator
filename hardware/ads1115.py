class ADS1115:
    def __init__(self, config):
        import board
        import busio
        import adafruit_ads1x15.ads1115 as ads
        from adafruit_ads1x15.analog_in import AnalogIn
        self._ads=ads.ADS1115(busio.I2C(board.SCL,board.SDA),address=int(config["address"]),gain=config.get("gain",1))
        self._channels={i:AnalogIn(self._ads,getattr(ads,f"P{i}")) for i in (config["flow_channel"],config["softpot_channel"])}
    def voltage(self, channel): return float(self._channels[channel].voltage)
