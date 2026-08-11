class FlowSensor:
    def __init__(self, adc, channel): self._adc,self._channel=adc,channel
    def voltage(self): return self._adc.voltage(self._channel)
