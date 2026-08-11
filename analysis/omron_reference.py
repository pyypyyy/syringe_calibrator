"""Omron D6F-P0010A1 manufacturer representative curve (manual A299)."""
from __future__ import annotations
import numpy as np

OMRON_COEFFICIENTS = (0.094003, -0.564312, 1.374705, -1.601495, 1.060657, -0.269996)

def omron_reference(voltage_v):
    """Return reference flow (L/min); this is not an individual calibration."""
    result = np.polyval(OMRON_COEFFICIENTS, voltage_v)
    return float(result) if np.ndim(result) == 0 else result
