"""Make repository packages importable with both `pytest` and `python -m pytest`."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parents[1]))
