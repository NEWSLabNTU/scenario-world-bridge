"""Make `swb` importable when pytest runs from the repository root, before a colcon install."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
