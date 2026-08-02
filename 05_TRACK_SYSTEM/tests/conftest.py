from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT / "pc", ROOT / "robot"):
    value = str(path)
    if value not in sys.path:
        sys.path.insert(0, value)
