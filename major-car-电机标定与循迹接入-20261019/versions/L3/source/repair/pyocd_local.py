"""Use this task's isolated pyOCD install; system Python packages are unchanged."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent/'tools'))
from pyocd.__main__ import main
main()
