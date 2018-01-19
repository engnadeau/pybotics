"""Pybotics modules."""
from importlib import import_module
from pathlib import Path

from typing import List

# glob modules
path = Path(__file__).parent
modules = list(path.glob('*.py'))  # type: List[Path]

# import
for mod in modules:
    import_module('.{}'.format(mod.stem), package=path.name)
