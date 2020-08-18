import logging
import subprocess
from pathlib import Path

import toml

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    # get deps
    path = Path.cwd() / "pyproject.toml"

    logging.info(f"Loading {path.resolve()}")
    with open(path) as f:
        config = toml.load(f)

    deps = config["tool"]["poetry"]["dev-dependencies"]

    # process deps
    for d in deps.keys():
        logging.info(f"Updating {d} to latest")
        args = ["poetry", "add", "--dev", f"{d}@latest"]
        subprocess.run(args)
