#!/usr/bin/env bash
# One-time VM setup: Zephyr v4.4.2 workspace for native_sim, no modules.
set -euo pipefail
sudo apt-get install -y --no-install-recommends device-tree-compiler gperf
python3 -m pip install --user west
export PATH=$HOME/.local/bin:$PATH
if [ ! -d "$HOME/zephyrproject/zephyr" ]; then
	mkdir -p "$HOME/zephyrproject"
	# Shallow clone of the tag only; west init -m fetches the whole history.
	git clone --depth 1 -b v4.4.2 https://github.com/zephyrproject-rtos/zephyr "$HOME/zephyrproject/zephyr"
	(cd "$HOME/zephyrproject" && west init -l zephyr)
fi
cd "$HOME/zephyrproject"
# No modules are needed for native_sim.
west config manifest.project-filter -- '-.*'
# Zephyr v4.4 needs Python 3.12; Ubuntu 22.04 (and ROS 2 Humble) has 3.10.
# A user-local 3.12 venv is used for firmware builds only.
python3 -m pip install --user uv
uv python install 3.12
if [ ! -d .venv ]; then
	uv venv --python 3.12 .venv
fi
VIRTUAL_ENV=$HOME/zephyrproject/.venv uv pip install west -r zephyr/scripts/requirements-base.txt
