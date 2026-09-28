#!/usr/bin/env bash
# Copies the working tree to ~/fts-sim on the VM.
set -euo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
VM=${FTS_VM:-user@vm-host}
KEY=${FTS_VM_KEY:-$HOME/.ssh/id_ed25519_vm}
rsync -a --delete \
	--exclude build/ --exclude run/ --exclude runs/ --exclude .superpowers/ \
	--exclude ros2_ws/build/ --exclude ros2_ws/install/ --exclude ros2_ws/log/ \
	--exclude .pytest_cache/ --exclude __pycache__/ \
	-e "ssh -i $KEY" "$REPO/" "$VM:fts-sim/"
