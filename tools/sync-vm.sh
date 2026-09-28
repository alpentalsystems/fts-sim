#!/usr/bin/env bash
# Copies the working tree to ~/fts-sim on the VM.
set -euo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
# Set FTS_VM (user@host of the VM) and FTS_VM_KEY (SSH key for it).
VM=${FTS_VM:?set FTS_VM to user@host of the VM}
KEY=${FTS_VM_KEY:?set FTS_VM_KEY to the SSH key for the VM}
rsync -a --delete \
	--exclude build/ --exclude run/ --exclude runs/ --exclude .superpowers/ \
	--exclude ros2_ws/build/ --exclude ros2_ws/install/ --exclude ros2_ws/log/ \
	--exclude .pytest_cache/ --exclude __pycache__/ \
	-e "ssh -i $KEY" "$REPO/" "$VM:fts-sim/"
