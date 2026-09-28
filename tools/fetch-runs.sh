#!/usr/bin/env bash
# Copies runs/ from the VM to the Mac.
set -euo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
# Set FTS_VM (user@host of the VM) and FTS_VM_KEY (SSH key for it).
VM=${FTS_VM:?set FTS_VM to user@host of the VM}
KEY=${FTS_VM_KEY:?set FTS_VM_KEY to the SSH key for the VM}
rsync -a --delete -e "ssh -i $KEY" "$VM:fts-sim/runs/" "$REPO/runs/"
