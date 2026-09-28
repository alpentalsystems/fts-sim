#!/usr/bin/env bash
# Copies runs/ from the VM to the Mac.
set -euo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
VM=${FTS_VM:-user@vm-host}
KEY=${FTS_VM_KEY:-$HOME/.ssh/id_ed25519_vm}
rsync -a --delete -e "ssh -i $KEY" "$VM:fts-sim/runs/" "$REPO/runs/"
