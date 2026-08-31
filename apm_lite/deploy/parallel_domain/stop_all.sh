#!/usr/bin/env bash
set -euo pipefail

for session in pd-px4-real pd-px4-virtual pd-apm-real pd-apm-virtual; do
  tmux has-session -t "$session" 2>/dev/null && tmux kill-session -t "$session"
done
echo "parallel-domain sessions stopped"
