#!/bin/bash

TMUX_SESSION_NAME="code"
RUN_SCRIPT="start_code.sh"

cd "$(dirname "$0")"
tmux new-session -d -s ${TMUX_SESSION_NAME} 
tmux send-keys -t ${TMUX_SESSION_NAME}:0 "./${RUN_SCRIPT}" C-m

