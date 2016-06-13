#!/bin/bash
fuser -n tcp -k 8000 
fuser -n tcp -k 5000
fuser -n tcp -k 80  
tmux new-session -s MetricsAPI -n etc -d 'cd /etc'

tmux new-window -n backend -s:MetricsAPI 'cd metrics-database; ./backend_launcher.sh'
tmux new-window -n frontend -t:backend 'cd metrics-database; ./frontend_launcher.sh'
tmux new-window -n site -t:backend 'nginx'

