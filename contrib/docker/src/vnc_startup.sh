## =============================================================================
## PROJECT CHRONO - http://projectchrono.org
##
## Copyright (c) 2023 projectchrono.org
## All rights reserved.
##
## Use of this source code is governed by a BSD-style license that can be found
## in the LICENSE file at the top level of the distribution and at
## http://projectchrono.org/license-chrono.txt.
##
## =============================================================================
## Authors: Thomas Liang
## =============================================================================

#!/bin/bash
set -e

# Shutdown signal
cleanup () {
    kill -s SIGTERM $!
    exit 0
}
trap cleanup SIGINT SIGTERM

## VNC password
mkdir -p "$HOME/.vnc"
echo "$VNC_PW" | vncpasswd -f >> $HOME/.vnc/passwd

# noVNC
$NO_VNC_HOME/utils/novnc_proxy --vnc localhost:$VNC_PORT --listen $NO_VNC_PORT &
PID_SUB=$!

# VNC
vncserver -kill $DISPLAY &> $STARTUPDIR/vnc_startup.log \
    || rm -rfv /tmp/.X*-lock /tmp/.X11-unix &> $STARTUPDIR/vnc_startup.log \
    || echo "No locks present"
vncserver $DISPLAY -depth $VNC_COL_DEPTH -geometry $VNC_RESOLUTION PasswordFile=$HOME/.vnc/passwd &

# XFCE
$HOME/wm_startup.sh &
wait $PID_SUB
