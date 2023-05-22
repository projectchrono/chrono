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

xset -dpms &
xset s noblank &
xset s off &

/usr/bin/startxfce4 --replace &
