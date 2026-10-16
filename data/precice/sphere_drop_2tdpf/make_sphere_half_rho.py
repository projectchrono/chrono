# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2026 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
#
# Generate 'fluid_tdpf/sphere_half_rho.h5' for the sphere_drop_2tdpf demo from
# the sphere hydrodynamic file used by the single-TDPF sphere drop demo
# (data/precice/sphere_drop/fluid_tdpf/sphere.h5), by halving the fluid
# density stored in 'simulation_parameters/rho'.
#
# The TDPF solver dimensionalizes every quantity it reads from the hydrodynamic
# file with this density: infinite-frequency added mass and radiation impulse
# response (rho), linear restoring stiffness and wave excitation (rho*g), and
# the buoyancy force (rho*g*displaced volume). Halving the density therefore
# halves all hydrodynamic loads and the infinite-frequency added mass.
#
# In the sphere_drop_2tdpf demo, denoting by M the sphere's infinite-frequency
# added mass, this file is used by:
# - both TDPF participants, each of which applies half of the hydrodynamic
#   loads and has an infinite-frequency added mass of M/2 (the demo scales the
#   latter to obtain the added mass increments it sends, M/6 and M/3), and
# - the solid participant, which reads its baseline added mass (M/2) from it.
#
# Usage (requires h5py):
#   python make_sphere_half_rho.py
#
# =============================================================================

import os
import shutil

import h5py

here = os.path.dirname(os.path.abspath(__file__))
src = os.path.join(here, "..", "sphere_drop", "fluid_tdpf", "sphere.h5")
dst = os.path.join(here, "fluid_tdpf", "sphere_half_rho.h5")

shutil.copyfile(src, dst)

with h5py.File(dst, "r+") as f:
    rho = f["simulation_parameters/rho"]
    old = rho[()].copy()
    rho[...] = 0.5 * old
    print(f"{os.path.relpath(dst, here)}: simulation_parameters/rho {old.ravel()[0]} -> {rho[()].ravel()[0]}")
