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
# Reconstruct the fluid surface for each output frame of demo_FSI-SPH_TankTrailer (run with --blender).
#
# For each frame:
# - read the SPH particle positions (particles/fluid_NNNNN.json, in the frame translating with the tank);
# - read the tank orientation from the ChBlender state file (blender/output/stateNNNNN.py);
# - drop particles lodged in the tank walls (more than half a spacing outside the tank inner surface);
# - run splashsurf on the remaining particles and write surfaces/fluid_NNNNN.ply (same frame as the particles).
#
# The tank dimensions and the mirror location below must match those in demo_FSI-SPH_TankTrailer.cpp, and --spacing
# must match the initial particle spacing of the run (the demo's --spacing option).
#
# Usage: python make_fluid_surfaces.py <run_dir> --spacing <spacing> [--first 0] [--last -1] [--jobs 4] [--threads 3]
#
# splashsurf must be on the PATH, or its executable given in the SPLASHSURF environment variable.
# See README.md for the complete pipeline.

import argparse, json, os, re, subprocess, sys
import numpy as np
from concurrent.futures import ThreadPoolExecutor

TANK_RADIUS = 0.33
TANK_LENGTH = 1.40
MIRROR_LOC = np.array([0.0, 0.0, 0.0])

SPLASHSURF = os.environ.get('SPLASHSURF', 'splashsurf')

TANK_RE = re.compile(r"make_chrono_object_assetlist\('tank',\(([^)]*)\),\(([^)]*)\)")


def tank_rotation(state_file):
    """Rotation matrix of the tank (and of the mirror body) from a ChBlender state file."""
    with open(state_file) as f:
        m = TANK_RE.search(f.read())
    if not m:
        raise RuntimeError(f'tank not found in {state_file}')
    w, x, y, z = (float(v) for v in m.group(2).split(','))
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def process(run_dir, frame, spacing, args):
    pfile = os.path.join(run_dir, 'particles', f'fluid_{frame:05d}.json')
    sfile = os.path.join(run_dir, 'blender', 'output', f'state{frame:05d}.py')
    ffile = os.path.join(run_dir, 'particles_filtered', f'fluid_{frame:05d}.json')
    ofile = os.path.join(run_dir, 'surfaces', f'fluid_{frame:05d}.ply')

    p = np.array(json.load(open(pfile)))
    R = tank_rotation(sfile)
    q = (p - MIRROR_LOC) @ R  # positions in the tank frame (q = R^T (p - c))
    tol = spacing / 2
    inside = (np.abs(q[:, 0]) <= TANK_LENGTH / 2 + tol) & (np.hypot(q[:, 1], q[:, 2]) <= TANK_RADIUS + tol)
    with open(ffile, 'w') as f:
        json.dump(p[inside].tolist(), f)

    radius = spacing / 2
    cmd = [SPLASHSURF, 'reconstruct', ffile, '-o', ofile, '-q',
           '-r', str(radius), '-l', str(args.smoothing_length), '-c', str(args.cube_size), '-t', str(args.threshold),
           '--normals=on', '--mesh-cleanup=on', '--decimate-barnacles=on',
           '--mesh-smoothing-iters', str(args.smoothing_iters), '--normals-smoothing-iters', '10',
           '--num-threads', str(args.threads)]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
    return frame, int((~inside).sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('--spacing', type=float, required=True, help='initial particle spacing of the run')
    ap.add_argument('--first', type=int, default=0, help='first frame')
    ap.add_argument('--last', type=int, default=-1, help='last frame (default: last frame of the run)')
    ap.add_argument('--jobs', type=int, default=4, help='number of frames processed concurrently')
    ap.add_argument('--threads', type=int, default=3, help='splashsurf threads per frame')
    ap.add_argument('--smoothing-length', type=float, default=2.0, help='splashsurf smoothing length (in particle radii)')
    ap.add_argument('--cube-size', type=float, default=0.6, help='splashsurf marching cubes cell size (in particle radii)')
    ap.add_argument('--threshold', type=float, default=0.6, help='splashsurf iso-surface threshold')
    ap.add_argument('--smoothing-iters', type=int, default=15, help='splashsurf mesh smoothing iterations')
    args = ap.parse_args()

    run_dir = args.run_dir
    os.makedirs(os.path.join(run_dir, 'particles_filtered'), exist_ok=True)
    os.makedirs(os.path.join(run_dir, 'surfaces'), exist_ok=True)
    n = len([f for f in os.listdir(os.path.join(run_dir, 'particles')) if f.endswith('.json')])
    last = n - 1 if args.last < 0 else args.last
    frames = range(args.first, last + 1)

    with ThreadPoolExecutor(max_workers=args.jobs) as ex:
        for frame, dropped in ex.map(lambda fr: process(run_dir, fr, args.spacing, args), frames):
            print(f'frame {frame:5d}: dropped {dropped} particles', flush=True)


if __name__ == '__main__':
    main()
