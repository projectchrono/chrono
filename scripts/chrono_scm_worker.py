#!/usr/bin/env python3
"""Chrono SCM subprocess worker — evaluates wheel contacts in an isolated process.

This avoids the Python SWIG deadlock (SCMTerrain + Bullet collision on macOS
arm64) by running Chrono in a dedicated subprocess with no conflicting shared
libraries (no PyTorch, no scipy, no OMP duplicate).
"""

import json
import sys

def evaluate_batch(queries_json: str) -> str:
    import numpy as np
    import pychrono.core as c
    import pychrono.vehicle as cv

    queries = json.loads(queries_json)
    results = []

    s = c.ChSystemNSC()
    s.SetNumThreads(1)
    s.SetGravitationalAcceleration(c.ChVector3d(0, 0, -9.81))
    cs = c.ChCollisionSystemBullet()
    s.SetCollisionSystem(cs)

    scm = cv.SCMTerrain(s)
    scm.SetSoilParameters(2e6, 0, 1.1, 50000, np.tan(np.radians(20)), 0.01, 5e7, 0)
    scm.Initialize(5.0, 5.0, 0.1)

    mat = c.ChContactMaterialNSC()

    for q in queries:
        body = c.ChBody()
        body.SetPos(c.ChVector3d(q["x"], q["y"], q["z"]))
        body.SetMass(0.5)
        body.SetInertiaXX(c.ChVector3d(0.001, 0.001, 0.001))
        body.AddCollisionShape(c.ChCollisionShapeSphere(mat, 0.02))
        s.AddBody(body)

        for _ in range(40):
            body.AccumulateForce(0, c.ChVector3d(0, 0, -q["load"]), body.GetPos(), False)
            s.DoStepDynamics(1e-4)

        force = c.ChVector3d()
        torque = c.ChVector3d()
        has_contact = scm.GetContactForceBody(body, force, torque)
        z = body.GetPos().z
        sinkage = max(0.0, q["z"] - z)

        results.append({
            "wheel_id": q["wheel_id"],
            "sinkage_m": float(sinkage),
            "traction_force_n": float(force.x) if has_contact else 0.0,
            "rolling_resistance_n": 0.0,
            "lateral_force_n": float(force.y) if has_contact else 0.0,
            "slip_ratio": 0.0,
            "ground_pressure_pa": float(-force.z) / 1e-4 if has_contact else 0.0,
            "contact_area_m2": 0.0,
        })

        s.RemoveBody(body)

    return json.dumps(results)


if __name__ == "__main__":
    input_data = sys.stdin.read()
    output = evaluate_batch(input_data)
    sys.stdout.write(output)
    sys.stdout.flush()
