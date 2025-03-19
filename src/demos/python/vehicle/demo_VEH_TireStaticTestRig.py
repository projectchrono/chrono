# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2025 projectchrono.org
# All right reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Radu Serban
# =============================================================================
#
# Demonstration of the single-wheel static tire test rig.
#
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import errno
import os
import math as m

# -----------------------------------------------------------------------------

def main():
    # --------------------------------
    # Create wheel and tire subsystems
    # --------------------------------

    print("Create wheel and tire")

    wheel = veh.ReadWheelJSON(wheel_json)
    tire = veh.ReadTireJSON(tire_json)

    # Set tire contact surface
    collision_family = 7
    surface_dim = 0.02
    tire.SetContactSurfaceType(surface_type, surface_dim, collision_family)

    # ----------------------------
    # Create system and set solver
    # ----------------------------

    print("Create Chrono system")

    sys = chrono.ChSystemSMC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    step_size = 5e-5
    tire.SetStepsize(step_size)

    sys.SetSolverType(chrono.ChSolver.Type_MINRES)
    #sys.SetSolverType(chrono.ChSolver.Type_PARDISO_MKL)
    #sys.SetSolverType(chrono.ChSolver.Type_SPARSE_QR)
    #sys.SetSolverType(chrono.ChSolver.Type_BICGSTAB)

    sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)
    #sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

    num_threads_chrono = 4     # Number of OpenMP threads used in Chrono (SCM ray-casting and FEA)
    num_threads_collision = 4  # Number of threads used in collision detection
    num_threads_eigen = 1      # Number of threads used by Eigen
    num_threads_pardiso = 4    # Number of threads used by PardisoMKL
    sys.SetNumThreads(num_threads_chrono, num_threads_collision, num_threads_eigen)

    # -----------------------------
    # Create and configure test rig
    # -----------------------------

    print("Create test rig")

    g = 9.8

    rig = veh.ChTireStaticTestRig(wheel, tire, sys)

    rig.SetGravitationalAcceleration(g)

    # Set tire options
    rig.SetTireStepsize(step_size)
    rig.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Set rig options
    rig.SetPlateMaterialProperties(0.8, 0, 2e7)

    # Set test parameters
    rig.SetNominalRadialLoad(3600 * g)
    rig.SetStateTransitionDelay(0.1)
    rig.SetRadialLoadSpeed(0.001)
    rig.SetLongitudinalLoadSpeed(0.1)
    rig.SetLateralLoadSpeed(0.1)
    rig.SetTorsionalLoadSpeed(0.1)

    # Initialize the tire test rig
    rig.Initialize(test_mode)

    # ---------------------------------
    # Create the run-time visualization
    # ---------------------------------

    print("Create visualization system")

    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Tire Test Rig')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1, 2.5, 1), rig.GetWheelPos())
    vis.AddTypicalLights()

    # ---------------
    # Simulation loop
    # ---------------

    # Timers and counters
    timer = chrono.ChTimer() # timer for measuring total run time
    time = 0                 # simulated time
    sim_time = 0             # simulation time
    fps = 120                # rendering frequency
    render_frame = 0         # render frame counter

    timer.start()

    while vis.Run():
        time = sys.GetChTime()

        if time >= render_frame / fps:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        rig.Advance(step_size)
        sim_time = sim_time + sys.GetTimerStep()

        if debug_output:
            print(f"{time:.3f}", "  state: ", rig.GetStateName(), "  load: ", f"{rig.GetLoad():.1f}")
    
    timer.stop()

    step_time = timer.GetTimeSeconds()
    #cout << "\rSimulated time: " << time << endl;
    #cout << "Run time (simulation): " << sim_time << "  |  RTF: " << sim_time / time << endl;
    #cout << "Run time (total):      " << step_time << "  |  RTF: " << step_time / time << endl;

# =============================================================================

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

tire_json = veh.GetDataFile("hmmwv/tire/HMMWV_MBTire.json")
wheel_json = veh.GetDataFile("hmmwv/wheel/HMMWV_Wheel.json")

# Tire contact model (nodes or faces)
surface_type = veh.ChTire.ContactSurfaceType_NODE_CLOUD

# Test mode
test_mode = veh.ChTireStaticTestRig.Mode_TEST_R

debug_output = False

main()
