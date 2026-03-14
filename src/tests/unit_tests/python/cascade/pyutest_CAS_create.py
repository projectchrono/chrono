# Unit test to create a OpenCascade shape and verify if it's dynamics is correct.

import pychrono.core as chrono
import pychrono.cascade as cascade
from OCC.Core import BRepPrimAPI
import numpy as np

def test_create_cylinder():
    #  Create the simulation system and add items
    sys = chrono.ChSystemNSC()
    sys.SetGravityY()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Set the global collision margins. This is expecially important for very large or
    # very small objects. Set this before creating shapes. Not before creating sys.
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)

    # A collision material, will be used by two colliding shapes
    material = chrono.ChContactMaterialNSC()
    material.SetFriction(0.5)

    cylinder = BRepPrimAPI.BRepPrimAPI_MakeCylinder(0.09,0.1).Shape()

    # use it to make a body with proper center of mass and inertia tensor,
    # given the CAD shape. Also visualize it.
    vis_params = cascade.ChCascadeTriangulate(0.1, True, 0.5)

    body = cascade.ChCascadeBodyEasy(cylinder,       # the CAD shape
                                    1000,        # the density
                                    vis_params,  # must visualize triangle mesh geometry?
                                    True,        # must collide?
                                    material)    # collision material
    sys.Add(body)

    # Create a large cube as a floor.
    floor = chrono.ChBodyEasyBox(1, 0.2, 1,   # x y z size
                                1000,        # density
                                True,        # must visualize?
                                True,        # must collide?
                                material)    # collision material
    floor.SetPos(chrono.ChVector3d(0,-0.3,0))
    floor.SetFixed(True)
    floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/blue.png'))
    sys.Add(floor)


    #  Run the simulation
    sys.SetSolverType(chrono.ChSolver.Type_PSOR)

    expected_start_pos = np.array([0, 0, 0.05])
    expected_end_pos = np.array([0, -0.11, 0.05])

    assert np.allclose(body.GetPos(), expected_start_pos, atol=1e-3)

    for _ in range(64):
        sys.DoStepDynamics(0.005)

    assert np.allclose(body.GetPos(), expected_end_pos, atol=1e-3)