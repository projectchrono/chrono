# A unit test that checks if PyChrono SWIG interface can work with the underlying multi-thread FSI-SPH solver correctly.
# This unit test is for PyChrono only

import pychrono as chrono
import pychrono.vsg3d as vsg3d
import pychrono.fsi as fsi

import threading

# =============================================================================

main_thread_id = threading.get_native_id()
solid_thread_id = 0

# =============================================================================

# Functor class implementing the force for a ChLinkTSDA link.
# In this simple demonstration, we just reimplement the default linear spring-damper.
class MySpringForce(chrono.ForceFunctor):
    def __init__(self):
        super(MySpringForce, self).__init__()
    
    def evaluate(self,         #
                 time,         # current time
                 rest_length,  # undeformed length
                 length,       # current length
                 vel,          # current velocity (positive when extending)
                 link):        # associated link
        global solid_thread_id
        solid_thread_id = threading.get_native_id()
        force = -50 * (length - rest_length) - 1 * vel
        return force

# =============================================================================

def test_fsisph_multithread():
    step_size = 5e-4

    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    # Create the ground body with two visualization spheres
    # -----------------------------------------------------

    ground = chrono.ChBody()
    sys.AddBody(ground)
    ground.SetFixed(True)
    ground.EnableCollision(False)

    sph = chrono.ChVisualShapeSphere(0.1)
    ground.AddVisualShape(sph, chrono.ChFramed(chrono.ChVector3d(1, 0, 0)))

    # Create a body suspended through a ChLinkTSDA (custom force functor)
    # -------------------------------------------------------------------

    body = chrono.ChBody()
    sys.AddBody(body)
    body.SetPos(chrono.ChVector3d(1, -3, 0))
    body.SetFixed(False)
    body.EnableCollision(False)
    body.SetMass(1)
    body.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

    geometry = chrono.ChBodyGeometry()
    geometry.materials.push_back(chrono.ChContactMaterialData())
    geometry.coll_boxes.push_back(chrono.BoxShape(chrono.VNULL, chrono.QUNIT, chrono.ChBox(1.0, 1.0, 1.0)))

    # Create the spring between body and ground. The spring end points are
    # specified in the body relative frames.
    force = MySpringForce()

    spring = chrono.ChLinkTSDA()
    spring.Initialize(body, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(1, 0, 0))
    spring.SetRestLength(1.5)
    spring.RegisterForceFunctor(force)
    sys.AddLink(spring)

    # Create FSI
    terrain = fsi.ChFsiProblemCartesian(0.04, sys)
    sysFSI = terrain.GetFsiSystemSPH()
    terrain.SetVerbose(False)
    terrain.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
    terrain.SetStepSizeCFD(step_size)

    terrain.Construct(
        chrono.ChVector3d(4.0, 4.0, 0.25),
        chrono.ChVector3d(4.0/2, 0, -2),
        (fsi.BoxSide_ALL & ~fsi.BoxSide_Z_POS)
    )

    terrain.AddRigidBody(body, geometry, True)
    terrain.Initialize()

    for _ in range(4):
        sysFSI.DoStepDynamics(step_size)

    assert main_thread_id != solid_thread_id and solid_thread_id != 0
