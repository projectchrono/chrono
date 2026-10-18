import pytest
import pychrono as chrono

GRAVITY = 9.81
MASS = 2.0


def make_pendulum():
    """Pendulum hanging at rest below a revolute joint to the ground."""
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -GRAVITY, 0))

    ground = chrono.ChBody()
    ground.SetFixed(True)
    sys.Add(ground)

    body = chrono.ChBody()
    body.SetMass(MASS)
    body.SetPos(chrono.ChVector3d(0, -1, 0))
    sys.Add(body)

    joint = chrono.ChLinkLockRevolute()
    joint.Initialize(body, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    sys.Add(joint)

    sys.Setup()
    return sys, joint


def test_state_vector_sizing():
    sys, _ = make_pendulum()
    nx = sys.GetNumCoordsPosLevel()
    nv = sys.GetNumCoordsVelLevel()

    x = chrono.ChState(nx, sys)
    v = chrono.ChStateDelta(nv, sys)
    assert x.Size() == nx
    assert v.Size() == nv

    y = chrono.ChState(sys)
    y.setZero(nx, sys)
    assert y.Size() == nx


def test_state_gather():
    sys, _ = make_pendulum()
    sys.DoStepDynamics(1e-3)

    x = chrono.ChState(sys.GetNumCoordsPosLevel(), sys)
    v = chrono.ChStateDelta(sys.GetNumCoordsVelLevel(), sys)
    T = sys.StateGather(x, v)
    assert T == pytest.approx(sys.GetChTime())
    # x holds position and rotation of the pendulum body
    assert x[1] == pytest.approx(-1.0, abs=1e-6)


def test_state_solve_acceleration():
    # Body sliding freely along a vertical prismatic joint accelerates with gravity
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -GRAVITY, 0))
    ground = chrono.ChBody()
    ground.SetFixed(True)
    sys.Add(ground)
    body = chrono.ChBody()
    body.SetMass(MASS)
    sys.Add(body)
    joint = chrono.ChLinkLockPrismatic()
    joint.Initialize(body, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI_2)))
    sys.Add(joint)
    sys.Setup()
    sys.DoAssembly(chrono.UpdateFlags_UPDATE_ALL)

    x = chrono.ChState(sys.GetNumCoordsPosLevel(), sys)
    v = chrono.ChStateDelta(sys.GetNumCoordsVelLevel(), sys)
    a = chrono.ChStateDelta(sys.GetNumCoordsVelLevel(), sys)
    L = chrono.ChVectorDynamicd(sys.GetNumConstraints())
    T = sys.StateGather(x, v)
    sys.StateScatter(x, v, T, chrono.UpdateFlags_UPDATE_ALL)

    assert sys.StateSolveA(a, L, x, v, T, 1e-3, False, chrono.UpdateFlags_UPDATE_ALL)
    expected = [0, -GRAVITY, 0, 0, 0, 0]
    for i in range(a.Size()):
        assert a[i] == pytest.approx(expected[i], abs=1e-9)


def test_system_descriptor():
    sys, _ = make_pendulum()
    sys.DoStepDynamics(1e-3)

    descriptor = sys.GetSystemDescriptor()
    assert isinstance(descriptor, chrono.ChSystemDescriptor)
    assert hasattr(descriptor, "WriteMatrixBlocks")


def test_wrench_member_of_temporary():
    sys, joint = make_pendulum()
    sys.DoStepDynamics(1e-3)

    held = joint.GetReaction2()
    expected = held.force.Length()
    assert expected == pytest.approx(MASS * GRAVITY, rel=1e-3)

    # Chained access on the returned temporary must not read freed memory
    for _ in range(100):
        assert joint.GetReaction2().force.Length() == pytest.approx(expected)
        assert joint.GetReaction2().torque.Length() == pytest.approx(held.torque.Length())

    # Writing through the member still modifies the wrench in place
    held.force.x = 5.0
    assert held.force.x == 5.0
