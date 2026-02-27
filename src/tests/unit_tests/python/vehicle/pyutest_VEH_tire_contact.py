import pychrono as chrono
import pychrono.vehicle as veh
import pytest
import math

tol = 1e-5

# Flat terrain inclined in the x-z plane
class TestTerrain(veh.ChTerrain):
    __test__ = False
    def __init__(self, angle):
        super().__init__()
        self.sa = math.sin(angle)
        self.ca = math.cos(angle)
        self.ta = math.tan(angle)

    def GetPoint(self, loc: chrono.ChVector3d):
        return chrono.ChVector3d(loc.x, loc.y, loc.x * self.sa)

    def GetHeight(self, loc: chrono.ChVector3d):
        return loc.x * self.sa

    def GetNormal(self, loc: chrono.ChVector3d):
        return chrono.ChVector3d(-self.sa, 0, self.ca)

    def GetCoefficientFriction(self, loc):
        return 1.0

@pytest.mark.parametrize("method", [
    veh.ChTire.CollisionType_SINGLE_POINT,
    veh.ChTire.CollisionType_FOUR_POINTS,
    veh.ChTire.CollisionType_ENVELOPE
])
@pytest.mark.parametrize("angle_deg", [0.0, 10.0, 20.0, 30.0])
def test_tire_collision(method, angle_deg):
    angle = math.radians(angle_deg)

    sa = math.sin(angle)
    ca = math.cos(angle)

    # Construct terrain inclined by given angle
    terrain = TestTerrain(angle)

    # Set up tire disc geometry
    r = 1.0
    w = 0.5
    delta = 0.1
    center = chrono.ChVector3d(0, 0, r - delta)
    normal = chrono.ChVector3d(0, 1, 0)

    csys = chrono.ChCoordsysd()
    depth = chrono.double_ptr()
    mu = chrono.float_ptr()

    # Calculate contact patch csys, penetration depth, and coefficient of friction
    if method == veh.ChTire.CollisionType_SINGLE_POINT:
        contact = veh.ChTire.DiscTerrainCollision1pt(
            terrain, center, normal, r, csys, depth, mu
        )
        print("1 point")
    
    elif method == veh.ChTire.CollisionType_FOUR_POINTS:
        contact = veh.ChTire.DiscTerrainCollision4pt(
            terrain, center, normal, r, w, csys, depth, mu
        )
        print("4 point")
    
    elif method == veh.ChTire.CollisionType_ENVELOPE:
        area_depth = chrono.ChFunctionInterp()
        veh.ChTire.ConstructAreaDepthTable(r, area_depth)
        contact = veh.ChTire.DiscTerrainCollisionEnvelope(
            terrain, center, normal, r, w, area_depth, csys, depth, mu
        )
        print("envelope")

    # Terrain normal at origin of csys and z direction of csys
    terrain_normal = terrain.GetNormal(csys.pos)

    # Z axis of csys
    csys_normal = csys.rot.GetAxisZ()

    # Distance from csys origin to terrain surface
    csys_dist = abs(sa * csys.pos.x - csys.pos.z)
    
    assert csys.pos.y == pytest.approx(0, abs=tol)
    assert csys.pos.z == pytest.approx(csys.pos.x * math.sin(angle), abs=tol)
    assert csys_dist == pytest.approx(0, abs=tol)
    assert depth.value() > 0.0
    assert csys_normal.x == pytest.approx(terrain_normal.x, abs=tol)
    assert csys_normal.y == pytest.approx(terrain_normal.y, abs=tol)
    assert csys_normal.z == pytest.approx(terrain_normal.z, abs=tol)
