# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2014 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Simone Benatti
# =============================================================================
#
# Chrono demonstration of using contact callbacks for non-smooth contacts
# (complementarity-based).
#
# The global reference frame has Y up.
#
# =============================================================================

import pychrono as chrono
import pychrono.irrlicht as chronoirr

# -----------------------------------------------------------------------------
# Callback class for contact reporting
# -----------------------------------------------------------------------------
class ContactReporter (chrono.ReportContactCallback):
    def __init__(self, box) : 
        self.m_box = box
        super().__init__()

    def OnReportContact(         self,
                                 pA,
                                 pB,
                                 plane_coord,
                                 distance,
                                 eff_radius,
                                 cforce,
                                 ctorque,
                                 modA,
                                 modB):
        bodyA = chrono.CastToChBody(modA)
        bodyB = chrono.CastToChBody(modB)
        if (bodyA == self.m_box) :
            print("       ", pA.x, pA.y, pA.z)
        elif (bodyB == self.m_box) :
            print("       ", pB.x, pB.y, pB.z)
        
        return True


# -----------------------------------------------------------------------------
# Callback class for modifying composite material
# -----------------------------------------------------------------------------
class ContactMaterial(chrono.AddContactCallback):
    def __init__(self):
        super().__init__()
    def OnAddContact(         self,
                              contactinfo,
                              material):
        # Downcast to appropriate composite material type
        mat = chrono.CastToChContactMaterialCompositeNSC(material)

        # Set different friction for left/right halfs
        if (contactinfo.vpA.z > 0) :
            friction =  0.3
        else: 
            friction =  0.8
        mat.static_friction = friction
        mat.sliding_friction = friction


print( "Copyright (c) 2017 projectchrono.org")

# ----------------
# Parameters
# ----------------

friction = 0.6
collision_envelope = .001

# -----------------
# Create the sys
# -----------------

sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -10, 0))

# Set solver settings
sys.GetSolver().AsIterative().SetMaxIterations(100)
sys.GetSolver().AsIterative().SetTolerance(0)
sys.SetMaxPenetrationRecoverySpeed(1e8)

# --------------------------------------------------
# Create a contact material, shared among all bodies
# --------------------------------------------------

material = chrono.ChContactMaterialNSC()
material.SetFriction(friction)

# ----------
# Add bodies
# ----------

container = chrono.ChBody()
sys.Add(container)
container.SetPos(chrono.ChVector3d(0, 0, 0))
container.SetFixed(True)

container.EnableCollision(True)
chrono.AddBoxGeometry(container, material, chrono.ChVector3d(8, 1, 8), chrono.ChVector3d(0, -0.5, 0))
container.GetCollisionModel().SetEnvelope(collision_envelope)

container.GetVisualShape(0).SetColor(chrono.ChColor(0.4, 0.4, 0.4))

box1 = chrono.ChBody()
box1.SetMass(10)
box1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
box1.SetPos(chrono.ChVector3d(-1, 0.21, -1))
box1.SetPosDt(chrono.ChVector3d(5, 0, 0))

box1.EnableCollision(True)
chrono.AddBoxGeometry(box1, material, chrono.ChVector3d(0.4, 0.2, 0.1))
box1.GetCollisionModel().SetEnvelope(collision_envelope)

box1.GetVisualShape(0).SetColor(chrono.ChColor(0.1, 0.1, 0.4))

sys.AddBody(box1)

box2 = chrono.ChBody()
box2.SetMass(10)
box2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
box2.SetPos(chrono.ChVector3d(-1, 0.21, +1))
box2.SetPosDt(chrono.ChVector3d(5, 0, 0))

box2.EnableCollision(True)
chrono.AddBoxGeometry(box2, material, chrono.ChVector3d(0.4, 0.2, 0.1))
box2.GetCollisionModel().SetEnvelope(collision_envelope)

box2.GetVisualShape(0).SetColor(chrono.ChColor(0.4, 0.1, 0.1))

sys.AddBody(box2)

# -------------------------------
# Create the visualization window
# -------------------------------

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('NSC callbacks')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4, 4, -6))
vis.AddTypicalLights()

# ---------------
# Simulate sys
# ---------------

creporter = ContactReporter(box1)

cmaterial = ContactMaterial()
sys.GetContactContainer().RegisterAddContactCallback(cmaterial)

while vis.Run():
    vis.BeginScene() 
    vis.Render()
    chronoirr.drawGrid(vis, 0.5, 0.5, 12, 12,
                       chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI_2)))
    chronoirr.drawAllCOGs(vis, 1.0)
    vis.EndScene()
    sys.DoStepDynamics(1e-3)
    
    # Process contacts
    print(str(sys.GetChTime() ) + "  "  + str(sys.GetNumContacts()) )
    sys.GetContactContainer().ReportAllContacts(creporter)
    

