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
# Chrono demonstration of using contact callbacks for smooth contacts
# (penalty-based).
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
    def __init__(self, box1, box2) : 
        self.m_box1 = box1
        self.m_box2 = box2
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
        frc = plane_coord * cforce;
        bodyA = chrono.CastToChBody(modA)
        bodyB = chrono.CastToChBody(modB)
        if (bodyA == self.m_box1) :
            print("   contact on Box 1 at pos:    ", pA.x, pA.y, pA.z)
            print("   frc:                        ", frc.x, frc.y, frc.z)
        elif (bodyB == self.m_box1) :
            print("   contact on Box 1 at pos:    ", pB.x, pB.y, pB.z)
            print("   frc:                        ", frc.x, frc.y, frc.z)
            
        if (bodyA == self.m_box2) :
            print("   contact on Box 2 at pos:    ", pA.x, pA.y, pA.z)
            print("   frc:                        ", frc.x, frc.y, frc.z)
        elif (bodyB == self.m_box2) :
            print("   contact on Box 2 at pos:    ", pB.x, pB.y, pB.z)
            print("   frc:                         ", frc.x, frc.y, frc.z)
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
        mat = chrono.CastToChContactMaterialCompositeSMC(material)

        # Set different friction for left/right halfs
        if (contactinfo.vpA.z > 0) :
            friction =  0.3
        else: 
            friction =  0.8
        mat.mu_eff = friction


print( "Copyright (c) 2017 projectchrono.org")

# ----------------
# Parameters
# ----------------

friction = 0.6

# -----------------
# Create the sys
# -----------------

sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -10, 0))

# Set solver settings
sys.GetSolver().AsIterative().SetMaxIterations(100)
sys.GetSolver().AsIterative().SetTolerance(0)

# --------------------------------------------------
# Create a contact material, shared among all bodies
# --------------------------------------------------

material = chrono.ChContactMaterialSMC()
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

container.GetVisualShape(0).SetColor(chrono.ChColor(0.4, 0.4, 0.4))

box1 = chrono.ChBody()
box1.SetMass(10)
box1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
box1.SetPos(chrono.ChVector3d(-1, 0.21, -1))
box1.SetPosDt(chrono.ChVector3d(5, 0, 0))

box1.EnableCollision(True)
chrono.AddBoxGeometry(box1, material, chrono.ChVector3d(0.4, 0.2, 0.1))

box1.GetVisualShape(0).SetColor(chrono.ChColor(0.1, 0.1, 0.4))

sys.AddBody(box1)

box2 = chrono.ChBody()
box2.SetMass(10)
box2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
box2.SetPos(chrono.ChVector3d(-1, 0.21, +1))
box2.SetPosDt(chrono.ChVector3d(5, 0, 0))

box2.EnableCollision(True)
chrono.AddBoxGeometry(box2, material, chrono.ChVector3d(0.4, 0.2, 0.1))

box2.GetVisualShape(0).SetColor(chrono.ChColor(0.4, 0.1, 0.1))

sys.AddBody(box2)

# -------------------------------
# Create the visualization window
# -------------------------------

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('SMC callbacks')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4, 4, -6))
vis.AddTypicalLights()

# ---------------
# Simulate sys
# ---------------

creporter = ContactReporter(box1, box2)

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
    
    # Cumulative contact force and torque on boxes (as applied to COM)
    frc1 = box1.GetContactForce()
    trq1 = box1.GetContactTorque()
    print("  Box 1 contact force at COM:    ", frc1.x, frc1.y, frc1.z)
    print("  contact torque at COM:    ", trq1.x, trq1.y, trq1.z)
    frc2 = box2.GetContactForce()
    trq2 = box2.GetContactTorque()
    print("  Box 2 contact force at COM:    ", frc2.x, frc2.y, frc2.z)
    print("  contact torque at COM:    ", trq2.x, trq2.y, trq2.z)
    

