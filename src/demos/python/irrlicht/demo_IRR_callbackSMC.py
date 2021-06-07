# =============================================================================
# PROJECT CHRONO - http:#projectchrono.org
#
# Copyright (c) 2014 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http:#projectchrono.org/license-chrono.txt.
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
        mat = chrono.CastToChMaterialCompositeSMC(material)

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
# Create the system
# -----------------

system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -10, 0))

# Set solver settings
system.SetSolverMaxIterations(100)
system.SetSolverForceTolerance(0)

# --------------------------------------------------
# Create a contact material, shared among all bodies
# --------------------------------------------------

material = chrono.ChMaterialSurfaceSMC()
material.SetFriction(friction)

# ----------
# Add bodies
# ----------

container = chrono.ChBody()
system.Add(container)
container.SetPos(chrono.ChVectorD(0, 0, 0))
container.SetBodyFixed(True)
container.SetIdentifier(-1)

container.SetCollide(True)
container.GetCollisionModel().ClearModel()
chrono.AddBoxGeometry(container, material, chrono.ChVectorD(4, 0.5, 4), chrono.ChVectorD(0, -0.5, 0))
container.GetCollisionModel().BuildModel()

container.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.4, 0.4, 0.4)))

box1 = chrono.ChBody()
box1.SetMass(10)
box1.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
box1.SetPos(chrono.ChVectorD(-1, 0.21, -1))
box1.SetPos_dt(chrono.ChVectorD(5, 0, 0))

box1.SetCollide(True)
box1.GetCollisionModel().ClearModel()
chrono.AddBoxGeometry(box1, material, chrono.ChVectorD(0.4, 0.2, 0.1))
box1.GetCollisionModel().BuildModel()

box1.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.1, 0.1, 0.4)))

system.AddBody(box1)

box2 = chrono.ChBody(system.NewBody())
box2.SetMass(10)
box2.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
box2.SetPos(chrono.ChVectorD(-1, 0.21, +1))
box2.SetPos_dt(chrono.ChVectorD(5, 0, 0))

box2.SetCollide(True)
box2.GetCollisionModel().ClearModel()
chrono.AddBoxGeometry(box2, material, chrono.ChVectorD(0.4, 0.2, 0.1))
box2.GetCollisionModel().BuildModel()

box2.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.4, 0.1, 0.1)))

system.AddBody(box2)

# -------------------------------
# Create the visualization window
# -------------------------------

application = chronoirr.ChIrrApp(system, "SMC callbacks", chronoirr.dimension2du(800, 600))
application.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(4, 4, -6))

application.AssetBindAll()
application.AssetUpdateAll()

# ---------------
# Simulate system
# ---------------

creporter = ContactReporter(box1, box2)

cmaterial = ContactMaterial()
system.GetContactContainer().RegisterAddContactCallback(cmaterial)

application.SetTimestep(1e-3)

while (application.GetDevice().run()) :
    application.BeginScene(True, True, chronoirr.SColor(255, 140, 161, 192))
    application.DrawAll()
    chronoirr.drawGrid(application.GetVideoDriver(), 0.5, 0.5, 12, 12,
                       chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)))
    chronoirr.drawAllCOGs(system, application.GetVideoDriver(), 1.0)
    application.DoStep()
    application.EndScene()
    
    # Process contacts
    print(str(system.GetChTime() ) + "  "  + str(system.GetNcontacts()) )
    system.GetContactContainer().ReportAllContacts(creporter)
    
    # Cumulative contact force and torque on boxes (as applied to COM)
    frc1 = box1.GetContactForce()
    trq1 = box1.GetContactTorque()
    print("  Box 1 contact force at COM:    ", frc1.x, frc1.y, frc1.z)
    print("  contact torque at COM:    ", trq1.x, trq1.y, trq1.z)
    frc2 = box2.GetContactForce()
    trq2 = box2.GetContactTorque()
    print("  Box 2 contact force at COM:    ", frc2.x, frc2.y, frc2.z)
    print("  contact torque at COM:    ", trq2.x, trq2.y, trq2.z)
    

