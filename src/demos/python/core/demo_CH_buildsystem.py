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

print ("Second tutorial: create and populate a physical system");


# Load the Chrono::Engine core module!
import pychrono as chrono


# Create a physical system,
my_system = chrono.ChSystemNSC()

# Create a contact material, shared by all collision shapes
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.3)
material.SetCompliance(0)

# Add two bodies
bodyA = chrono.ChBody()
bodyA.SetMass(20)
bodyA.SetName('BodyA')
bodyA.SetInertiaXX( chrono.ChVectorD(10,10,10) )
print (bodyA.GetInertia() )
bodyA.SetPos(chrono.ChVectorD(1,-1,0))
bodyA.GetCollisionModel().AddBox(material,10,1,10)
bodyA.SetBodyFixed(True)
bodyA.SetCollide(True)

bodyB = chrono.ChBody()
bodyB.SetName('BodyB')
bodyB.SetPos(chrono.ChVectorD(0,2,0))
bodyB.GetCollisionModel().AddBox(material,1,1,1)
bodyB.SetCollide(True)

markerB = chrono.ChMarker()
my_funct = chrono.ChFunction_Sine(0,0.5,3)
markerB.SetMotion_X(my_funct)
markerB.SetPos(chrono.ChVectorD(1,2,3))
bodyB.AddMarker(markerB)

my_system.Add(bodyA)
my_system.Add(bodyB)


# Report Contact callback
class MyReportContactCallback(chrono.ReportContactCallback):
    def __init__(self):
         chrono.ReportContactCallback.__init__(self)
    def OnReportContact(self,vA,vB,cA,dist,rad,force,torque,modA,modB):
         bodyUpA = chrono.CastContactableToChBody(modA)
         nameA = bodyUpA.GetName()
         bodyUpB = chrono.CastContactableToChBody(modB)
         nameB = bodyUpB.GetName()
         print ('  contact: point A=' , vA,  '  dist=', dist, 'first body:', nameA, 'second body:', nameB)
         return True        # return False to stop reporting contacts

my_rep = MyReportContactCallback()



# Simulation loop
my_system.SetChTime(0)
while (my_system.GetChTime() < 1.2) :

    my_system.DoStepDynamics(0.01)

    print ('time=', my_system.GetChTime(), ' bodyB y=', bodyB.GetPos().y)

    my_system.GetContactContainer().ReportAllContacts(my_rep)

print('----------')

# Iterate over added bodies (Python style)
print ('Positions of all bodies in the system:')
for abody in my_system.Get_bodylist():
    print (' ', abody.GetName(), ' pos =', abody.GetPos() )

# Use a body with an auxiliary reference (REF) that does not correspond to the center of gravity (COG)
bodyC = chrono.ChBodyAuxRef()
my_system.AddBody(bodyC)
bodyC.SetName('Parte1-1')
bodyC.SetPos(chrono.ChVectorD(-0.0445347481124079,0.0676266363930238,-0.0230808979433518))
bodyC.SetRot(chrono.ChQuaternionD(1,0,0,0))
bodyC.SetMass(346.17080777653)
bodyC.SetInertiaXX(chrono.ChVectorD(48583.2418823358,526927.118351673,490689.966726565))
bodyC.SetInertiaXY(chrono.ChVectorD(1.70380722975012e-11,1.40840344485366e-11,-2.31869065456271e-12))
bodyC.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(68.9923703887577,-60.1266363930238,70.1327223302498),chrono.ChQuaternionD(1,0,0,0)))
myasset = chrono.ChModelFileShape()
myasset.SetFilename("shapes/test.obj")
bodyC.AddVisualShape(myasset)

# Add a revolute joint 
rev = chrono.ChLinkLockRevolute()
rev.SetName('Revolute')
rev.Initialize(bodyA, bodyC, chrono.ChCoordsysD(chrono.ChVectorD(1, 2, 3), chrono.ChQuaternionD(1, 0, 0, 0)))
my_system.AddLink(rev)

# Iterate over added links (Python style)
print ('Names of all links in the system:')
for alink in my_system.Get_linklist():
    print ('  link: ', alink.GetName() )


print ('Done...')
