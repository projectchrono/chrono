#------------------------------------------------------------------------------
# Name:        pychrono example
# Purpose:
#
# Author:      Alessandro Tasora
#
# Created:     1/01/2019
# Copyright:   (c) ProjectChrono 2019
#------------------------------------------------------------------------------

print ("Second tutorial: create and populate a physical system");


# Load the Chrono::Engine core module!
import pychrono as chrono


# Create a physical system,
my_system = chrono.ChSystemNSC()

# Add two bodies
my_shbodyA = chrono.ChBody()
my_shbodyA.SetMass(20)
my_shbodyA.SetInertiaXX( chrono.ChVectorD(10,10,10) )
print (my_shbodyA.GetInertia() )
my_shbodyA.SetPos(chrono.ChVectorD(1,-1,0))
my_shbodyA.GetCollisionModel().AddBox(10,1,10)
my_shbodyA.SetBodyFixed(True)
my_shbodyA.SetCollide(True)

my_shbodyB = chrono.ChBody()
my_shbodyB.SetPos(chrono.ChVectorD(0,2,0))
my_shbodyB.GetCollisionModel().AddBox(1,1,1)
my_shbodyB.SetCollide(True)

my_shmarker = chrono.ChMarker()
my_funct = chrono.ChFunction_Sine(0,0.5,3)
my_shmarker.SetMotion_X(my_funct)
my_shmarker.SetPos(chrono.ChVectorD(1,2,3))
my_shbodyB.AddMarker(my_shmarker)

my_system.Add(my_shbodyA)
my_system.Add(my_shbodyB)

# Define surface material(s)
my_shmaterial = chrono.ChMaterialSurfaceNSC()
my_shmaterial.SetFriction(0.3)
my_shmaterial.SetCompliance(0)
my_shbodyA.SetMaterialSurface(my_shmaterial)
my_shbodyB.SetMaterialSurface(my_shmaterial)


# Add Contact callback (TO FIX downcast matcouple to ChMaterialCompositeSMC !!)
class MyContactCallback(chrono.ChAddContactCallbackP):
    def __init__(self):
         chrono.ChAddContactCallbackP.__init__(self)
    def OnAddContact(self,
                     collinfo, # get infos as ChCollisionInfo 
                     matcouple # change values here if needed, as ChMaterialComposite
                     ):
         print (' add contact: distance=' , collinfo, matcouple)
         #matcouple.static_friction = 0.001  # change the default friction from srface materials
         
my_on_add_contact = MyContactCallback()
my_system.GetContactContainer().RegisterAddContactCallback(my_on_add_contact)


# Report Contact callback
class MyReportContactCallback(chrono.ChReportContactCallbackP):
    def __init__(self):
         chrono.ChReportContactCallbackP.__init__(self)
    def OnReportContact(self,
                        pA,             # point A as ChVectorD
                        pB,             # point B as ChVectorD
                        plane_coord,    # plane coords ChMatrix33D
                        distance,       # distance
                        eff_radius,     # surf. radius 
                        react_forces,   # as ChVectorD
                        react_torques,  # as ChVectorD
                        contactobjA,    # contacted object as ChContactable
                        contactobjB):   # contacted object as ChContactable
         print ('  contact: point A=' , pA,  '  dist=',distance)
         return True        # return False to stop reporting contacts

my_rep = MyReportContactCallback()



# Simulation loop
my_system.SetChTime(0)
while (my_system.GetChTime() < 1.2) :

    my_system.DoStepDynamics(0.01)

    print ('time=', my_system.GetChTime(), ' bodyB y=', my_shbodyB.GetPos().y)

    my_system.GetContactContainer().ReportAllContacts(my_rep)


# Iterate over added bodies (Python style)
print ('Positions of all bodies in the system:')
for abody in my_system.Get_bodylist():
    print ('  body pos=', abody.GetPos() )


# Move a body, using a ChFrame
my_displacement = chrono.ChFrameMovingD(chrono.ChVectorD(5,1,0));
my_shbodyA %= my_displacement
# ..also as:
#  my_shbody.ConcatenatePreTransformation(my_displacement)

print ('Moved body pos=', my_shbodyA.GetPos() )


# Use a body with an auxiliary reference (REF) that does not correspond
# to the center of gravity (COG)
body_1= chrono.ChBodyAuxRef()
body_1.SetName('Parte1-1')
body_1.SetPos(chrono.ChVectorD(-0.0445347481124079,0.0676266363930238,-0.0230808979433518))
body_1.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_1.SetMass(346.17080777653)
body_1.SetInertiaXX(chrono.ChVectorD(48583.2418823358,526927.118351673,490689.966726565))
body_1.SetInertiaXY(chrono.ChVectorD(1.70380722975012e-11,1.40840344485366e-11,-2.31869065456271e-12))
body_1.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(68.9923703887577,-60.1266363930238,70.1327223302498),chrono.ChQuaternionD(1,0,0,0)))
myasset = chrono.ChObjShapeFile()
myasset.SetFilename("shapes/test.obj")
body_1.GetAssets().push_back(myasset)

print ('Done...')
