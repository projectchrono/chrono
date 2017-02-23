#-------------------------------------------------------------------------------
# Name:        demo_python_2
#-------------------------------------------------------------------------------
#!/usr/bin/env python

def main():
    pass

if __name__ == '__main__':
    main()


# Load the Chrono::Engine unit!!!
import ChronoEngine_python_core as chrono


# Create a physical system,
my_system = chrono.ChSystem()

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
my_shmaterial = chrono.ChMaterialSurface()
my_shmaterial.SetFriction(0.3)
my_shmaterial.SetCompliance(0)
my_shbodyA.SetMaterialSurface(my_shmaterial)
my_shbodyB.SetMaterialSurface(my_shmaterial)


# Add Contact callback (TO FIX!!)
##class MyContactCallback(chrono.ChCustomCollisionPointCallbackP):
##    def __init__(self):
##         chrono.ChCustomCollisionPointCallbackP.__init__(self)
##    def ContactCallback(self,collinfo,matcouple):
##         print (' add contact: ' , collinfo.distance, matcouple.static_friction)
##
##my_call = MyContactCallback()
##my_system.SetCustomCollisionPointCallback(my_call)

# Report Contact callback
class MyReportContactCallback(chrono.ChReportContactCallback):
    def __init__(self):
         chrono.ChReportContactCallback.__init__(self)
    def ReportContactCallback(self,vA,vB,cA,dist,force,torque,modA,modB):
         print ('  contact: point A=' , vA,  '  dist=',dist)
         return True        # return False to stop reporting contacts

my_rep = MyReportContactCallback()



# Simulation loop
my_system.SetChTime(0)
while (my_system.GetChTime() < 1.2) :

    my_system.DoStepDynamics(0.01)

    print ('time=', my_system.GetChTime(), ' bodyB y=', my_shbodyB.GetPos().y)

    my_system.GetContactContainer().ReportAllContacts(my_rep)


# Iterate over added bodies - how to use iterators
print ('This is the list of bodies in the system:')
iterbodies = my_system.IterBeginBodies()
while (iterbodies !=  my_system.IterEndBodies()):
    print ('  body pos=', iterbodies.Ref().GetPos() )
    iterbodies = iterbodies.Next()

# Easier (but a bit slower) iteration in the style of Python:
print ('This is the list of bodies in the system:')
for abody in chrono.IterBodies(my_system):
    print ('  body pos=', abody.GetPos() )

# Also iterate on links, Python style:
for alink in chrono.IterLinks(my_system):
    print ('  link: ', alink )



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








