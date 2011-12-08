#-------------------------------------------------------------------------------
# Name:        demo_python_2
#-------------------------------------------------------------------------------
#!/usr/bin/env python

def main():
    pass

if __name__ == '__main__':
    main()


# Load the Chrono::Engine unit!!!
import ChronoEngine_PYTHON_core as chrono

# Create a physical system,
my_system = chrono.ChSystem()

# Add two bodies
my_shbodyA = chrono.ChBodyShared()
my_shbodyA.SetMass(20)
my_shbodyA.SetInertiaXX( chrono.ChVectorD(10,10,10) )
print my_shbodyA.GetXInertia()
my_shbodyA.SetPos(chrono.ChVectorD(1,-1,0))
my_shbodyA.GetCollisionModel().AddBox(10,1,10)
my_shbodyA.SetBodyFixed(1)
my_shbodyA.SetCollide(1)

my_shbodyB = chrono.ChBodyShared()
my_shbodyB.SetPos(chrono.ChVectorD(0,2,0))
my_shbodyB.GetCollisionModel().AddBox(1,1,1)
my_shbodyB.SetCollide(1)

my_shmarker = chrono.ChMarkerShared()
my_shmarker.SetPos(chrono.ChVectorD(1,2,3))
my_shbodyB.AddMarker(my_shmarker)

my_system.Add(my_shbodyA)
my_system.Add(my_shbodyB)

my_shlink = chrono.ChLinkEngineShared()
my_system.Add(my_shlink)


# Define surface material(s)
my_shmaterial = chrono.ChMaterialSurfaceShared()
my_shmaterial.SetFriction(0.3)
my_shmaterial.SetCompliance(1e-6)
my_shbodyA.SetMaterialSurface(my_shmaterial)
my_shbodyB.SetMaterialSurface(my_shmaterial)


#Contact callback (TO FIX)
class MyContactCallback(chrono.ChAddContactCallback):
    def __init__(self):
         chrono.ChAddContactCallback.__init__(self)
    def ContactCallback(self,collinfo,matcouple):
         print 'contact default friction=' , matcouple.static_friction

my_call = MyContactCallback()
my_system.GetContactContainer().SetAddContactCallback(my_call)


# Simulation loop
while (my_system.GetChTime() < 1.2) :
    my_system.DoStepDynamics(0.01)
    print 'time=', my_system.GetChTime(), ' bodyB y=', my_shbodyB.GetPos().y


# Iterate over added bodies - how to use iterators
print 'This is the list of bodies in the system:'
iterbodies = my_system.IterBeginBodies()
while (iterbodies !=  my_system.IterEndBodies()):
    print '  body pos=', iterbodies.Ref().GetPos()
    iterbodies = iterbodies.Next()

# Easier (but a bit slower) iteration in the style of Python:
for abody in chrono.IterBodies(my_system):
    print '  body pos=', abody.GetPos()

# Also iterate on links, Python style:
for alink in chrono.IterLinks(my_system):
    print '  link: ', alink


# Move a body, using a ChFrame
my_displacement = chrono.ChFrameMovingD(chrono.ChVectorD(5,1,0));
my_shbodyA %= my_displacement
# ..also as:
#  my_shbody.ConcatenatePreTransformation(my_displacement)

print '  body pos=', my_shbodyA.GetPos()
