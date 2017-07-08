#-------------------------------------------------------------------------------
# Name:        demo_python_3
#
# This file shows how to use POV ray for postprocessing, thanks to the
# utility functions in the unit_POSTPROCESS of Chrono::Engine.
#
# Note. Since this file requires a template file ( "_template_POV.pov" in the
# bin/data/ directory) whose position is set with a relative path, please
# make sure that the current directory of Python is the one where this demo
# resides, otherwise if you launch it from another directory it does not
# find the POV template.
#
#-------------------------------------------------------------------------------
#!/usr/bin/env python

def main():
    pass

if __name__ == '__main__':
    main()


# Load the Chrono::Engine unit and the postprocessing unit!!!
import ChronoEngine_python_core as chrono
import ChronoEngine_python_postprocess as postprocess

# We will create two directories for saving some files, we need this:
import os


# Create a physical system,
my_system = chrono.ChSystemNSC()
my_systemB = my_system
my_system.SetTol(2)
print (my_systemB.GetTol())

# Create a body
body_1= chrono.ChBodyAuxRef()
my_system.Add(body_1)

# Attach a visualization asset to the body (a Wavefront .obj mesh)
myasset = chrono.ChSphereShape()
myasset.GetSphereGeometry().rad =0.2
body_1.GetAssets().push_back(myasset)

# Assets can be shared, ex. to save memory...
body_2= chrono.ChBodyAuxRef()
body_2.SetPos(chrono.ChVectorD(0.5,0,0))
my_system.Add(body_2)
body_2.GetAssets().push_back(myasset)

#
# Create an exporter to POVray !!!
#

pov_exporter = postprocess.ChPovRay(my_system)

 # Sets some file names for in-out processes.
pov_exporter.SetTemplateFile        ("../../../data/_template_POV.pov")
pov_exporter.SetOutputScriptFile    ("rendering_frames.pov")
pov_exporter.SetOutputDataFilebase  ("my_state")
pov_exporter.SetPictureFilebase     ("picture")
 # Save the .dat files and the .bmp files in two subdirectories,
 # to avoid cluttering the current directory...
if not os.path.exists("output"):
    os.mkdir("output")
if not os.path.exists("anim"):
    os.mkdir("anim")
pov_exporter.SetOutputDataFilebase("output/my_state")
pov_exporter.SetPictureFilebase("anim/picture")

 # Tell selectively which physical items you want to render, or use AddAll()
pov_exporter.Add(body_1)
pov_exporter.Add(body_2)


 # 1) Create the two .pov and .ini files for POV-Ray (this must be done
 #    only once at the beginning of the simulation).
pov_exporter.ExportScript()

 # Perform a short simulation
while (my_system.GetChTime() < 0.2) :

    my_system.DoStepDynamics(0.01)

    print ('time=', my_system.GetChTime() )

    # 2) Create the incremental nnnn.dat and nnnn.pov files that will be load
    #    by the pov .ini script in POV-Ray (do this at each simulation timestep)
    pov_exporter.ExportData()



# That's all! If all worked ok, this python script should
# have created a  "rendering_frames.pov.ini"  file that you can
# load in POV-Ray, then when you press 'RUN' you will see that
# POV-Ray will start rendering a short animation, saving the frames
# in the directory 'anim'.


