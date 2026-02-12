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

import pychrono as chrono
import pychrono.postprocess as postprocess
import os

# Set output root directory
chrono.SetChronoOutputPath("../DEMO_OUTPUT/")

# Create the output directory
out_dir = chrono.GetChronoOutputPath() + "Povray1/"
try:
    os.mkdir(out_dir)
except OSError as exc:
    if exc.errno != errno.EEXIST:
       print("Error creating output directory " )

# Create a physical system,
my_system = chrono.ChSystemNSC()
my_system.SetGravityY()
my_systemB = my_system

# Create a body
body_1= chrono.ChBodyAuxRef()
my_system.Add(body_1)

# Attach a visualization asset to the body (ex.: a sphere)
myasset = chrono.ChVisualShapeSphere(0.2)
body_1.AddVisualShape(myasset)


# Assets can be shared, ex. to save memory...
body_2= chrono.ChBodyAuxRef()
body_2.SetPos(chrono.ChVector3d(0.5,0,0))
my_system.Add(body_2)
body_2.AddVisualShape(myasset)

# Create an exporter to POVray
print(chrono.GetChronoDataFile("POVRay_chrono_template.pov"))

pov_exporter = postprocess.ChPovRay(my_system)

# Important: set the location of the template
# This path depends to where you execute this script.
# Here we assume you run it from src/demo/python/postprocess/
pov_exporter.SetTemplateFile(chrono.GetChronoDataFile("POVRay_chrono_template.pov"))

# Set the path where it will save all .pov, .ini, .asset and .dat files
pov_exporter.SetBasePath(out_dir)

# Selectively add physical items to be rendered (or use AddAll)
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


