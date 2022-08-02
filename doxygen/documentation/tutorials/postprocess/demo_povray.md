Export POVray animations (demo_POST_povray.cpp)  {#tutorial_demo_povray}
==========================


Tutorial that teaches how to use the [POSTPROCESS module](group__postprocess__module.html) to create animations with [POVray](http://www.povray.org). 

When the simulation is run, a set of .pov and .ini files are saved on disk, so that one can use POVray later to do high-quality rendering of simulations.

- Learn how to attach @ref chrono::ChVisualShape to rigid bodies.
- Learn how to output data for POVray. 

Note: the same assets can be used to render animations in real-time in the interactive 3D view of Irrlicht, as explained in demo_IRR_assets.cpp


# Example 1

Create a @ref chrono::ChBody, and attach some 'assets' that define 3D shapes. 
These shapes can be shown by Irrlicht, OpenGL, or POV postprocessing.
Note: these assets are independent from collision shapes.

\snippet demo_POST_povray.cpp Example 1


# Example 2

Textures, colors, asset levels with transformations.
This section shows how to add more advanced types of assets.

\snippet demo_POST_povray.cpp Example 2


# Example 3

Create a @ref chrono::ChParticleCloud cluster, and attach 'assets' that define a 
single "sample" 3D shape. This will be shown N times in POV or Irrlicht.
	
\snippet demo_POST_povray.cpp Example 3


# The POV exporter

The following part is very important because this is what makes 
this demo different from the demo_IRR_assets, that used Irrlicht. 
We need to create a postprocessor of type @ref chrono::postprocess::ChPovRay and tell him that 
we are going to export our visualization assets:

\snippet demo_POST_povray.cpp POV exporter

# The simulation loop

Now you have to write the usual while() loop to perform the simulation. 
Note that before running the loop you need to use  pov_exporter.ExportScript(); , 
and for each timestep you must use pov_exporter.ExportData(); 
actually this is the instruction that creates the many .dat and .pov files in the output directory.

\snippet demo_POST_povray.cpp POV simulation

# Executing and rendering with POVray

Once you created your program, compile it, then:

- execute the `demo_POST_povray.exe`
- on the console you will see a time counter showing that the system is load and it is being simulated
- when the program ends, you must open POVray and open the `rendering_frames.pov.ini` 
  file, using the Open menu or button, or drag&drop (you can find this .ini file and 
  other POVray as they are saved in the same directory of the executable) 

![](http://projectchrono.org/assets/manual/Povray.jpg)

- press the Run button in POVray to execute the .ini file , and you should see that 
  POVray generates lot of frames, being saved in the directory `anim`.

![](http://projectchrono.org/assets/manual/Tutorial_pov.jpg)


# Optional encoding into an AVI or MPEG animation

If you want to generate a .mpeg or .avi animation from the rendered .bmp images, 
we suggest to use the VirtualDub tool:

![](http://projectchrono.org/assets/manual/Tutorial_pov2.jpg)

- drag&drop the first .jpg frame in its interface; it will automatically load all 
  other frames in the timeline
- use menu Video/Compression... to setup the proper video codec (suggested: Xvid, DivX, mpeg4, etc.)
- use menu File/Save As Avi... to encode and save the animation on disk.


# Listing

In the following we report the entire source code for reference.

\include demo_POST_povray.cpp

