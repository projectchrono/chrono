Export POVray animations (demo_postprocess.cpp)  {#tutorial_demo_postprocess}
==========================


Tutorial that teaches how to use the 
[POSTPROCESSING module](@ref module_postprocessing_installation)
to create animations with [POVray](http://www.povray.org). 

When the simulation is run, a set of .pov and .ini files 
are saved on disk, so that one can use POVray 
later to do high-quality rendering of simulations.

- Learn how to attach ChAsset shapes to rigid bodies.
- Learn how to output data for POVray. 

Note: the same assets can be used to render animations 
in real-time in the interactive 3D view of Irrlicht, 
as explained in demo_irr_assets.cpp


# Example 1

Create a @ref ChBody, and attach some 'assets' that define 3D shapes. 
These shapes can be shown by Irrlicht or POV postprocessing, etc...
Note: these assets are independent from collision shapes!

~~~{.cpp}
			// Create a rigid body as usual, and add it 
			// to the physical system:
	ChSharedPtr<ChBody> mfloor(new ChBody);
	mfloor->SetBodyFixed(true);

			// Define a collision shape 
	mfloor->GetCollisionModel()->ClearModel();
	mfloor->GetCollisionModel()->AddBox(10,0.5,10, ChVector<>(0,-1,0));
	mfloor->GetCollisionModel()->BuildModel();
	mfloor->SetCollide(true);

			// Add body to system
	mphysicalSystem.Add(mfloor);

			// ==Asset== attach a 'box' shape.
			// Note that assets are managed via shared pointer, so they 
			// can also be shared). Do not forget AddAsset() at the end!
	ChSharedPtr<ChBoxShape> mboxfloor(new ChBoxShape);
	mboxfloor->GetBoxGeometry().Pos = ChVector<>(0,-1,0);
	mboxfloor->GetBoxGeometry().Size = ChVector<>(10,0.5,10);
	mfloor->AddAsset(mboxfloor);

			// ==Asset== attach color asset. 
	ChSharedPtr<ChVisualization> mfloorcolor(new ChVisualization);
	mfloorcolor->SetColor(ChColor(0.3,0.3,0.6));
	mfloor->AddAsset(mfloorcolor);
~~~


# Example 2

Textures, colors, asset levels with transformations.
This section shows how to add more advanced types of assets and 
how to group assets in @ref ChAssetLevel containers.

~~~{.cpp}
			// Create the rigid body as usual (this won't move,
			// it is only for visualization tests)
	ChSharedPtr<ChBody> mbody(new ChBody);
	mbody->SetBodyFixed(true);
	mphysicalSystem.Add(mbody);

			// ==Asset== Attach a 'sphere' shape  
	ChSharedPtr<ChSphereShape> msphere(new ChSphereShape);
	msphere->GetSphereGeometry().rad = 0.5;
	msphere->GetSphereGeometry().center = ChVector<>(-1,0,0);
	mbody->AddAsset(msphere);

			// ==Asset== Attach also a 'box' shape 
	ChSharedPtr<ChBoxShape> mbox(new ChBoxShape);
	mbox->GetBoxGeometry().Pos = ChVector<>(1,0,0);
	mbox->GetBoxGeometry().Size = ChVector<>(0.2,0.5,0.1);
	mbody->AddAsset(mbox);

			// ==Asset== Attach also a 'cylinder' shape 
	ChSharedPtr<ChCylinderShape> mcyl(new ChCylinderShape);
	mcyl->GetCylinderGeometry().p1  = ChVector<>(2,-0.2,0);
	mcyl->GetCylinderGeometry().p2  = ChVector<>(2.2,0.5,0);
	mcyl->GetCylinderGeometry().rad = 0.3;
	mbody->AddAsset(mcyl);

			// ==Asset== Attach color. To set colors for all assets  
			// in the same level, just add this:
	ChSharedPtr<ChVisualization> mvisual(new ChVisualization);
	mvisual->SetColor(ChColor(0.9,0.4,0.2));
	mbody->AddAsset(mvisual);


			// ==Asset== Attach a level that contains other assets.
			// Note: a ChAssetLevel can define a rotation/translation respect to paren level,
			// Note: a ChAssetLevel can contain colors or textures: if any, they affect only objects in the level.
	ChSharedPtr<ChAssetLevel> mlevelA(new ChAssetLevel);

				// ==Asset== Attach, in this level, a 'Wavefront mesh' asset, 
				// referencing a .obj file:
	ChSharedPtr<ChObjShapeFile> mobjmesh(new ChObjShapeFile);
	mobjmesh->SetFilename("../data/forklift_body.obj"); 
	mlevelA->AddAsset(mobjmesh);

				// ==Asset== Attach also a texture, that will affect only the 
				// assets in mlevelA:
	ChSharedPtr<ChTexture> mtexture(new ChTexture);
	mtexture->SetTextureFilename("../data/bluwhite.png");
	mlevelA->AddAsset(mtexture);
	
			// Change the position of mlevelA, thus moving also its sub-assets:
	mlevelA->GetFrame().SetPos(ChVector<>(0,0,2));
	mbody->AddAsset(mlevelA);

			// ==Asset== Attach sub level, then add to it an array of sub-levels, 
			// each rotated, and each containing a displaced box, thus making a 
			// spiral of cubes
	ChSharedPtr<ChAssetLevel> mlevelB(new ChAssetLevel);
	for (int j = 0; j<20; j++)
	{
				// ==Asset== the sub sub level..
		ChSharedPtr<ChAssetLevel> mlevelC(new ChAssetLevel);

					// ==Asset== the contained box..
		ChSharedPtr<ChBoxShape> msmallbox(new ChBoxShape);
		msmallbox->GetBoxGeometry().Pos = ChVector<>(0.4,0,0);
		msmallbox->GetBoxGeometry().Size = ChVector<>(0.1,0.1,0.01);
		mlevelC->AddAsset(msmallbox);

		ChQuaternion<> mrot;
		mrot.Q_from_AngAxis(j*21*CH_C_DEG_TO_RAD, ChVector<>(0,1,0));
		mlevelC->GetFrame().SetRot(mrot);
		mlevelC->GetFrame().SetPos(ChVector<>(0,j*0.02,0));

		mlevelB->AddAsset(mlevelC);
	}

	mbody->AddAsset(mlevelB);

			// ==Asset== Attach a video camera. This will be used by Irrlicht, 
			// or POVray postprocessing, etc. Note that a camera can also be 
			// put in a moving object.
	ChSharedPtr<ChCamera> mcamera(new ChCamera);
	mcamera->SetAngle(50);
	mcamera->SetPosition(ChVector<>(-3,4,-5));
	mcamera->SetAimPoint(ChVector<>(0,1,0));
	mbody->AddAsset(mcamera);
~~~


# Example 3

Create a @ref ChParticleClones cluster, and attach 'assets' that define a 
single "sample" 3D shape. This will be shown N times in POV or Irrlicht.
	
~~~{.cpp}

			// Create the ChParticleClones, populate it with some random particles,
			// and add it to physical system:
	ChSharedPtr<ChParticlesClones> mparticles(new ChParticlesClones);

			// Note: coll. shape, if needed, must be specified before creating particles
	mparticles->GetCollisionModel()->ClearModel();
	mparticles->GetCollisionModel()->AddSphere(0.05);
	mparticles->GetCollisionModel()->BuildModel();
	mparticles->SetCollide(true);
		
			// Create the random particles 
	for (int np=0; np<100; ++np)
		mparticles->AddParticle(ChCoordsys<>(ChVector<>(ChRandom()-2,1.5,ChRandom()+2)));

			// Do not forget to add the particle cluster to the system:
	mphysicalSystem.Add(mparticles);

			//  ==Asset== Attach a 'sphere' shape asset.. it will be used as a sample 
			// shape to display all particles when rendering in 3D! 
	ChSharedPtr<ChSphereShape> mspherepart(new ChSphereShape);
	mspherepart->GetSphereGeometry().rad = 0.05;
	mparticles->AddAsset(mspherepart);
~~~


# The POV exporter

The following part is very important because this is what makes 
this demo different from the demo_irr_assets, that used Irrlicht. 
We need to create a postprocessor of type @ref ChPovRay and tell him that 
we are going to export our visualization assets:

~~~{.cpp}	
			// Create an exporter to POVray !!!
	
	ChPovRay pov_exporter = ChPovRay(&mphysicalSystem);

			// Sets some file names for in-out processes.
	pov_exporter.SetTemplateFile		("../data/_template_POV.pov");
	pov_exporter.SetOutputScriptFile    ("rendering_frames.pov");
	pov_exporter.SetOutputDataFilebase  ("my_state");
	pov_exporter.SetPictureFilebase     ("picture");
		
			// Even better: save the .dat files and the .bmp files
			// in two subdirectories, to avoid cluttering the current
			// directory...
	ChFileutils::MakeDirectory("output");
	ChFileutils::MakeDirectory("anim");

	pov_exporter.SetOutputDataFilebase("output/my_state");
	pov_exporter.SetPictureFilebase("anim/picture");

			// --Optional: modify default light
	pov_exporter.SetLight(ChVector<>(-3,4,2), ChColor(0.15,0.15,0.12), false);

			// --Optional: add further POV commands, for example in this case:
			//     create an area light for soft shadows
			//     create a Grid object; Grid() parameters: step, linewidth, linecolor, planecolor
			//   Remember to use \ at the end of each line for a multiple-line string.
	pov_exporter.SetCustomPOVcommandsScript(" \
		light_source {   \
		<2, 10, -3>  \
		color rgb<1.2,1.2,1.2> \
		area_light <4, 0, 0>, <0, 0, 4>, 8, 8 \
		adaptive 1 \
		jitter\
		} \
		object{ Grid(1,0.02, rgb<0.7,0.8,0.8>, rgbt<1,1,1,1>) rotate <0, 0, 90>  } \
    ");

			// --Optional: attach additional custom POV commands to some of the rigid bodies,
			//   using the ChPovRayAssetCustom asset. This asset for example projects a 
			//   checkered texture to the floor. This POV specific asset won't be rendered 
			//   by Irrlicht or other interfaces.
	ChSharedPtr<ChPovRayAssetCustom> mPOVcustom(new ChPovRayAssetCustom);
	mPOVcustom->SetCommands("pigment { checker rgb<0.9,0.9,0.9>, rgb<0.75,0.8,0.8> }");
	mfloor->AddAsset(mPOVcustom);


			// IMPORTANT! Tell to the POVray exporter that 
			// he must take care of converting the shapes of
			// all items!

	pov_exporter.AddAll();

			// (Optional: tell selectively which physical items you 
			// want to render in the folllowing way...)
			//	pov_exporter.RemoveAll();
			//	pov_exporter.Add(mfloor);
			//	pov_exporter.Add(mbody);
			//	pov_exporter.Add(mparticles);
~~~


# The simulation loop

Now you have to write the usual while() loop to perform the simulation. 
Note that before running the loop you need to use  pov_exporter.ExportScript(); , 
and for each timestep you must use pov_exporter.ExportData(); 
actually this is the instruction that creates the many .dat and .pov files in the output directory.

~~~{.cpp}
	// 1) Create the two .pov and .ini files for POV-Ray (this must be done
	//    only once at the beginning of the simulation).

	pov_exporter.ExportScript();

	while (mphysicalSystem.GetChTime() < 1.5) 
	{
		mphysicalSystem.DoStepDynamics(0.01);

		GetLog()<< "time= " << mphysicalSystem.GetChTime() <<"\n";

		// 2) Create the incremental nnnn.dat and nnnn.pov files that will be load
		//    by the pov .ini script in POV-Ray (do this at each simulation timestep)
		pov_exporter.ExportData();
	}
~~~

# Executing and rendering with POVray

Once you created your program, compile it, then:

- execute the `demo_postprocess.exe`
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

\include demo_postprocess.cpp

