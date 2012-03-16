//////////////////////////////////////////////////
//  
//   ChPovRay.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

    
  
#include "ChPovRay.h"
#include "geometry/ChCTriangleMeshConnected.h"
#include "assets/ChObjShapeFile.h"
#include "assets/ChSphereShape.h"
#include "assets/ChBoxShape.h"
#include "physics/ChParticlesClones.h"

using namespace chrono;
using namespace postprocess;
using namespace geometry;

 
ChPovRay::ChPovRay(ChSystem* system) : ChPostProcessBase(system)
{
	this->pic_filename		= "pic";
	this->template_filename = "../data/_template_POV.pov";
	this->out_script_filename = "render_frames.pov";
	this->out_data_filename = "state";
	this->framenumber = 0;
	this->camera_location = ChVector<>(0,1.5,-2);
	this->camera_aim = ChVector<>(0,0,0);
	this->camera_angle =30;
	this->camera_orthographic = false;
	this->def_light_location = ChVector<>(2,3,-1);
	this->def_light_color = ChColor(1,1,1);
	this->def_light_cast_shadows = true;
	this->background = ChColor(1,1,1);
	this->antialias = false;
	this->antialias_depth = 2;
	this->antialias_treshold = 0.1;
	this->picture_height = 600;
	this->picture_width = 800;
	this->ambient_light = ChColor(2,2,2);
	this->COGs_show = false;
	this->COGs_size = 0.04;
	this->frames_show = false;
	this->frames_size = 0.05;
}

void ChPovRay::AddAll()
{
	ChSystem::IteratorBodies myiter = mSystem->IterBeginBodies();
	while (myiter != mSystem->IterEndBodies())
	{	
		this->mdata.push_back((*myiter));
		++myiter;
	}
}

void ChPovRay::Add(ChSharedPtr<ChPhysicsItem> mitem)
{
	this->mdata.push_back(mitem);
}

void ChPovRay::RemoveAll()
{
	this->mdata.clear();
}



std::string replaceOnce(
  std::string result, 
  const std::string& replaceWhat, 
  const std::string& replaceWithWhat)
{
  const int pos = result.find(replaceWhat);
  if (pos==-1) return result;
  result.replace(pos,replaceWhat.size(),replaceWithWhat);
  return result;
}


std::string replaceAll(
  std::string result, 
  const std::string& replaceWhat, 
  const std::string& replaceWithWhat)
{
  while(1)
  {
    const int pos = result.find(replaceWhat);
    if (pos==-1) break;
    result.replace(pos,replaceWhat.size(),replaceWithWhat);
  }
  return result;
}


void ChPovRay::SetCamera(ChVector<> location, ChVector<> aim, double angle, bool ortho)
{
	this->camera_location = location;
	this->camera_aim = aim;
	this->camera_angle = angle;
	this->camera_orthographic = ortho;
}

void ChPovRay::SetLight(ChVector<> location, ChColor color, bool cast_shadow)
{
	this->def_light_location = location;
	this->def_light_color = color;
	this->def_light_cast_shadows = cast_shadow;
}

void ChPovRay::SetShowCOGs(bool show, double msize)
{
	this->COGs_show = show;
	if (show) 
		this->COGs_size = msize;
}
void ChPovRay::SetShowFrames(bool show, double msize)
{
	this->frames_show = show;
	if (show) 
		this->frames_size = msize;
}


void ChPovRay::ExportScript(const std::string &filename)
{
	this->out_script_filename = filename;
	
	pov_assets.clear();

	// Generate the _assets.pov script (initial state, it will be populated later by
	// appending assets as they enter the exporter, only once if shared, using ExportAssets() )

	std::string assets_filename = filename + ".assets";
	{
		ChStreamOutAsciiFile assets_file(assets_filename.c_str());
		assets_file << "// File containing meshes and objects for rendering POV scenes.\n";
		assets_file << "// This file is automatically included by " << filename.c_str() << ".pov , \n";
		assets_file << "// and you should not modify it.\n\n";
	}

	// Generate the .INI script
	std::string ini_filename = filename + ".ini";

	ChStreamOutAsciiFile ini_file(ini_filename.c_str());

	ini_file << "; Script for rendering an animation with POV-Ray. \n";
	ini_file << "; Generated autumatically by Chrono::Engine. \n\n";
	if (this->antialias)
		ini_file << "Antialias=On \n";
	else
		ini_file << "Antialias=Off \n";
	ini_file << "Antialias_Threshold=" << this->antialias_treshold << " \n";
	ini_file << "Antialias_Depth=" << this->antialias_depth << " \n";
	ini_file << "Height=" << this->picture_height << " \n";
	ini_file << "Width =" << this->picture_width << " \n";
	ini_file << "Input_File_Name=" << out_script_filename << "\n";
	ini_file << "Output_File_Name=" << pic_filename << "\n";
	ini_file << "Initial_Frame=0000 \n";
	ini_file << "Final_Frame=0999 \n";
	ini_file << "Initial_Clock=0 \n";
	ini_file << "Final_Clock=1 \n";
	ini_file << "Pause_when_Done=off \n";


	// Generate the .POV script:

	ChStreamOutAsciiFile mfile(filename.c_str());

	// Rough way to load the template head file in the string buffer
	if (template_filename != "")
	{
		ChStreamInAsciiFile templatefile(template_filename.c_str());
		std::string buffer_template;
		while(!templatefile.End_of_stream())
		{
			char m;
			try{
				templatefile >> m;
			} catch(ChException mex){};

			buffer_template += m;
		}

		// Do template replacement of [xxxyyyzzz] keywords, if any
		replaceAll(buffer_template, "[xxxyyyzzz]", "blabla" );
		
		mfile << buffer_template;
	}
	
	// Write default global settings and background

	mfile <<	"global_settings { \n" << 
				" ambient_light rgb<" << ambient_light.R <<","<< ambient_light.G <<","<< ambient_light.B <<"> \n"; 
	mfile <<	"}\n\n\n";
	mfile <<	"background { \n" << 
				" rgb <" << background.R <<","<< background.G <<","<< background.B <<"> \n"; 
	mfile <<	"}\n\n\n";

	// Write default camera

	mfile <<	"camera { \n" <<             
				" right -x*image_width/image_height \n" <<
				" location <" << camera_location.x <<","<<camera_location.y <<","<<camera_location.z <<"> \n" <<
				" direction 2*z \n" << 
				" look_at <" << camera_aim.x <<","<<camera_aim.y <<","<<camera_aim.z <<"> \n" << 
				" angle " << camera_angle << " \n";
	if (camera_orthographic) 
		mfile <<" orthographic \n";
	mfile <<	"}\n\n\n"; 

	// Write default light

	mfile <<	"light_source { \n" << 
				" <" << def_light_location.x <<","<< def_light_location.y <<","<< def_light_location.z <<"> \n" <<
				" color rgb<" << def_light_color.R <<","<< def_light_color.G <<","<<def_light_color.B <<"> \n"; 
	if (!def_light_cast_shadows) 
		mfile <<" shadowless \n";
	mfile <<	"}\n\n\n"; 

	// Write POV custom code

	if ( this->custom_script.size() >0)
	{
		mfile << "// Custom user-added script: \n\n";
		mfile << this->custom_script;
		mfile << "\n\n";
	}

	// Write POV code to open the asset file

	mfile << "#include \"" << assets_filename << "\"\n\n";

	// Write POV code to open the n.th scene file

	mfile << "#declare scene_file = concat(\"" << this->out_data_filename << "\", str(frame_number,-5,0), \".pov\") \n"; 
	mfile << "#include scene_file \n";


	// Populate the assets
	this->ExportAssets();

}



void ChPovRay::ExportAssets()
{
	// open asset file in append mode.
	std::string assets_filename = this->out_script_filename + ".assets";
	ChStreamOutAsciiFile assets_file(assets_filename.c_str(), std::ios::app);

	// This will scan all the ChPhysicsItem added objects, and if
	// they have some reference to renderizable assets, write geoemtries in 
	// the POV assets script.

	for (unsigned int i = 0; i< this->mdata.size(); i++)
	{

		// Scan assets in object i
		for (unsigned int k = 0; k < mdata[i]->GetAssets().size(); k++)
		{
			ChSharedPtr<ChAsset> k_asset = mdata[i]->GetAssets()[k];

			ChHashTable<unsigned int, ChSharedPtr<ChAsset> >::iterator mcached = pov_assets.find( (unsigned int)k_asset.get_ptr() );
			if (mcached == pov_assets.end())
			{
				// Ok, add the asset in POV file, it was not already saved. 
				// Otherwise it was a shared asset.
				pov_assets.insert((unsigned int)k_asset.get_ptr(), k_asset);

				// Do dynamic casting of the shared pointer to see which type
				// of asset is contained...


				// 1) asset k of object i references an .obj wavefront mesh?
				if (k_asset.IsType<ChObjShapeFile>() )
				{
					ChSharedPtr<ChObjShapeFile> myobjshapeasset(k_asset);
					ChTriangleMeshConnected mytrimesh;

					try {
						// Load from the .obj file and convert.
						mytrimesh.LoadWavefrontMesh( myobjshapeasset->GetFilename() );

						// POV macro to build the asset - begin
						assets_file << "#macro sh_"<< (int) k_asset.get_ptr() << "(apx, apy, apz, aq0, aq1, aq2, aq3)\n";

						// Create mesh
						assets_file << "mesh2  {\n";

						assets_file << " vertex_vectors {\n";
						assets_file << mytrimesh.m_vertices.size() << ",\n";
						for (unsigned int iv = 0; iv < mytrimesh.m_vertices.size(); iv++)
							assets_file << "  <" << mytrimesh.m_vertices[iv].x << "," <<  mytrimesh.m_vertices[iv].y << "," <<  mytrimesh.m_vertices[iv].z << ">,\n";
						assets_file <<" }\n";

						assets_file << " normal_vectors {\n";
						assets_file << mytrimesh.m_normals.size() << ",\n";
						for (unsigned int iv = 0; iv < mytrimesh.m_normals.size(); iv++)
							assets_file << "  <" << mytrimesh.m_normals[iv].x << "," <<  mytrimesh.m_normals[iv].y << "," <<  mytrimesh.m_normals[iv].z << ">,\n";
						assets_file <<" }\n";

						assets_file << " face_indices {\n";
						assets_file << mytrimesh.m_face_v_indices.size() << ",\n";
						for (unsigned int it = 0; it < mytrimesh.m_face_v_indices.size(); it++)
							assets_file << "  <" << mytrimesh.m_face_v_indices[it].x << "," <<  mytrimesh.m_face_v_indices[it].y << "," <<  mytrimesh.m_face_v_indices[it].z << ">,\n";
						assets_file <<" }\n";

						assets_file <<" pigment {color rgbt <" << 
							myobjshapeasset->GetColor().R << "," << 
							myobjshapeasset->GetColor().G << "," << 
							myobjshapeasset->GetColor().B << "," << 
							myobjshapeasset->GetFading() << "> }\n";
						assets_file <<" quatRotation(<aq0, aq1, aq2, aq3>) \n";
						assets_file <<" translate  <apx, apy, apz> \n";
						//assets_file <<" texture{ atexture }\n";
						assets_file <<"}\n";

						// POV macro - end
						assets_file << "#end \n";
					} 
					catch (ChException)
					{
						char error[400];
						sprintf(error,"Asset n.%d of object %d : can't read .obj file %s", k,i,myobjshapeasset->GetFilename().c_str() );
						throw (ChException(error));
					}
				}


				// 2) asset k of object i is a sphere ?
				if (k_asset.IsType<ChSphereShape>() )
				{
					ChSharedPtr<ChSphereShape> myobjshapeasset(k_asset);

					// POV macro to build the asset - begin
					assets_file << "#macro sh_"<< (int) k_asset.get_ptr() << "(apx, apy, apz, aq0, aq1, aq2, aq3)\n";

					// POV will make the sphere
					assets_file << "sphere  {\n";

					assets_file << " <" << myobjshapeasset->GetSphereGeometry().center.x;
					assets_file << ","  << myobjshapeasset->GetSphereGeometry().center.y;
					assets_file << ","  << myobjshapeasset->GetSphereGeometry().center.z << ">\n";
					assets_file << " "  << myobjshapeasset->GetSphereGeometry().rad << "\n";

					assets_file <<" pigment {color rgbt <" << 
							myobjshapeasset->GetColor().R << "," << 
							myobjshapeasset->GetColor().G << "," << 
							myobjshapeasset->GetColor().B << "," << 
							myobjshapeasset->GetFading() << "> }\n";
					assets_file <<" quatRotation(<aq0, aq1, aq2, aq3>) \n";
					assets_file <<" translate  <apx, apy, apz> \n";
					assets_file <<"}\n";

					// POV macro - end 
					assets_file << "#end \n";
				}


				// 3) asset k of object i is a box ?
				if (k_asset.IsType<ChBoxShape>() )
				{
					ChSharedPtr<ChBoxShape> myobjshapeasset(k_asset);

					// POV macro to build the asset - begin
					assets_file << "#macro sh_"<< (int) k_asset.get_ptr() << "(apx, apy, apz, aq0, aq1, aq2, aq3)\n";

					// POV will make the sphere
					assets_file << "union  {\n";
					assets_file << "box  {\n";

					assets_file << " <" << -myobjshapeasset->GetBoxGeometry().Size.x;
					assets_file << ","  << -myobjshapeasset->GetBoxGeometry().Size.y;
					assets_file << ","  << -myobjshapeasset->GetBoxGeometry().Size.z << ">\n";
					assets_file << " <" <<  myobjshapeasset->GetBoxGeometry().Size.x;
					assets_file << ","  <<  myobjshapeasset->GetBoxGeometry().Size.y;
					assets_file << ","  <<  myobjshapeasset->GetBoxGeometry().Size.z << ">\n";

					ChQuaternion<> boxrot = myobjshapeasset->GetBoxGeometry().Rot.Get_A_quaternion();
					assets_file <<" quatRotation(<" << boxrot.e0;
					assets_file <<"," << boxrot.e1;
					assets_file <<"," << boxrot.e2;
					assets_file <<"," << boxrot.e3 << ">) \n";
					assets_file <<" translate  <" << myobjshapeasset->GetBoxGeometry().Pos.x;
					assets_file <<"," << myobjshapeasset->GetBoxGeometry().Pos.y;
					assets_file <<"," << myobjshapeasset->GetBoxGeometry().Pos.z << "> \n";

					assets_file <<"}\n"; // end box

					assets_file <<" pigment {color rgbt <" << 
							myobjshapeasset->GetColor().R << "," << 
							myobjshapeasset->GetColor().G << "," << 
							myobjshapeasset->GetColor().B << "," << 
							myobjshapeasset->GetFading() << "> }\n";
					assets_file <<" quatRotation(<aq0, aq1, aq2, aq3>) \n";
					assets_file <<" translate  <apx, apy, apz> \n";
					assets_file <<"}\n"; // end union

					// POV macro - end 
					assets_file << "#end \n";
				}

			} // end if asset not yet saved

		} // end loop on assets of i-th object 
	} // end loop on objects



}








void ChPovRay::ExportData(const std::string &filename)
{
	// Populate the assets (because maybe that during the 
	// animation someone created an object with asset, after
	// the initial call to ExportScript() - but already present
	// assets won't be appended)

	this->ExportAssets();


	// Generate the nnnn.dat and nnnn.pov files:

	try 
	{
		char pathdat[200];
		sprintf(pathdat,"%s.dat", filename.c_str());
		ChStreamOutAsciiFile mfiledat(pathdat);

		char pathpov[200];
		sprintf(pathpov,"%s.pov", filename.c_str());
		ChStreamOutAsciiFile mfilepov(pathpov);


		// Write custom data commands, if provided by the user
		if ( this->custom_data.size() >0)
		{
			mfilepov << "// Custom user-added script: \n\n";
			mfilepov << this->custom_data;
			mfilepov << "\n\n";
		}

		// Save time-dependent data for the geometry of objects in ...nnnn.POV 
		// and in ...nnnn.DAT file

		for (unsigned int i = 0; i< this->mdata.size(); i++)
		{			
			if (mdata[i].IsType<ChBody>() )
			{
				ChSharedPtr<ChBody> mybody(mdata[i]);

				// Get the current coordinate frame of the i-th object
				ChCoordsys<> assetcsys = CSYSNORM;
				ChFrame<> bodyframe = mybody->GetFrame_REF_to_abs();
				assetcsys = bodyframe.GetCoord();

				// Scan assets in object and write the macro to set their position
				for (unsigned int k = 0; k < mdata[i]->GetAssets().size(); k++)
				{
					ChSharedPtr<ChAsset> k_asset = mdata[i]->GetAssets()[k];

					// 1) asset k of object i references an .obj wavefront mesh?
					if ( k_asset.IsType<ChObjShapeFile>() ||
						 k_asset.IsType<ChSphereShape>() || 
						 k_asset.IsType<ChBoxShape>() )
					{
						mfilepov << "sh_"<< (unsigned int) k_asset.get_ptr() << "("; // apx, apy, apz, aq0, aq1, aq2, aq3, atexture)\n";

						mfilepov << assetcsys.pos.x << "," << assetcsys.pos.y << "," << assetcsys.pos.z << ",";
						mfilepov << assetcsys.rot.e0 << "," << assetcsys.rot.e1 << "," << assetcsys.rot.e2 << "," << assetcsys.rot.e3 << ")\n";
					}

				} // end loop on assets

				// Show body COG?
				if (this->COGs_show)
				{
					ChCoordsys<> cogcsys = mybody->GetFrame_COG_to_abs().GetCoord();
					mfilepov << "sh_csysCOG(";
					mfilepov << cogcsys.pos.x << "," << cogcsys.pos.y << "," << cogcsys.pos.z << ",";
					mfilepov << cogcsys.rot.e0 << "," << cogcsys.rot.e1 << "," << cogcsys.rot.e2 << "," << cogcsys.rot.e3  << ",";
					mfilepov << this->COGs_size << ")\n";
				}
				// Show body frame ref?
				if (this->frames_show)
				{
					mfilepov << "sh_csysFRM(";
					mfilepov << assetcsys.pos.x << "," << assetcsys.pos.y << "," << assetcsys.pos.z << ",";
					mfilepov << assetcsys.rot.e0 << "," << assetcsys.rot.e1 << "," << assetcsys.rot.e2 << "," << assetcsys.rot.e3  << ",";
					mfilepov << this->frames_size << ")\n";
				}
				
			}


			if (mdata[i].IsType<ChParticlesClones>() )
			{
				ChSharedPtr<ChParticlesClones> myclones(mdata[i]);

				// Loop on all particle clones
				for (unsigned int m = 0; m < myclones->GetNparticles() ; ++m)
				{
					// Get the current coordinate frame of the i-th particle
					ChCoordsys<> assetcsys = CSYSNORM;
					assetcsys = myclones->GetParticle(m).GetCoord();

					// Scan assets in object and write the macro to set their position
					for (unsigned int k = 0; k < mdata[i]->GetAssets().size(); k++)
					{
						ChSharedPtr<ChAsset> k_asset = mdata[i]->GetAssets()[k];

						// 1) asset k of object i references an .obj wavefront mesh?
						if ( k_asset.IsType<ChObjShapeFile>() ||
							 k_asset.IsType<ChSphereShape>() )
						{
							mfilepov << "sh_"<< (int) k_asset.get_ptr() << "("; // apx, apy, apz, aq0, aq1, aq2, aq3, atexture)\n";

							mfilepov << assetcsys.pos.x << "," << assetcsys.pos.y << "," << assetcsys.pos.z << ",";
							mfilepov << assetcsys.rot.e0 << "," << assetcsys.rot.e1 << "," << assetcsys.rot.e2 << "," << assetcsys.rot.e3 << ")\n";
						}

					} // end loop on assets
				} // end loop on particles
			}
			
		} // end loop on objects

	}catch (ChException)
	{
		char error[400];
		sprintf(error,"Can't save data into file %s.pov (or .dat)", filename.c_str() );
		throw (ChException(error));
	}

	// Increment the number of the frame.
	this->framenumber ++;
}




