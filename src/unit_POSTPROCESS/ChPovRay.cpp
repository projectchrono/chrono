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



void ChPovRay::ExportScript(const std::string &filename)
{
	this->out_script_filename = filename;
	
	// Generate the .INI script
	std::string ini_filename = filename + ".ini";

	ChStreamOutAsciiFile ini_file(ini_filename.c_str());

	ini_file << "; Script for rendering an animation with POV-Ray. \n";
	ini_file << "; Generated autumatically by Chrono::Engine. \n\n";
	ini_file << "Antialias=Off \n";
	ini_file << "Antialias_Threshold=0.1 \n";
	ini_file << "Antialias_Depth=2 \n";
	ini_file << "Height=600 \n";
	ini_file << "Width =800 \n";
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
		replaceAll(buffer_template, "[xxxyyyzzz]", "blable" );
		
		mfile << buffer_template;
	}
	
	// Write POV code to open the n.th datafile

	mfile << "#declare data_file = concat(\"" << this->out_data_filename << "\", str(frame_number,-5,0), \".dat\") \n"; 
    mfile << "#warning concat(\"---- LOADING PEBBLES FILE : \", data_file, \"\\n\") \n";  
    mfile << "#fopen MyPosFile data_file read \n";


	// Write geometry of objects in POV file
	// This will scan all the ChPhysicsItem added objects, and if
	// they have some reference to renderizable assets, write geoemtries in the POV script.

	for (unsigned int i = 0; i< this->mdata.size(); i++)
	{
		// Get the coordinate frame of the i-th object, if any.
		ChCoordsys<> assetcsys = CSYSNORM;
		if (mdata[i].IsType<ChBody>() )
		{
			ChSharedPtr<ChBody> mybody(mdata[i]);
			ChFrame<> bodyframe = mybody->GetFrame_REF_to_abs();
			assetcsys = bodyframe.GetCoord();
		}

		mfile <<"\n\n// Item:" << mdata[i]->GetName() << "\n\n";


		// Scan assets in object i
		for (unsigned int k = 0; k < mdata[i]->GetAssets().size(); k++)
		{
			// Do dynamic casting of the shared pointer to see which type
			// of asset is contined...

			// 1) asset k of object i references an .obj wavefront mesh?
			if (mdata[i]->GetAssets()[k].IsType<ChObjShapeFile>() )
			{
				ChSharedPtr<ChObjShapeFile> myobjshapeasset(mdata[i]->GetAssets()[k]);
				ChTriangleMeshConnected mytrimesh;

				try {
					// Load from the .obj file and convert.
					mytrimesh.LoadWavefrontMesh( myobjshapeasset->GetFilename() );

					// POV will read pos & rotation of mesh from a row of the .dat file
					mfile << "#read (MyPosFile, apx, apy, apz, aq0, aq1, aq2, aq3) \n\n";

					// Create mesh
					mfile << "mesh2  {\n";

					mfile << " vertex_vectors {\n";
					mfile << mytrimesh.m_vertices.size() << ",\n";
					for (unsigned int iv = 0; iv < mytrimesh.m_vertices.size(); iv++)
						mfile << "  <" << mytrimesh.m_vertices[iv].x << "," <<  mytrimesh.m_vertices[iv].y << "," <<  mytrimesh.m_vertices[iv].z << ">,\n";
					mfile <<" }\n";

					mfile << " normal_vectors {\n";
					mfile << mytrimesh.m_normals.size() << ",\n";
					for (unsigned int iv = 0; iv < mytrimesh.m_normals.size(); iv++)
						mfile << "  <" << mytrimesh.m_normals[iv].x << "," <<  mytrimesh.m_normals[iv].y << "," <<  mytrimesh.m_normals[iv].z << ">,\n";
					mfile <<" }\n";

					mfile << " face_indices {\n";
					mfile << mytrimesh.m_face_v_indices.size() << ",\n";
					for (unsigned int it = 0; it < mytrimesh.m_face_v_indices.size(); it++)
						mfile << "  <" << mytrimesh.m_face_v_indices[it].x << "," <<  mytrimesh.m_face_v_indices[it].y << "," <<  mytrimesh.m_face_v_indices[it].z << ">,\n";
					mfile <<" }\n";

					mfile <<" pigment {color rgb <" << myobjshapeasset->GetColor().R << "," << myobjshapeasset->GetColor().G << "," << myobjshapeasset->GetColor().B << "> }\n";
					mfile <<" quatRotation(<aq0, aq1, aq2, aq3>) \n";
                    mfile <<" translate  <apx, apy, apz> \n";
					mfile <<"}\n";
				} 
				catch (ChException)
				{
					char error[400];
					sprintf(error,"Asset n.%d of object %d : can't read .obj file %s", k,i,myobjshapeasset->GetFilename().c_str() );
					throw (ChException(error));
				}
			}

			// 2) asset k of object i is a sphere ?
			if (mdata[i]->GetAssets()[k].IsType<ChSphereShape>() )
			{
				ChSharedPtr<ChSphereShape> myobjshapeasset(mdata[i]->GetAssets()[k]);

				// POV will read pos & rotation of sphere from a row of the .dat file
				mfile << "#read (MyPosFile, apx, apy, apz, aq0, aq1, aq2, aq3) \n\n";

				// POV will make the sphere
				mfile << "sphere  {\n";

				mfile << " <" << myobjshapeasset->GetSphereGeometry().center.x;
				mfile << ","  << myobjshapeasset->GetSphereGeometry().center.y;
				mfile << ","  << myobjshapeasset->GetSphereGeometry().center.z << ">\n";
				mfile << " "  << myobjshapeasset->GetSphereGeometry().rad << "\n";

				mfile <<" pigment {color rgb <" << myobjshapeasset->GetColor().R << "," << myobjshapeasset->GetColor().G << "," << myobjshapeasset->GetColor().B << "> }\n";
				mfile <<" quatRotation(<aq0, aq1, aq2, aq3>) \n";
                mfile <<" translate  <apx, apy, apz> \n";
				mfile <<"}\n";
			}



		} // end loop on assets of i-th object 
	} // end loop on objects


}

void ChPovRay::ExportData(const std::string &filename)
{
	// Generate the .dat file:

	try {
		ChStreamOutAsciiFile mfile(filename.c_str());


		// Save time-dependent data for the geometry of objects in POV file
		// NOTE!!! The following loop MUST reflect the same sequence of loadings
		// that are done in the .pov script, as defined in the ExportScript() 

		for (unsigned int i = 0; i< this->mdata.size(); i++)
		{
			// Get the coordinate frame of the i-th object, if any.
			ChCoordsys<> assetcsys = CSYSNORM;
			if (mdata[i].IsType<ChBody>() )
			{
				ChSharedPtr<ChBody> mybody(mdata[i]);
				ChFrame<> bodyframe = mybody->GetFrame_REF_to_abs();
				assetcsys = bodyframe.GetCoord();
			}

			// Scan assets in object i
			for (unsigned int k = 0; k < mdata[i]->GetAssets().size(); k++)
			{

				// Do dynamic casting of the shared pointer to see which type
				// of asset is contined...

				// 1) asset k of object i references an .obj wavefront mesh?
				if (mdata[i]->GetAssets()[k].IsType<ChObjShapeFile>() )
				{
					// Save pos & rotation of mesh in a row of the .dat file
					mfile << assetcsys.pos.x << "," << assetcsys.pos.y << "," << assetcsys.pos.z << ",";
					mfile << assetcsys.rot.e0 << "," << assetcsys.rot.e1 << "," << assetcsys.rot.e2 << "," << assetcsys.rot.e3 << "\n";
				}

				// 2) asset k of object i is a sphere ?
				if (mdata[i]->GetAssets()[k].IsType<ChSphereShape>() )
				{
					// Save pos & rotation of mesh in a row of the .dat file
					mfile << assetcsys.pos.x << "," << assetcsys.pos.y << "," << assetcsys.pos.z << ",";
					mfile << assetcsys.rot.e0 << "," << assetcsys.rot.e1 << "," << assetcsys.rot.e2 << "," << assetcsys.rot.e3 << "\n";
				}

			} // end loop on assets
		} // end loop on objects

	}catch (ChException)
	{
		char error[400];
		sprintf(error,"Can't save data into file %s", filename.c_str() );
		throw (ChException(error));
	}

	// Increment the number of the frame.
	this->framenumber ++;
}




