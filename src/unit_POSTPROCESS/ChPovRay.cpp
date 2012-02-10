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
 

using namespace chrono;
using namespace postprocess;
using namespace geometry;

 
ChPovRay::ChPovRay(ChSystem* system) : ChPostProcessBase(system)
{
	this->pic_filename		= "pic";
	this->template_filename = "../data/_template_POV.pov";
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
	ini_file << "Input_File_Name=" << ini_filename << "\n";
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
	
	// Write geometry of objects in POV file
	// This will scan all the ChPhysicsItem added objects, and if
	// they have some reference to renderizable assets, write geoemtries in the POV script.

	for (unsigned int i = 0; i< this->mdata.size(); i++)
	{
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
					mytrimesh.LoadWavefrontMesh( myobjshapeasset->GetFilename() );
					GetLog() << "  converted mesh : " << myobjshapeasset->GetFilename() << "\n ";

					mfile <<"\n\n// Item:" << mdata[i]->GetName() << "\n";

		// Temporary: this will rely on  #read (MyPosFile, apx, apy, apz, aq0, aq1, aq2, aq3)
					ChCoordsys<> assetcsys = CSYSNORM;
					if (mdata[i].IsType<ChBody>() )
					{
						ChSharedPtr<ChBody> mybody(mdata[i]);
						ChFrame<> bodyframe = mybody->GetFrame_REF_to_abs();
						assetcsys = bodyframe.GetCoord();
					}	 
					mfile << "#declare apx=" << assetcsys.pos.x << ";\n";
					mfile << "#declare apy=" << assetcsys.pos.y << ";\n";
					mfile << "#declare apz=" << assetcsys.pos.z << ";\n";
					mfile << "#declare aq0=" << assetcsys.rot.e0 << ";\n";
					mfile << "#declare aq1=" << assetcsys.rot.e1 << ";\n";
					mfile << "#declare aq2=" << assetcsys.rot.e2 << ";\n";
					mfile << "#declare aq3=" << assetcsys.rot.e3 << ";\n\n";
		// ...end temporary trick

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

			// 2) asset k of object i is a bla bla.. ?


		}
	}


}

void ChPovRay::ExportData(const std::string &filename)
{
	//***TO DO***
}




