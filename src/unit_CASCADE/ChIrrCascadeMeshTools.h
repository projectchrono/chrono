//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHIRRCASCADEMESHTOOLS_H
#define CHIRRCASCADEMESHTOOLS_H

//////////////////////////////////////////////////
//
//   ChIrrCascadeMeshTools.h
//
//   FOR IRRLICHT USERS ONLY!
//
//   Some functions to allow easy creation of
//   meshes for Irrlicht visualization from OpenCASCADE shapes
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <irrlicht.h>

#include "ChCascadeDoc.h"
#include "ChCascadeMeshTools.h"

#include <TopoDS_Shape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_HShape.hxx>
#include <Handle_TopoDS_TShape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <BRepMesh.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS_Iterator.hxx>
#include <BRepAdaptor_HSurface.hxx>
#include <Poly_Connect.hxx>
#include <Poly_Triangle.hxx>
#include <Poly_Triangulation.hxx>
#include <TColgp_Array1OfDir.hxx>


namespace irr
{
namespace scene
{



/// Tools to convert an OpenCASCADE shapes into 
/// 'Irrlicht' triangle meshes.

class ChIrrCascadeMeshTools
{

public:

	//---------------------------------------------------------------------------------
	// CONVERSION TO 'IRRLICHT' MESHES


		/// Function to use to convert a OpenCASCADE face into a Irrlicht mesh,
		/// so that it can be used for visualizing it.

	static void fillIrrlichtMeshFromCascadeFace(scene::IMesh* pMesh, 
											const TopoDS_Face& F, 
											video::SColor clr = video::SColor(255, 255,255,255))
	{
		BRepAdaptor_Surface  BS(F, Standard_False);
		Handle(BRepAdaptor_HSurface) gFace = new BRepAdaptor_HSurface(BS);
		GeomAbs_SurfaceType thetype = BS.GetType();
		if (thetype == GeomAbs_Cylinder) {;};

		Handle(Poly_Triangulation) T;
		TopLoc_Location theLocation;
		T = BRep_Tool::Triangulation(F, theLocation);
			
		int vertface=0; int vertshift=1;

		if (!T.IsNull()) 
		{
			Poly_Connect pc(T);
					
			irr::scene::SMeshBuffer* buffer = new irr::scene::SMeshBuffer();
		
			buffer->Vertices.set_used( T->NbNodes() );
			buffer->Indices.set_used ( T->NbTriangles()*3);
			
			const TColgp_Array1OfPnt& mNodes = T->Nodes();
			TColgp_Array1OfDir mNormals(mNodes.Lower(), mNodes.Upper());
			
			// to compute the normal.
			chrono::cascade::ChCascadeMeshTools::ComputeNormal(F, pc, mNormals);

			int ivert = 0;
			for (int j= mNodes.Lower(); j<= mNodes.Upper(); j++) 
			{ 
				gp_Pnt p;
				gp_Dir pn;
				p = mNodes(j).Transformed(theLocation.Transformation());
				pn = mNormals(j);
				chrono::ChVector<> pos( p.X(),p.Y(),p.Z() );
				chrono::ChVector<> nor( pn.X(),pn.Y(),pn.Z() );
				
				buffer->Vertices[ivert] = irr::video::S3DVertex((irr::f32)pos.x, (irr::f32)pos.y, (irr::f32)pos.z,
							(irr::f32)nor.x, (irr::f32)nor.y, (irr::f32)nor.z,
							clr, 0, 0);
				ivert++;
			} 

			int itri =0;
			for (int j= T->Triangles().Lower(); j<= T->Triangles().Upper(); j++) 
			{ 
				Standard_Integer n[3];
				if (F.Orientation() == TopAbs_REVERSED) 
					T->Triangles()(j).Get(n[0],n[2],n[1]);
				else 
					T->Triangles()(j).Get(n[0],n[1],n[2]);
				int ia = (n[0])-1;
				int ib = (n[1])-1;
				int ic = (n[2])-1;

				buffer->Indices[itri*3+0]=ia;
				buffer->Indices[itri*3+1]=ib;
				buffer->Indices[itri*3+2]=ic;

				itri++;
			}

			irr::scene::SMesh* mmesh = dynamic_cast<irr::scene::SMesh*>(pMesh);
			mmesh->addMeshBuffer(buffer);
			mmesh->recalculateBoundingBox();
		}
	}


		/// Function to use to convert a OpenCASCADE shape into a Irrlicht mesh,
		/// so that it can be used for visualizing it.

	static void fillIrrlichtMeshFromCascade (scene::IMesh* pMesh, 
											 const TopoDS_Shape& mshape, 
											 double deflection=0.5, 
											 double angulardeflection=20, 
											 video::SColor clr = video::SColor(255, 255,255,255))
	{
		BRepTools::Clean(mshape);
		BRepMesh::Mesh(mshape, deflection);  
		// BRepMesh_IncrementalMesh M(mshape, deflection, Standard_False , angulardeflection);
		//GetLog() << "    ..tesselation done \n";
	  
		// Loop on faces..
		TopExp_Explorer ex;
		for (ex.Init(mshape, TopAbs_FACE); ex.More(); ex.Next()) 
		{
			const TopoDS_Face& F = TopoDS::Face(ex.Current());
			fillIrrlichtMeshFromCascadeFace(pMesh, F, clr);
		} 
	}

};



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

#endif // END of header

