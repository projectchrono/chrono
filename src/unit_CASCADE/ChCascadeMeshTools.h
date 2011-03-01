#ifndef CHCASCADEMESHTOOLS_H
#define CHCASCADEMESHTOOLS_H

//////////////////////////////////////////////////
//
//   ChCascadeMeshTools.h
//
//   Tools to convert an OpenCASCADE shape into 
//   triangle meshes.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "unit_CASCADE/ChApiCASCADE.h"
#include "core/ChStream.h"
#include "geometry/ChCTriangleMesh.h"

class TopoDS_Face;
class TopoDS_Shape;
class Poly_Connect;
class TColgp_Array1OfDir;
class Handle_TDocStd_Document;
class TopLoc_Location;
class TDF_Label;

namespace chrono
{
namespace cascade
{


/// Tools to convert an OpenCASCADE shapes into 
/// triangle meshes.

class ChApiCASCADE ChCascadeMeshTools
{

public:

	//---------------------------------------------------------------------------------
	// CONVERSION TO CHRONO TRIANGLE MESHES 


		/// This function can be used to convert a OpenCASCADE face into a triangle mesh.
		/// The face must be already mshed (ex because you called fillTriangleMeshFromCascade before).
	static void fillTriangleMeshFromCascadeFace (geometry::ChTriangleMesh& chmesh,	///< Mesh that will be filled with triangles
											 const TopoDS_Face& F		///< OpenCASCADE face to be meshed
											 );


		/// This function can be used to convert a OpenCASCADE shape into a
		/// Chrono::Engine ChTriangleMesh triangle mesh.
	static void fillTriangleMeshFromCascade (geometry::ChTriangleMesh& chmesh,	 ///< Mesh that will be filled with triangles
											 const TopoDS_Shape& mshape, ///< OpenCASCADE face to be meshed
											 double deflection=0.5,		 ///< Tolerance on meshing (the lower, the finer the mesh)
											 double angulardeflection=20
											 );


	//---------------------------------------------------------------------------------
	// CONVERSION TO 'OBJ' WAVEFRONT FILE FORMAT

		/// This function can be used to convert a OpenCASCADE shape into a
		/// 'obj' file format. The file 'objfile' must be already opened, and empty. 
		/// Also normals are saved. 
	static void fillObjFileFromCascade		(ChStreamOutAscii& objfile,  ///< the .obj file will be written here
											 const TopoDS_Shape& mshape, ///<
											 double deflection=0.5,		 ///< Tolerance on meshing (the lower, the finer the mesh)
											 double angulardeflection=20
											 );



	//---------------------------------------------------------------------------------
	// Oter utility stuff

		/// Given an OpenCASCADE face, computes the normals of triangles, if
		/// already meshed. Mostly used internally.
	static void ComputeNormal(const TopoDS_Face& aFace, 
							  Poly_Connect& pc, 
							  TColgp_Array1OfDir& Nor);

};



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif // END of header