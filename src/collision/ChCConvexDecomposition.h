#ifndef CHC_CONVEXDECOMPOSITION_H
#define CHC_CONVEXDECOMPOSITION_H

//////////////////////////////////////////////////
//
//   ChCConvexDecomposition.h
//
//   Wrapper for the convex decomposition code
//   by John W. Ratcliff (in the convexdecomp/ dir)
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChApiCE.h"
#include "collision/convexdecomp/NvConvexDecomposition.h"
#include "geometry/ChCTriangleMesh.h"


using namespace chrono::geometry;

namespace chrono
{
namespace collision
{




///
/// Class for wrapping the NvConvexDecomposition code
/// by John W. Ratcliff (in the convexdecomp/ directory)
/// so that it is easier to use it by passing the Chrono::Engine
/// structures of type ChTriangleMesh.
///


class ChApi ChConvexDecomposition
{
public:

	//
	// FUNCTIONS
	//

		/// Basic constructor
	ChConvexDecomposition();

		/// Destructor
	virtual ~ChConvexDecomposition();


		/// Access directly the wrapped J.W.Ratcliff convex decomposition object 
		/// although it shouldn't be necessary, since this class already provide
		/// wrapping to all the functions).
	CONVEX_DECOMPOSITION::iConvexDecomposition* GetDecompositionObject();

		/// Reset the input mesh data
	virtual void Reset(void);

		/// Add a triangle, by passing three points for vertexes. 
		/// Note: the vertexes must be properly ordered (oriented triangle, normal pointing outside)
	virtual bool AddTriangle(const ChVector<>& v1,const ChVector<>& v2,const ChVector<>& v3);

		/// Add a triangle, by passing a  ChTriangle object (that will be copied, not referenced). 
		/// Note: the vertexes must be properly ordered (oriented triangle, normal pointing outside)
	virtual bool AddTriangle(const ChTriangle& t1);


		/// Add a triangle mesh soup, by passing an entire ChTriangleMesh object.
		/// Note 1: the triangle mesh does not need connectivity information (a basic 'triangle soup' is enough)
		/// Note 2: all vertexes must be properly ordered (oriented triangles, normals pointing outside).
		/// Note 3: the triangles must define closed volumes (holes, gaps in edges, etc. may trouble the decomposition)
	virtual bool AddTriangleMesh(const ChTriangleMesh& tm);


		/// Perform the convex decomposition. 
		/// This operation is time consuming, and it may take a while to complete.
		/// Quality of the results can depend a lot on the parameters. Also, meshes
		/// with triangles that are not well oriented (normals always pointing outside)
		/// or with gaps/holes, may give wrong results.
	virtual int ComputeConvexDecomposition( 
						float skinWidth=0,					///< Skin width on the convex hulls generated
						unsigned int decompositionDepth=8,	///< Recursion depth for convex decomposition.
						unsigned int maxHullVertices=64,	///< Maximum number of vertices in output convex hulls.
						float concavityThresholdPercent=0.1f, ///< The percentage of concavity allowed without causing a split to occur.
						float mergeThresholdPercent=30.0f,      ///< The percentage of volume difference allowed to merge two convex hulls.
						float volumeSplitThresholdPercent=0.1f, ///< The percentage of the total volume of the object above which splits will still occur.
						bool  useInitialIslandGeneration=true,	///< Whether or not to perform initial island generation on the input mesh.
						bool  useIslandGeneration=false		///< Whether or not to perform island generation at each split.  Currently disabled.
											 );


		/// Get the number of computed hulls after the convex decomposition
	virtual unsigned int GetHullCount();

		/// Get the n-th computed convex hull, by accessing the ConvexHullResult structure
		/// from the wrapped convex decomposition code (in convexdecomp/ dir)
	virtual bool GetConvexHullResult(unsigned int hullIndex, CONVEX_DECOMPOSITION::ConvexHullResult& result);


		/// Get the n-th computed convex hull, by filling a ChTriangleMesh object
		/// that is passed as a parameter.
	virtual bool GetConvexHullResult(unsigned int hullIndex, ChTriangleMesh& convextrimesh);



	//
	// SERIALIZATION
	//

		/// Write the convex decomposition to a ".chulls" file,
		/// where each hull is a sequence of x y z coords. Can throw exceptions.
	virtual bool WriteConvexHullsAsChullsFile(ChStreamOutAscii& mstream);


		/// Save the computed convex hulls as a Wavefront file using the
		/// '.obj' fileformat, with each hull as a separate group. 
		/// May throw exceptions if file locked etc.
	virtual void WriteConvexHullsAsWavefrontObj(ChStreamOutAscii& mstream);


	//
	// DATA
	//

private:
	CONVEX_DECOMPOSITION::iConvexDecomposition* mydecomposition;

};




} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

#endif
