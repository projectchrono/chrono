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
	ChConvexDecomposition()	
			{
				mydecomposition = CONVEX_DECOMPOSITION::createConvexDecomposition();
			}

		/// Destructor
	virtual ~ChConvexDecomposition()
			{
				CONVEX_DECOMPOSITION::releaseConvexDecomposition(mydecomposition);
			}


		/// Access directly the wrapped J.W.Ratcliff convex decomposition object 
		/// although it shouldn't be necessary, since this class already provide
		/// wrapping to all the functions).
	CONVEX_DECOMPOSITION::iConvexDecomposition* GetDecompositionObject() {return this->mydecomposition;}

		/// Reset the input mesh data
	virtual void Reset(void) { this->mydecomposition->reset();}

		/// Add a triangle, by passing three points for vertexes. 
		/// Note: the vertexes must be properly ordered (oriented triangle, normal pointing outside)
	virtual bool AddTriangle(const ChVector<>& v1,const ChVector<>& v2,const ChVector<>& v3)
			{
				NxF32 p1[3]; p1[0]=(float)v1.x; p1[1]=(float)v1.y; p1[2]=(float)v1.z;
				NxF32 p2[3]; p2[0]=(float)v2.x; p2[1]=(float)v2.y; p2[2]=(float)v2.z;
				NxF32 p3[3]; p3[0]=(float)v3.x; p3[1]=(float)v3.y; p3[2]=(float)v3.z;
				return this->mydecomposition->addTriangle(p1,p2,p3); // add the input mesh one triangle at a time.
			}

		/// Add a triangle, by passing a  ChTriangle object (that will be copied, not referenced). 
		/// Note: the vertexes must be properly ordered (oriented triangle, normal pointing outside)
	virtual bool AddTriangle(const ChTriangle& t1)
			{
				return this->AddTriangle(t1.p1, t1.p2, t1.p3); // add the input mesh one triangle at a time.
			}

		/// Add a triangle mesh soup, by passing an entire ChTriangleMesh object.
		/// Note 1: the triangle mesh does not need connectivity information (a basic 'triangle soup' is enough)
		/// Note 2: all vertexes must be properly ordered (oriented triangles, normals pointing outside).
		/// Note 3: the triangles must define closed volumes (holes, gaps in edges, etc. may trouble the decomposition)
	virtual bool AddTriangleMesh(const ChTriangleMesh& tm)
			{
				for (int i= 0; i< tm.getNumTriangles(); i++)
				{
					if (!this->AddTriangle( tm.getTriangle(i) )) return false;
				}
				return true;
			}

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
											 )
			{
				return this->mydecomposition->computeConvexDecomposition(skinWidth,
											 decompositionDepth, 
											 maxHullVertices,
											 concavityThresholdPercent,
											 mergeThresholdPercent,
											 volumeSplitThresholdPercent,
											 useInitialIslandGeneration,
											 useIslandGeneration,
											 false);
			}

		/// Get the number of computed hulls after the convex decomposition
	virtual unsigned int GetHullCount() { return this->mydecomposition->getHullCount(); }

		/// Get the n-th computed convex hull, by accessing the ConvexHullResult structure
		/// from the wrapped convex decomposition code (in convexdecomp/ dir)
	virtual bool GetConvexHullResult(unsigned int hullIndex, CONVEX_DECOMPOSITION::ConvexHullResult& result)
			{
				return this->mydecomposition->getConvexHullResult(hullIndex, result);
			}

		/// Get the n-th computed convex hull, by filling a ChTriangleMesh object
		/// that is passed as a parameter.
	virtual bool GetConvexHullResult(unsigned int hullIndex, ChTriangleMesh& convextrimesh)
			{
				CONVEX_DECOMPOSITION::ConvexHullResult result;
				if (!this->mydecomposition->getConvexHullResult(hullIndex, result)) return false;
				
				for (unsigned int i=0; i<result.mTcount; i++)
				{
						unsigned int i1 = 3*result.mIndices[i*3+0];
						unsigned int i2 = 3*result.mIndices[i*3+1];
						unsigned int i3 = 3*result.mIndices[i*3+2];
						convextrimesh.addTriangle(
							ChVector<>(result.mVertices[i1+0], result.mVertices[i1+1], result.mVertices[i1+2]),
							ChVector<>(result.mVertices[i2+0], result.mVertices[i2+1], result.mVertices[i2+2]),
							ChVector<>(result.mVertices[i3+0], result.mVertices[i3+1], result.mVertices[i3+2]) );
				}
				return true;
			}



	//
	// SERIALIZATION
	//

		/// Write the convex decomposition to a ".chulls" file,
		/// where each hull is a sequence of x y z coords. Can throw exceptions.
	virtual bool WriteConvexHullsAsChullsFile(ChStreamOutAscii& mstream)
			{
				mstream.SetNumFormat("%0.9f");
				mstream <<"# Convex hulls obtained with Chrono::Engine \n# convex decomposition (.chulls format: only vertexes)\n";
				for (int ih = 0; ih < this->GetHullCount(); ih++)
				{
					CONVEX_DECOMPOSITION::ConvexHullResult result;
					if (!this->mydecomposition->getConvexHullResult(ih, result)) return false;
					mstream <<"hull\n";
					for (unsigned int i=0; i<result.mVcount; i++)
					{
						mstream << (double)result.mVertices[3*i+0] << " " << (double)result.mVertices[3*i+1] << " " << (double)result.mVertices[3*i+2] << "\n";
					}
				}
				return true;
			}

		/// Save the computed convex hulls as a Wavefront file using the
		/// '.obj' fileformat, with each hull as a separate group. 
		/// May throw exceptions if file locked etc.
	virtual void WriteConvexHullsAsWavefrontObj(ChStreamOutAscii& mstream)
			{
				mstream << "# Convex hulls obtained with Chrono::Engine \n# convex decomposition \n\n";
				NxU32 vcount_base = 1;
				NxU32 vcount_total = 0;
				NxU32 tcount_total = 0;
				char buffer[200];
				for (NxU32 i=0; i< this->GetHullCount(); i++)
				{
					mstream << "g hull_" << i << "\n";
					CONVEX_DECOMPOSITION::ConvexHullResult result;
					this->GetDecompositionObject()->getConvexHullResult(i,result);
					vcount_total+=result.mVcount;
					tcount_total+=result.mTcount;
					for (NxU32 i=0; i<result.mVcount; i++)
					{
						const NxF32 *pos = &result.mVertices[i*3];
						sprintf(buffer,"v %0.9f %0.9f %0.9f\r\n", pos[0], pos[1], pos[2] );
						mstream << buffer;
					}
					for (NxU32 i=0; i<result.mTcount; i++)
					{
						NxU32 i1 = result.mIndices[i*3+0];
						NxU32 i2 = result.mIndices[i*3+1];
						NxU32 i3 = result.mIndices[i*3+2];
						sprintf(buffer,"f %d %d %d\r\n", i1+vcount_base, i2+vcount_base, i3+vcount_base );
						mstream << buffer;
					}
					vcount_base+=result.mVcount;
				}
			}


	//
	// DATA
	//

private:
	CONVEX_DECOMPOSITION::iConvexDecomposition* mydecomposition;

};




} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

#endif
