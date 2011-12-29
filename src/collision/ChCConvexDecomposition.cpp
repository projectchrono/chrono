//////////////////////////////////////////////////
//  
//   ChCConvexDecomposition.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
   
 
#include "collision/ChCConvexDecomposition.h"


namespace chrono 
{
namespace collision 
{


	/// Basic constructor
ChConvexDecomposition::ChConvexDecomposition()	
		{
		}

	/// Destructor
ChConvexDecomposition::~ChConvexDecomposition()
		{
		}

bool ChConvexDecomposition::AddTriangle(const ChTriangle& t1)
		{
			return this->AddTriangle(t1.p1, t1.p2, t1.p3); // add the input mesh one triangle at a time.
		}

bool ChConvexDecomposition::AddTriangleMesh(const ChTriangleMesh& tm)
		{
			for (int i= 0; i< tm.getNumTriangles(); i++)
			{
				if (!this->AddTriangle( tm.getTriangle(i) )) return false;
			}
			return true;
		}

bool ChConvexDecomposition::WriteConvexHullsAsChullsFile(ChStreamOutAscii& mstream)
		{
			mstream.SetNumFormat("%0.9f");
			mstream <<"# Convex hulls obtained with Chrono::Engine \n# convex decomposition (.chulls format: only vertexes)\n";

			for (unsigned int ih = 0; ih < this->GetHullCount(); ih++)
			{
				std::vector< ChVector<double> > aconvexhull;
				
				if (!this->GetConvexHullResult(ih, aconvexhull)) return false;

				mstream <<"hull\n";
				for (unsigned int i=0; i<aconvexhull.size(); i++)
				{
					mstream << aconvexhull[i].x << " " << aconvexhull[i].y << " " << aconvexhull[i].z << "\n";
				}
			}
			return true;
		}




/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

//
//  ChConvexDecompositionHACD
// 


	/// Basic constructor
ChConvexDecompositionHACD::ChConvexDecompositionHACD()	
		{
			myHACD = HACD::CreateHACD();
		}

	/// Destructor
ChConvexDecompositionHACD::~ChConvexDecompositionHACD()
		{
			if (myHACD) HACD::DestroyHACD(myHACD); myHACD=0;
		}


void ChConvexDecompositionHACD::Reset(void)
		{ 
			if (myHACD) HACD::DestroyHACD(myHACD); myHACD=0;
			myHACD = HACD::CreateHACD();
			this->points.clear();
			this->triangles.clear();
		}

bool ChConvexDecompositionHACD::AddTriangle(const ChVector<>& v1,const ChVector<>& v2,const ChVector<>& v3)
		{
			int lastpoint = this->points.size();
			HACD::Vec3<HACD::Real> vertex1(v1.x,v1.y,v1.z);
			HACD::Vec3<HACD::Real> vertex2(v2.x,v2.y,v2.z);
			HACD::Vec3<HACD::Real> vertex3(v3.x,v3.y,v3.z);
			this->points.push_back(vertex1);
			this->points.push_back(vertex2);
			this->points.push_back(vertex3);
			HACD::Vec3<long> newtri(lastpoint,lastpoint+1,lastpoint+2);
			this->triangles.push_back(newtri);
			return true;
		}

bool ChConvexDecompositionHACD::AddTriangleMesh(const ChTriangleMesh& tm)
		{
			for (int i= 0; i< tm.getNumTriangles(); i++)
			{
				if (!this->ChConvexDecomposition::AddTriangle( tm.getTriangle(i) )) return false;
			}
			return true;
		}

void ChConvexDecompositionHACD::SetParameters( 
					unsigned int nClusters,
					unsigned int targetDecimation,
					double smallClusterThreshold,
					bool  addFacesPoints,
					bool  addExtraDistPoints,
					double concavity,				
					double ccConnectDist,				
					double volumeWeight,				
					double compacityAlpha,				
					unsigned int nVerticesPerCH	
										 )
		{
			myHACD->SetNClusters(nClusters);
			myHACD->SetNTargetTrianglesDecimatedMesh(targetDecimation);
			myHACD->SetSmallClusterThreshold(smallClusterThreshold);
			myHACD->SetAddFacesPoints(addFacesPoints);
			myHACD->SetAddExtraDistPoints(addExtraDistPoints);
			myHACD->SetConcavity(concavity);
			myHACD->SetConnectDist(ccConnectDist);
			myHACD->SetVolumeWeight(volumeWeight);
			myHACD->SetCompacityWeight(compacityAlpha);
			myHACD->SetNVerticesPerCH(nVerticesPerCH);
		}

int ChConvexDecompositionHACD::ComputeConvexDecomposition()
		{
			myHACD->SetPoints(&this->points[0]);
			myHACD->SetNPoints(points.size());
			myHACD->SetTriangles(&this->triangles[0]);
			myHACD->SetNTriangles(triangles.size());

			myHACD->Compute();

			return myHACD->GetNClusters();
		}

	/// Get the number of computed hulls after the convex decomposition
unsigned int ChConvexDecompositionHACD::GetHullCount() 
		{ 
			return this->myHACD->GetNClusters(); 
		}


bool ChConvexDecompositionHACD::GetConvexHullResult(unsigned int hullIndex, std::vector< ChVector<double> >& convexhull)
		{
			if (hullIndex > myHACD->GetNClusters()) 
				return false;

			size_t nPoints = myHACD->GetNPointsCH(hullIndex);
			size_t nTriangles = myHACD->GetNTrianglesCH(hullIndex);

			float* vertices = new float[nPoints*3];
			unsigned int* triangles = new unsigned int[nTriangles*3];
			
			HACD::Vec3<HACD::Real> * pointsCH = new HACD::Vec3<HACD::Real>[nPoints];
			HACD::Vec3<long> * trianglesCH = new HACD::Vec3<long>[nTriangles];
			myHACD->GetCH(hullIndex, pointsCH, trianglesCH);
			
			// convert to chrono data...
			convexhull.clear();
			for (unsigned int i=0; i<nPoints; i++)
			{
				ChVector<double> point( pointsCH[i].X(), pointsCH[i].Y(), pointsCH[i].Z() );
				convexhull.push_back(point);
			}
			
			delete[] pointsCH;
			delete[] trianglesCH;

			return true;
		}

	/// Get the n-th computed convex hull, by filling a ChTriangleMesh object
	/// that is passed as a parameter.
bool ChConvexDecompositionHACD::GetConvexHullResult(unsigned int hullIndex, ChTriangleMesh& convextrimesh)
		{
			if (hullIndex > myHACD->GetNClusters()) 
				return false;

			size_t nPoints = myHACD->GetNPointsCH(hullIndex);
			size_t nTriangles = myHACD->GetNTrianglesCH(hullIndex);

			float* vertices = new float[nPoints*3];
			unsigned int* triangles = new unsigned int[nTriangles*3];
			
			HACD::Vec3<HACD::Real> * pointsCH = new HACD::Vec3<HACD::Real>[nPoints];
			HACD::Vec3<long> * trianglesCH = new HACD::Vec3<long>[nTriangles];
			myHACD->GetCH(hullIndex, pointsCH, trianglesCH);

			for (unsigned int i=0; i<nTriangles; i++)
			{
				unsigned int i1 = trianglesCH[i].X();
				unsigned int i2 = trianglesCH[i].Y();
				unsigned int i3 = trianglesCH[i].Z();
				convextrimesh.addTriangle(
					ChVector<>(pointsCH[i1].X(), pointsCH[i1].Y(), pointsCH[i1].Z()),
					ChVector<>(pointsCH[i2].X(), pointsCH[i2].Y(), pointsCH[i2].Z()),
					ChVector<>(pointsCH[i3].X(), pointsCH[i3].Y(), pointsCH[i3].Z()) );
			}
			
			delete[] pointsCH;
			delete[] trianglesCH;

			return true;
		}



//
// SERIALIZATION
//

void ChConvexDecompositionHACD::WriteConvexHullsAsWavefrontObj(ChStreamOutAscii& mstream)
		{
			mstream << "# Convex hulls obtained with Chrono::Engine \n# convex decomposition \n\n";
			NxU32 vcount_base = 1;
			NxU32 vcount_total = 0;
			NxU32 tcount_total = 0;
			char buffer[200];
			for (unsigned int hullIndex=0; hullIndex< this->GetHullCount(); hullIndex++)
			{
				mstream << "g hull_" << hullIndex << "\n";

				size_t nPoints = myHACD->GetNPointsCH(hullIndex);
				size_t nTriangles = myHACD->GetNTrianglesCH(hullIndex);

				float* vertices = new float[nPoints*3];
				unsigned int* triangles = new unsigned int[nTriangles*3];
				
				HACD::Vec3<HACD::Real> * pointsCH = new HACD::Vec3<HACD::Real>[nPoints];
				HACD::Vec3<long> * trianglesCH = new HACD::Vec3<long>[nTriangles];
				myHACD->GetCH(hullIndex, pointsCH, trianglesCH);

				vcount_total+=nPoints;
				tcount_total+=nTriangles;
				for (unsigned int i=0; i<nPoints; i++)
				{
					sprintf(buffer,"v %0.9f %0.9f %0.9f\r\n", pointsCH[i].X(), pointsCH[i].Y(), pointsCH[i].Z() );
					mstream << buffer;
				}
				for (unsigned int i=0; i<nTriangles; i++)
				{
					unsigned int i1 = trianglesCH[i].X();
					unsigned int i2 = trianglesCH[i].Y();
					unsigned int i3 = trianglesCH[i].Z();
					sprintf(buffer,"f %d %d %d\r\n", i1+vcount_base, i2+vcount_base, i3+vcount_base );
					mstream << buffer;
				}
				vcount_base+=nPoints;

				delete[] pointsCH;
				delete[] trianglesCH;
			}
		}



/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

//
//  ChConvexDecompositionJR
// 


//CONVEX_DECOMPOSITION::iConvexDecomposition* ChConvexDecompositionJR::GetDecompositionObject() {return this->mydecomposition;}

ChConvexDecompositionJR::ChConvexDecompositionJR()	
		{
			mydecomposition = CONVEX_DECOMPOSITION::createConvexDecomposition();
			skinWidth=0;
			decompositionDepth=8;
			maxHullVertices=64;
			concavityThresholdPercent=0.1f;
			mergeThresholdPercent=30.0f;
			volumeSplitThresholdPercent=0.1f;
			useInitialIslandGeneration=true;
			useIslandGeneration=false;
		}

ChConvexDecompositionJR::~ChConvexDecompositionJR()
		{
			CONVEX_DECOMPOSITION::releaseConvexDecomposition(mydecomposition);
		}

void ChConvexDecompositionJR::Reset(void) 
		{ 
			this->mydecomposition->reset();
		}

bool ChConvexDecompositionJR::AddTriangle(const ChVector<>& v1,const ChVector<>& v2,const ChVector<>& v3)
		{
			NxF32 p1[3]; p1[0]=(float)v1.x; p1[1]=(float)v1.y; p1[2]=(float)v1.z;
			NxF32 p2[3]; p2[0]=(float)v2.x; p2[1]=(float)v2.y; p2[2]=(float)v2.z;
			NxF32 p3[3]; p3[0]=(float)v3.x; p3[1]=(float)v3.y; p3[2]=(float)v3.z;
			return this->mydecomposition->addTriangle(p1,p2,p3); 
		}

bool ChConvexDecompositionJR::AddTriangleMesh(const ChTriangleMesh& tm)
		{
			for (int i= 0; i< tm.getNumTriangles(); i++)
			{
				if (!this->ChConvexDecomposition::AddTriangle( tm.getTriangle(i) )) return false;
			}
			return true;
		}

void ChConvexDecompositionJR::SetParameters( 
					float mskinWidth,					///< Skin width on the convex hulls generated
					unsigned int mdecompositionDepth,	///< Recursion depth for convex decomposition.
					unsigned int mmaxHullVertices,	///< Maximum number of vertices in output convex hulls.
					float mconcavityThresholdPercent, ///< The percentage of concavity allowed without causing a split to occur.
					float mmergeThresholdPercent,      ///< The percentage of volume difference allowed to merge two convex hulls.
					float mvolumeSplitThresholdPercent, ///< The percentage of the total volume of the object above which splits will still occur.
					bool  museInitialIslandGeneration,	///< Whether or not to perform initial island generation on the input mesh.
					bool  museIslandGeneration		///< Whether or not to perform island generation at each split.  Currently disabled.
										 )
		{
			skinWidth= mskinWidth;
			decompositionDepth=mdecompositionDepth;
			maxHullVertices=mmaxHullVertices;
			concavityThresholdPercent=mconcavityThresholdPercent;
			mergeThresholdPercent=mmergeThresholdPercent;
			volumeSplitThresholdPercent=mvolumeSplitThresholdPercent;
			useInitialIslandGeneration=museInitialIslandGeneration;
			useIslandGeneration=museIslandGeneration;
		}

int ChConvexDecompositionJR::ComputeConvexDecomposition()
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
unsigned int ChConvexDecompositionJR::GetHullCount() { return this->mydecomposition->getHullCount(); }


	/// Get the n-th computed convex hull, by filling a ChTriangleMesh object
	/// that is passed as a parameter.
bool ChConvexDecompositionJR::GetConvexHullResult(unsigned int hullIndex, ChTriangleMesh& convextrimesh)
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

bool ChConvexDecompositionJR::GetConvexHullResult(unsigned int hullIndex, std::vector< ChVector<double> >& convexhull)
		{
			CONVEX_DECOMPOSITION::ConvexHullResult result;
			if (!this->mydecomposition->getConvexHullResult(hullIndex, result)) return false;
			
			// convert to chrono data...
			convexhull.clear();
			for (unsigned int i=0; i<result.mVcount; i++)
			{
				ChVector<double> point( result.mVertices[i*3+0], result.mVertices[i*3+1], result.mVertices[i*3+2] );
				convexhull.push_back(point);	
			}

			return true;
		}

//
// SERIALIZATION
//

	/// Save the computed convex hulls as a Wavefront file using the
	/// '.obj' fileformat, with each hull as a separate group. 
	/// May throw exceptions if file locked etc.
void ChConvexDecompositionJR::WriteConvexHullsAsWavefrontObj(ChStreamOutAscii& mstream)
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
				this->mydecomposition->getConvexHullResult(i,result);
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







} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


