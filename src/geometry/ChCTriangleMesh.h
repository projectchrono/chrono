#ifndef CHC_TRIANGLEMESH_H
#define CHC_TRIANGLEMESH_H

//////////////////////////////////////////////////
//  
//   ChCTriangleMesh.h
//
//   Basic triangle mesh in 3d.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <math.h>

#include "ChCTriangle.h"



namespace chrono
{
namespace geometry 
{



#define CH_GEOCLASS_TRIANGLEMESH   9


///
/// A basic triangle mesh: just a list of triangles (no edge connectivity info).
///

class ChTriangleMesh : public ChGeometry
{
					// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChTriangleMesh,ChGeometry);

		// 
		// DATA
		// 

	std::vector<ChTriangle>	m_triangles;
	

public:
	ChTriangleMesh () {};


		//
		// CUSTOM FUNCTIONS
		//

			/// Add a triangle to this triangle mesh, by specifying the three coordinates
	void addTriangle(const ChVector<>& vertex0, const ChVector<>& vertex1, const ChVector<>& vertex2)
	{
		ChTriangle tri(vertex0,vertex1,vertex2);
		m_triangles.push_back(tri);
	}
			/// Add a triangle to this triangle mesh, by specifying a ChTriangle 
	void addTriangle(const ChTriangle& atriangle)
	{
		m_triangles.push_back(atriangle);
	}

			/// Get the number of triangles already added to this mesh
	int getNumTriangles() const
	{
		return m_triangles.size();
	}

			/// Access the n-th triangle in mesh
	ChTriangle&	getTriangle(int index) 
	{
		return m_triangles[index];
	}

			/// Access the n-th triangle in mesh
	const ChTriangle&	getTriangle(int index) const
	{
		return m_triangles[index];
	}


		//
		// OVERRIDE BASE CLASS FUNCTIONS
		//

	virtual int GetClassType () {return CH_GEOCLASS_TRIANGLEMESH;};

	/*
	virtual void GetBoundingBox(double& xmin, double& xmax, 
					    double& ymin, double& ymax, 
						double& zmin, double& zmax, 
						ChMatrix33<>* Rot = NULL) { }; //***TO DO***
	
	virtual Vector Baricenter();//***TO DO***
	virtual void CovarianceMatrix(ChMatrix33<>& C);//***TO DO***
    */
				/// This is a surface
	virtual int GetManifoldDimension() {return 2;}



		//
		// STREAMING
		//

	//void StreamOUT(ChStreamOutBinary& mstream);//***TO DO***

	//void StreamIN(ChStreamInBinary& mstream); //***TO DO***

};



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif
