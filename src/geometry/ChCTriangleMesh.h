//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010, 2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHC_TRIANGLEMESH_H
#define CHC_TRIANGLEMESH_H

//////////////////////////////////////////////////
//  
//   ChCTriangleMesh.h
//
//   Basic interface for triangle meshes in 3d.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
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

class ChApi ChTriangleMesh : public ChGeometry
{
					// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChTriangleMesh,ChGeometry);

		// 
		// DATA
		// 

	//std::vector<ChTriangle>	m_triangles;
	

public:
	ChTriangleMesh () {};


		//
		// MESH INTERFACE FUNCTIONS
		//

			/// Add a triangle to this triangle mesh, by specifying the three coordinates
	virtual void addTriangle(const ChVector<>& vertex0, const ChVector<>& vertex1, const ChVector<>& vertex2) = 0;
	
			/// Add a triangle to this triangle mesh, by specifying a ChTriangle 
	virtual void addTriangle(const ChTriangle& atriangle) = 0;

			/// Get the number of triangles already added to this mesh
	virtual int getNumTriangles() const = 0;

			/// Get the n-th triangle in mesh
	virtual ChTriangle getTriangle(int index) const = 0;

			/// Clear all data
	virtual void Clear() = 0;


		//
		// OVERRIDE BASE CLASS FUNCTIONS
		//

	virtual int GetClassType () {return CH_GEOCLASS_TRIANGLEMESH;};

	/*
	virtual void GetBoundingBox(double& xmin, double& xmax, 
					    double& ymin, double& ymax, 
						double& zmin, double& zmax, 
						ChMatrix33<>* Rot = NULL) { }; //TODO
	
	virtual Vector Baricenter();//TODO
	virtual void CovarianceMatrix(ChMatrix33<>& C);//TODO
    */
				/// This is a surface
	virtual int GetManifoldDimension() {return 2;}



		//
		// STREAMING
		//

	//void StreamOUT(ChStreamOutBinary& mstream);//TODO

	//void StreamIN(ChStreamInBinary& mstream); //TODO

};



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif
