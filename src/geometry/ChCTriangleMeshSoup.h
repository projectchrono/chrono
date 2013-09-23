//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHC_TRIANGLEMESHSOUP_H
#define CHC_TRIANGLEMESHSOUP_H

//////////////////////////////////////////////////
//  
//   ChCTriangleMeshSoup.h
//
//   Basic triangle mesh with disconnected triangles
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <math.h>

#include "ChCTriangleMesh.h"



namespace chrono
{
namespace geometry 
{



#define CH_GEOCLASS_TRIANGLEMESHSOUP   12


///
/// A basic triangle mesh: just a list of triangles (no edge connectivity info).
///

class ChApi ChTriangleMeshSoup : public ChTriangleMesh
{
					// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChTriangleMeshSoup,ChTriangleMesh);

		// 
		// DATA
		// 

	std::vector<ChTriangle>	m_triangles;
	

public:
	ChTriangleMeshSoup () {};


				/// Access the n-th triangle in mesh
	virtual ChTriangle& Triangle(int index)
	{
		return m_triangles[index];
	}


		//
		// MESH INTERFACE FUNCTIONS
		//

			/// Add a triangle to this triangle mesh, by specifying the three coordinates
	virtual void  addTriangle(const ChVector<>& vertex0, const ChVector<>& vertex1, const ChVector<>& vertex2)
	{
		ChTriangle tri(vertex0,vertex1,vertex2);
		m_triangles.push_back(tri);
	}
			/// Add a triangle to this triangle mesh, by specifying a ChTriangle 
	virtual void addTriangle(const ChTriangle& atriangle)
	{
		m_triangles.push_back(atriangle);
	}

			/// Get the number of triangles already added to this mesh
	virtual int getNumTriangles() const
	{
		return m_triangles.size();
	}

			/// Access the n-th triangle in mesh
	virtual ChTriangle getTriangle(int index) const
	{
		return m_triangles[index];
	}


		//
		// OVERRIDE BASE CLASS FUNCTIONS
		//

	virtual int GetClassType () {return CH_GEOCLASS_TRIANGLEMESHSOUP;};

		//
		// STREAMING
		//

	//void StreamOUT(ChStreamOutBinary& mstream);//***TO DO***

	//void StreamIN(ChStreamInBinary& mstream); //***TO DO***

};



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif
