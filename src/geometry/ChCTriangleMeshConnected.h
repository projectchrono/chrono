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

#ifndef CHC_TRIANGLEMESHCONNECTED_H
#define CHC_TRIANGLEMESHCONNECTED_H

//////////////////////////////////////////////////
//  
//   ChCTriangleMeshConnected.h
//
//   Triangle mesh in 3d, with shared vertices
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



#define CH_GEOCLASS_TRIANGLEMESHCONNECTED   11



/// A triangle mesh with connectivity info: vertices can be 
/// shared between faces.

class ChApi ChTriangleMeshConnected : public ChTriangleMesh
{
					// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChTriangleMeshConnected,ChTriangleMesh);

		// 
		// DATA
		// 

	std::vector< ChVector<double> >	m_vertices;
	std::vector< ChVector<double> >	m_normals;
	std::vector< ChVector<double> >	m_UV;
	std::vector< ChVector<float> >	m_colors;

	std::vector< ChVector<int> >	m_face_v_indices;
	std::vector< ChVector<int> >	m_face_n_indices;
	std::vector< ChVector<int> >	m_face_u_indices;
	std::vector< ChVector<int> >	m_face_col_indices;
	
	   //file string if loading an obj file.
	std::string m_filename;

public:
	ChTriangleMeshConnected () {
	   m_filename = "";
	};


		//
		// CUSTOM FUNCTIONS
		//

	std::vector< ChVector<double> >& getCoordsVertices() {return m_vertices;}
	std::vector< ChVector<double> >& getCoordsNormals()  {return m_normals;}
	std::vector< ChVector<double> >& getCoordsUV()	     {return m_UV;}
	std::vector< ChVector<float > >& getCoordsColors()	 {return m_colors;}

	std::vector< ChVector<int> >&	getIndicesVertexes() {return m_face_v_indices;}
	std::vector< ChVector<int> >&	getIndicesNormals() {return m_face_n_indices;}
	std::vector< ChVector<int> >&	getIndicesUV() {return m_face_u_indices;}
	std::vector< ChVector<int> >&	getIndicesColors() {return m_face_col_indices;}

		// Load a triangle mesh saved as a Wavefront .obj file
	void LoadWavefrontMesh(std::string filename, bool load_normals = true, bool load_uv = false);


		//
		// MESH INTERFACE FUNCTIONS
		//

			/// Add a triangle to this triangle mesh, by specifying the three coordinates.
			/// This is disconnected - no vertex sharing is used even if it could be..
	virtual void addTriangle(const ChVector<>& vertex0, const ChVector<>& vertex1, const ChVector<>& vertex2)
	{
		size_t base_v = m_vertices.size();
		m_vertices.push_back(vertex0);
		m_vertices.push_back(vertex1);
		m_vertices.push_back(vertex2);
		m_face_v_indices.push_back(ChVector<int>(base_v, base_v+1, base_v+2));
	}
			/// Add a triangle to this triangle mesh, by specifying a ChTriangle 
	virtual void addTriangle(const ChTriangle& atriangle)
	{
		size_t base_v = m_vertices.size();
		m_vertices.push_back(atriangle.p1);
		m_vertices.push_back(atriangle.p2);
		m_vertices.push_back(atriangle.p3);
		m_face_v_indices.push_back(ChVector<int>(base_v, base_v+1, base_v+2));
	}

			/// Get the number of triangles already added to this mesh
	virtual int getNumTriangles() const
	{
		return m_face_v_indices.size();
	}

			/// Access the n-th triangle in mesh
	virtual ChTriangle getTriangle(int index) const
	{
		return ChTriangle( m_vertices[m_face_v_indices[index].x],  m_vertices[m_face_v_indices[index].y],  m_vertices[m_face_v_indices[index].z] );
	}

			/// Clear all data
	virtual void Clear()
	{
		this->getCoordsVertices().clear();
		this->getCoordsNormals().clear();
		this->getCoordsUV().clear();
		this->getCoordsColors().clear();
		this->getIndicesVertexes().clear();
		this->getIndicesNormals().clear();
		this->getIndicesNormals().clear();
		this->getIndicesColors().clear();
	}

			/// Compute barycenter, mass, inertia tensor
	void ComputeMassProperties (bool bodyCoords, double& mass, ChVector<>& center, ChMatrix33<>& inertia);


		//
		// OVERRIDE BASE CLASS FUNCTIONS
		//

	virtual int GetClassType () {return CH_GEOCLASS_TRIANGLEMESHCONNECTED;};

		//
		// STREAMING
		//

	//void StreamOUT(ChStreamOutBinary& mstream);//***TO DO***

	//void StreamIN(ChStreamInBinary& mstream); //***TO DO***

};



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif
