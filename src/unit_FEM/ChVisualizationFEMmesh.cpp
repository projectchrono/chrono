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
// File author: A. Tasora
  

#include "unit_FEM/ChVisualizationFEMmesh.h"
#include "unit_FEM/ChElementTetra_4.h"
#include "assets/ChTriangleMeshShape.h"



namespace chrono
{
namespace fem
{


ChVisualizationFEMmesh::ChVisualizationFEMmesh(ChMesh& mymesh) 
{
	FEMmesh = &mymesh;
	fem_data_type = E_PLOT_NODE_DISP_NORM;

	colorscale_min= 0;
	colorscale_max= 1;

	ChSharedPtr<ChTriangleMeshShape> new_mesh_asset(new ChTriangleMeshShape);
	this->AddAsset(new_mesh_asset);
}

ChVector<float> ChVisualizationFEMmesh::ComputeFalseColor(double mv)
{   
	if (mv < this->colorscale_min)
		return ChVector<float> (0,0,0) ;
	if (mv > this->colorscale_max)
		return ChVector<float> (1,1,1) ;

	ChVector<float> c(1,1,1);
	float dv;
	float v = (float)mv;
	float vmax = (float)this->colorscale_max;
	float vmin = (float)this->colorscale_min;

	dv = vmax - vmin;

	if (v < (vmin + 0.25 * dv)) {
		c.x = 0.f;
		c.y = 4.f * (v - vmin) / dv;
	} else if (v < (vmin + 0.5 * dv)) {
		c.x = 0;
		c.z = 1.f + 4.f * (vmin + 0.25f * dv - v) / dv;
	} else if (v < (vmin + 0.75f * dv)) {
		c.x = 4.f * (v - vmin - 0.5f * dv) / dv;
		c.z = 0;
	} else {
		c.y = 1.f + 4.f * (vmin + 0.75f * dv - v) / dv;
		c.z = 0;
	}

	return(c);
}

double ChVisualizationFEMmesh::ComputeScalarOutput( ChNodeFEMxyz* mnode, ChElementBase* melement)
{
	switch (this->fem_data_type)
	{
	case E_PLOT_NODE_DISP_NORM:
		return (mnode->GetPos()-mnode->GetX0()).Length();
	case E_PLOT_NODE_DISP_X:
		return (mnode->GetPos()-mnode->GetX0()).x;
	case E_PLOT_NODE_DISP_Y:
		return (mnode->GetPos()-mnode->GetX0()).y;
	case E_PLOT_NODE_DISP_Z:
		return (mnode->GetPos()-mnode->GetX0()).z;
	case E_PLOT_NODE_SPEED_NORM:
		return mnode->GetPos_dt().Length();
	case E_PLOT_NODE_SPEED_X:
		return mnode->GetPos_dt().x;
	case E_PLOT_NODE_SPEED_Y:
		return mnode->GetPos_dt().y;
	case E_PLOT_NODE_SPEED_Z:
		return mnode->GetPos_dt().z;
	case E_PLOT_NODE_ACCEL_NORM:
		return mnode->GetPos_dtdt().Length();
	case E_PLOT_NODE_ACCEL_X:
		return mnode->GetPos_dtdt().x;
	case E_PLOT_NODE_ACCEL_Y:
		return mnode->GetPos_dtdt().y;
	case E_PLOT_NODE_ACCEL_Z:
		return mnode->GetPos_dtdt().z;
	default:
		return 0;
	}
	//***TO DO*** other types of scalar outputs
	return 0;
}

ChVector<float>& FetchOrAllocate(std::vector< ChVector<float > >& mvector, unsigned int& id)
{
	if (id > mvector.size())
	{
		id = 0;
		return mvector[0]; // error
	}
	if (id ==  mvector.size())
	{
		mvector.push_back( ChVector<float>(0,0,0) );
	}
	++id;
	return mvector[id-1];
}

void ChVisualizationFEMmesh::Update ()
{
	if (!this->FEMmesh) 
		return;

	ChSharedPtr<ChTriangleMeshShape> mesh_asset;

	// try to retrieve previously added mesh asset in sublevel..
	if (this->GetAssets().size() == 1)
		if (GetAssets()[0].IsType<ChTriangleMeshShape>() )
			mesh_asset = GetAssets()[0];

	// if not available, create it...
	if (mesh_asset.IsNull())
	{
		ChSharedPtr<ChTriangleMeshShape> new_mesh_asset(new ChTriangleMeshShape);
		this->AddAsset(new_mesh_asset);
		mesh_asset = new_mesh_asset;
	}
	geometry::ChTriangleMeshConnected& trianglemesh = mesh_asset->GetMesh();

	unsigned int n_verts = 0;
	unsigned int n_vcols = 0;
	unsigned int n_index = 0;

	//
	// A - Count the needed vertexes and faces
	//

	for (unsigned int iel=0; iel < this->FEMmesh->GetNelements(); ++iel)
	{
		ChElementBase* myel =  this->FEMmesh->GetElement(iel);

		// ELEMENT IS A TETRAHEDRON
		if (ChElementTetra_4* mytetra = dynamic_cast<ChElementTetra_4*>(myel))
		{
			n_verts +=4;
			n_vcols +=4;
			n_index +=4;
		}

		// ELEMENT IS A HEXAEDRON

		//***TO DO*** other types of elements...

	}

	//
	// B - resize mesh buffers if needed
	//

	if (trianglemesh.getCoordsVertices().size() != n_verts)
		trianglemesh.getCoordsVertices().resize(n_verts);
	if (trianglemesh.getCoordsColors().size() != n_vcols)
		trianglemesh.getCoordsColors().resize(n_vcols);
	if (trianglemesh.getIndicesVertexes().size() != n_index)
		trianglemesh.getIndicesVertexes().resize(n_index);
	if (trianglemesh.getIndicesColors().size() != n_index)
		trianglemesh.getIndicesColors().resize(n_index);

	//
	// C - update mesh buffers 
	//

	unsigned int i_verts = 0;
	unsigned int i_vcols = 0;
	unsigned int i_index = 0;
	unsigned int i_colindex = 0;

	for (unsigned int iel=0; iel < this->FEMmesh->GetNelements(); ++iel)
	{
		ChElementBase* myel =  this->FEMmesh->GetElement(iel);

		// ELEMENT IS A TETRAHEDRON
		if (ChElementTetra_4* mytetra = dynamic_cast<ChElementTetra_4*>(myel))
		{
			// vertexes
			unsigned int ivert_el = i_verts;

			trianglemesh.getCoordsVertices()[i_verts] = ((ChNodeFEMxyz*)mytetra->GetNodeN(0))->GetPos(); 
			++i_verts;
			trianglemesh.getCoordsVertices()[i_verts] = ((ChNodeFEMxyz*)mytetra->GetNodeN(1))->GetPos(); 
			++i_verts;
			trianglemesh.getCoordsVertices()[i_verts] = ((ChNodeFEMxyz*)mytetra->GetNodeN(2))->GetPos(); 
			++i_verts;
			trianglemesh.getCoordsVertices()[i_verts] = ((ChNodeFEMxyz*)mytetra->GetNodeN(3))->GetPos(); 
			++i_verts;

			// colour
			trianglemesh.getCoordsColors()[i_vcols] =  ComputeFalseColor( ComputeScalarOutput ( (ChNodeFEMxyz*)mytetra->GetNodeN(0), mytetra ) );
			++i_vcols;
			trianglemesh.getCoordsColors()[i_vcols] =  ComputeFalseColor( ComputeScalarOutput ( (ChNodeFEMxyz*)mytetra->GetNodeN(1), mytetra ) );
			++i_vcols;
			trianglemesh.getCoordsColors()[i_vcols] =  ComputeFalseColor( ComputeScalarOutput ( (ChNodeFEMxyz*)mytetra->GetNodeN(2), mytetra ) );
			++i_vcols;
			trianglemesh.getCoordsColors()[i_vcols] =  ComputeFalseColor( ComputeScalarOutput ( (ChNodeFEMxyz*)mytetra->GetNodeN(3), mytetra ) );
			++i_vcols;
			// normals: none, defaults to flat triangles. UV: none.

			// faces indexes
			trianglemesh.getIndicesVertexes()[i_index] = ChVector<int> (0,1,2) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_index;
			trianglemesh.getIndicesVertexes()[i_index] = ChVector<int> (1,3,2) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_index;
			trianglemesh.getIndicesVertexes()[i_index] = ChVector<int> (2,3,0) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_index;
			trianglemesh.getIndicesVertexes()[i_index] = ChVector<int> (3,1,0) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_index;

			// colour indexes
			trianglemesh.getIndicesColors()[i_colindex]  = ChVector<int> (0,1,2) + ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_colindex;
			trianglemesh.getIndicesColors()[i_colindex]  = ChVector<int> (1,3,2) + ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_colindex;
			trianglemesh.getIndicesColors()[i_colindex]  = ChVector<int> (2,3,0) + ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_colindex;
			trianglemesh.getIndicesColors()[i_colindex]  = ChVector<int> (3,1,0) + ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_colindex;
		}

		// ELEMENT IS A HEXAEDRON

		//***TO DO*** other types of elements...

	}

	// Finally, update also the children, in case they implemented Update(), 
	// and do this by calling the parent class implementation of ChAssetLevel
	ChAssetLevel::Update();
}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


