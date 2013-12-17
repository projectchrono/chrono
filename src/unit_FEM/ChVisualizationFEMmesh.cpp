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
#include "unit_FEM/ChElementTetra_10.h"
#include "unit_FEM/ChElementHexa_8.h"
#include "unit_FEM/ChElementHexa_20.h"
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

	shrink_elements = false;
	shrink_factor = 0.9;

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

double ChVisualizationFEMmesh::ComputeScalarOutput( ChSharedPtr<ChNodeFEMxyz> mnode, ChSharedPtr<ChElementBase> melement)
{
	switch (this->fem_data_type)
	{
	case E_PLOT_NONE:
		return 0;
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
		// ELEMENT IS A TETRAHEDRON
		if (this->FEMmesh->GetElement(iel).IsType<ChElementTetra_4>() )
		{
			n_verts +=4;
			n_vcols +=4;
			n_index +=4; // n. triangle faces
		}

		// ELEMENT IS A HEXAEDRON
		if (this->FEMmesh->GetElement(iel).IsType<ChElementHexa_8>() )
		{
			n_verts +=8;
			n_vcols +=8;
			n_index +=12; // n. triangle faces
		}

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
		// ------------ELEMENT IS A TETRAHEDRON 4 NODES?
		
		if (this->FEMmesh->GetElement(iel).IsType<ChElementTetra_4>() )
		{
			// downcasting 
			ChSharedPtr<ChElementTetra_4> mytetra ( this->FEMmesh->GetElement(iel) );
			ChSharedPtr<ChNodeFEMxyz> node0( mytetra->GetNodeN(0) ); 
			ChSharedPtr<ChNodeFEMxyz> node1( mytetra->GetNodeN(1) );
			ChSharedPtr<ChNodeFEMxyz> node2( mytetra->GetNodeN(2) );
			ChSharedPtr<ChNodeFEMxyz> node3( mytetra->GetNodeN(3) );

			// vertexes
			unsigned int ivert_el = i_verts;
			
			ChVector<> p0 = node0->GetPos();
			ChVector<> p1 = node1->GetPos();
			ChVector<> p2 = node2->GetPos();
			ChVector<> p3 = node3->GetPos();
 
			if (this->shrink_elements)
			{
				ChVector<> vc = (p0+p1+p2+p3)*(0.25);
				p0 = vc + this->shrink_factor*(p0-vc);
				p1 = vc + this->shrink_factor*(p1-vc);
				p2 = vc + this->shrink_factor*(p2-vc);
				p3 = vc + this->shrink_factor*(p3-vc);
			}
			trianglemesh.getCoordsVertices()[i_verts] = p0; 
			++i_verts;
			trianglemesh.getCoordsVertices()[i_verts] = p1; 
			++i_verts;
			trianglemesh.getCoordsVertices()[i_verts] = p2; 
			++i_verts;
			trianglemesh.getCoordsVertices()[i_verts] = p3; 
			++i_verts;

			// colour
			trianglemesh.getCoordsColors()[i_vcols] =  ComputeFalseColor( ComputeScalarOutput ( node0, this->FEMmesh->GetElement(iel) ) );
			++i_vcols;
			trianglemesh.getCoordsColors()[i_vcols] =  ComputeFalseColor( ComputeScalarOutput ( node1, this->FEMmesh->GetElement(iel) ) );
			++i_vcols;
			trianglemesh.getCoordsColors()[i_vcols] =  ComputeFalseColor( ComputeScalarOutput ( node2, this->FEMmesh->GetElement(iel) ) );
			++i_vcols;
			trianglemesh.getCoordsColors()[i_vcols] =  ComputeFalseColor( ComputeScalarOutput ( node3, this->FEMmesh->GetElement(iel) ) );
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

		// ------------ELEMENT IS A HEXAHEDRON 8 NODES?
		if (this->FEMmesh->GetElement(iel).IsType<ChElementHexa_8>() )
		{
			// downcasting 
			ChSharedPtr<ChElementHexa_8> mytetra ( this->FEMmesh->GetElement(iel) );
			ChSharedPtr<ChNodeFEMxyz> node0( mytetra->GetNodeN(0) ); 
			ChSharedPtr<ChNodeFEMxyz> node1( mytetra->GetNodeN(1) );
			ChSharedPtr<ChNodeFEMxyz> node2( mytetra->GetNodeN(2) );
			ChSharedPtr<ChNodeFEMxyz> node3( mytetra->GetNodeN(3) );
			ChSharedPtr<ChNodeFEMxyz> node4( mytetra->GetNodeN(4) );
			ChSharedPtr<ChNodeFEMxyz> node5( mytetra->GetNodeN(5) );
			ChSharedPtr<ChNodeFEMxyz> node6( mytetra->GetNodeN(6) );
			ChSharedPtr<ChNodeFEMxyz> node7( mytetra->GetNodeN(7) );

			// vertexes
			unsigned int ivert_el = i_verts;
			
			ChVector<> p0 = node0->GetPos();
			ChVector<> p1 = node1->GetPos();
			ChVector<> p2 = node2->GetPos();
			ChVector<> p3 = node3->GetPos();
			ChVector<> p4 = node4->GetPos();
			ChVector<> p5 = node5->GetPos();
			ChVector<> p6 = node6->GetPos();
			ChVector<> p7 = node7->GetPos();
 
			if (this->shrink_elements)
			{
				ChVector<> vc = (p0+p1+p2+p3+p4+p5+p6+p7)*(1.0/8.0);
				p0 = vc + this->shrink_factor*(p0-vc);
				p1 = vc + this->shrink_factor*(p1-vc);
				p2 = vc + this->shrink_factor*(p2-vc);
				p3 = vc + this->shrink_factor*(p3-vc);
				p4 = vc + this->shrink_factor*(p4-vc);
				p5 = vc + this->shrink_factor*(p5-vc);
				p6 = vc + this->shrink_factor*(p6-vc);
				p7 = vc + this->shrink_factor*(p7-vc);
			}
			trianglemesh.getCoordsVertices()[i_verts] = p0; 
			++i_verts;
			trianglemesh.getCoordsVertices()[i_verts] = p1; 
			++i_verts;
			trianglemesh.getCoordsVertices()[i_verts] = p2; 
			++i_verts;
			trianglemesh.getCoordsVertices()[i_verts] = p3; 
			++i_verts;
			trianglemesh.getCoordsVertices()[i_verts] = p4; 
			++i_verts;
			trianglemesh.getCoordsVertices()[i_verts] = p5; 
			++i_verts;
			trianglemesh.getCoordsVertices()[i_verts] = p6; 
			++i_verts;
			trianglemesh.getCoordsVertices()[i_verts] = p7; 
			++i_verts;

			// colour
			trianglemesh.getCoordsColors()[i_vcols] =  ComputeFalseColor( ComputeScalarOutput ( node0, this->FEMmesh->GetElement(iel) ) );
			++i_vcols;
			trianglemesh.getCoordsColors()[i_vcols] =  ComputeFalseColor( ComputeScalarOutput ( node1, this->FEMmesh->GetElement(iel) ) );
			++i_vcols;
			trianglemesh.getCoordsColors()[i_vcols] =  ComputeFalseColor( ComputeScalarOutput ( node2, this->FEMmesh->GetElement(iel) ) );
			++i_vcols;
			trianglemesh.getCoordsColors()[i_vcols] =  ComputeFalseColor( ComputeScalarOutput ( node3, this->FEMmesh->GetElement(iel) ) );
			++i_vcols;
			trianglemesh.getCoordsColors()[i_vcols] =  ComputeFalseColor( ComputeScalarOutput ( node4, this->FEMmesh->GetElement(iel) ) );
			++i_vcols;
			trianglemesh.getCoordsColors()[i_vcols] =  ComputeFalseColor( ComputeScalarOutput ( node5, this->FEMmesh->GetElement(iel) ) );
			++i_vcols;
			trianglemesh.getCoordsColors()[i_vcols] =  ComputeFalseColor( ComputeScalarOutput ( node6, this->FEMmesh->GetElement(iel) ) );
			++i_vcols;
			trianglemesh.getCoordsColors()[i_vcols] =  ComputeFalseColor( ComputeScalarOutput ( node7, this->FEMmesh->GetElement(iel) ) );
			++i_vcols;
			// normals: none, defaults to flat triangles. UV: none.

			// faces indexes
			trianglemesh.getIndicesVertexes()[i_index] = ChVector<int> (0,2,1) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_index;
			trianglemesh.getIndicesVertexes()[i_index] = ChVector<int> (0,3,2) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_index;
			trianglemesh.getIndicesVertexes()[i_index] = ChVector<int> (4,5,6) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_index;
			trianglemesh.getIndicesVertexes()[i_index] = ChVector<int> (4,6,7) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_index;
			trianglemesh.getIndicesVertexes()[i_index] = ChVector<int> (0,7,3) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_index;
			trianglemesh.getIndicesVertexes()[i_index] = ChVector<int> (0,4,7) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_index;
			trianglemesh.getIndicesVertexes()[i_index] = ChVector<int> (0,5,4) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_index;
			trianglemesh.getIndicesVertexes()[i_index] = ChVector<int> (0,1,5) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_index;
			trianglemesh.getIndicesVertexes()[i_index] = ChVector<int> (3,7,6) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_index;
			trianglemesh.getIndicesVertexes()[i_index] = ChVector<int> (3,6,2) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_index;
			trianglemesh.getIndicesVertexes()[i_index] = ChVector<int> (2,5,1) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_index;
			trianglemesh.getIndicesVertexes()[i_index] = ChVector<int> (2,6,5) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_index;
			

			// colour indexes
			for (int ic= 0; ic <7; ++ic)
			{
				trianglemesh.getIndicesColors()[i_colindex]  = trianglemesh.getIndicesVertexes()[i_colindex];
				++i_colindex;
			}
		}




		// ------------***TO DO*** other types of elements...

	}

	// Finally, update also the children, in case they implemented Update(), 
	// and do this by calling the parent class implementation of ChAssetLevel
	ChAssetLevel::Update();
}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


