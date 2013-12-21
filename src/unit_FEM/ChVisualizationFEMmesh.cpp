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

	wireframe = false;

	smooth_faces = false;

	meshcolor = ChColor(1,1,1,0);

	undeformed_reference = false;

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
		return 1e30; // to force 'white' in false color scale. Hack, to be improved.
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
	case E_PLOT_ELEM_STRAIN_VONMISES:
		if (melement.IsType<ChElementTetra_4>())
		{
			ChSharedPtr<ChElementTetra_4> mytetra ( melement );
			return mytetra->GetStrain().GetEquivalentVonMises();
		}
	case E_PLOT_ELEM_STRESS_VONMISES:
		if (melement.IsType<ChElementTetra_4>())
		{
			ChSharedPtr<ChElementTetra_4> mytetra ( melement );
			return mytetra->GetStress().GetEquivalentVonMises();
		}
	default:
		return 1e30;
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


void TriangleNormalsReset(std::vector< ChVector<> >& normals, std::vector<int>& accumul)
{
	for (unsigned int nn = 0; nn< normals.size(); ++nn)
	{
		normals[nn] = ChVector<>(0,0,0);
		accumul[nn] = 0;
	}
}
void TriangleNormalsCompute(ChVector<int> norm_indexes, ChVector<int> vert_indexes, std::vector< ChVector<> >& vertexes, std::vector< ChVector<> >& normals, std::vector<int>& accumul)
{
	ChVector<> tnorm = Vcross(vertexes[vert_indexes.y]-vertexes[vert_indexes.x], vertexes[vert_indexes.z]-vertexes[vert_indexes.x]).GetNormalized();
	normals[norm_indexes.x] += tnorm;
	normals[norm_indexes.y] += tnorm;
	normals[norm_indexes.z] += tnorm;
	accumul[norm_indexes.x] +=1;
	accumul[norm_indexes.y] +=1;
	accumul[norm_indexes.z] +=1;
}
void TriangleNormalsSmooth(std::vector< ChVector<> >& normals, std::vector<int>& accumul)
{
	for (unsigned int nn = 0; nn< normals.size(); ++nn)
	{
		normals[nn] = normals[nn] * (1.0 / (double)accumul[nn]);
	}
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
	unsigned int n_vnorms = 0;
	unsigned int n_triangles = 0;

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
			n_vnorms +=4; // flat faces
			n_triangles +=4; // n. triangle faces
		}

		// ELEMENT IS A HEXAEDRON
		if (this->FEMmesh->GetElement(iel).IsType<ChElementHexa_8>() )
		{
			n_verts +=8;
			n_vcols +=8;
			n_vnorms +=24;
			n_triangles +=12; // n. triangle faces
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
	if (trianglemesh.getIndicesVertexes().size() != n_triangles)
		trianglemesh.getIndicesVertexes().resize(n_triangles);

	if (this->smooth_faces)
	{
		if (trianglemesh.getCoordsNormals().size() != n_vnorms)
			trianglemesh.getCoordsNormals().resize(n_vnorms);
		if (trianglemesh.getIndicesNormals().size() != n_triangles)
			trianglemesh.getIndicesNormals().resize(n_triangles);
		if (normal_accumulators.size() != n_vnorms)
			normal_accumulators.resize(n_vnorms);
			
		TriangleNormalsReset(trianglemesh.getCoordsNormals(), normal_accumulators); //***TODO***
	}

	//
	// C - update mesh buffers 
	//

	unsigned int i_verts = 0;
	unsigned int i_vcols = 0;
	unsigned int i_vnorms = 0;
	unsigned int i_triindex = 0;
	unsigned int i_normindex = 0;

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

			unsigned int ivert_el = i_verts;
			unsigned int inorm_el = i_vnorms;

			// vertexes
			ChVector<> p0 = node0->GetPos();
			ChVector<> p1 = node1->GetPos();
			ChVector<> p2 = node2->GetPos();
			ChVector<> p3 = node3->GetPos();
			if (undeformed_reference)
			{
				p0 = node0->GetX0();
				p1 = node1->GetX0();
				p2 = node2->GetX0();
				p3 = node3->GetX0();
			}
 
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

			// faces indexes
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (0,1,2) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (1,3,2) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (2,3,0) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (3,1,0) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_triindex;

			// normals indices (if not defaulting to flat triangles)
			if (this->smooth_faces)
			{
				trianglemesh.getIndicesNormals()[i_triindex-4] = ChVector<int> (0,0,0)+ChVector<int> (inorm_el,inorm_el,inorm_el);
				trianglemesh.getIndicesNormals()[i_triindex-3] = ChVector<int> (1,1,1)+ChVector<int> (inorm_el,inorm_el,inorm_el);
				trianglemesh.getIndicesNormals()[i_triindex-2] = ChVector<int> (2,2,2)+ChVector<int> (inorm_el,inorm_el,inorm_el);
				trianglemesh.getIndicesNormals()[i_triindex-1] = ChVector<int> (3,3,3)+ChVector<int> (inorm_el,inorm_el,inorm_el);
				i_vnorms +=4;
			}
		}

		// ------------ELEMENT IS A HEXAHEDRON 8 NODES?
		if (this->FEMmesh->GetElement(iel).IsType<ChElementHexa_8>() )
		{
			// downcasting 
			ChSharedPtr<ChElementHexa_8> mytetra ( this->FEMmesh->GetElement(iel) );

			unsigned int ivert_el = i_verts;
			unsigned int inorm_el = i_vnorms;

			ChSharedPtr<ChNodeFEMxyz> nodes[8];
			ChVector<> pt[8];

			for (int in= 0; in <8; ++in)
			{
				nodes[in]= mytetra->GetNodeN(in);
				if (!undeformed_reference)
					pt[in] = nodes[in]->GetPos();
				else
					pt[in] = nodes[in]->GetX0();
			}

			// vertexes

			if (this->shrink_elements)
			{
				ChVector<> vc(0,0,0);
				for (int in= 0; in <8; ++in)
					vc += pt[in];
				vc = vc*(1.0/8.0); // average, center of element
				for (int in= 0; in <8; ++in)
					pt[in] = vc + this->shrink_factor*(pt[in]-vc);
			}

			for (int in= 0; in <8; ++in)
			{
				trianglemesh.getCoordsVertices()[i_verts] = pt[in]; 
				++i_verts;
			}

			// colours and colours indexes
			for (int in= 0; in <8; ++in)
			{
				trianglemesh.getCoordsColors()[i_vcols] =  ComputeFalseColor( ComputeScalarOutput ( nodes[in], this->FEMmesh->GetElement(iel) ) );
				++i_vcols;
			}

			// faces indexes
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (0,2,1) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (0,3,2) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (4,5,6) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (4,6,7) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (0,7,3) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (0,4,7) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (0,5,4) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (0,1,5) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (3,7,6) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (3,6,2) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (2,5,1) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (2,6,5) +  ChVector<int> (ivert_el,ivert_el,ivert_el);
			++i_triindex;

			// normals indices (if not defaulting to flat triangles)
			if (this->smooth_faces)
			{
				trianglemesh.getIndicesNormals()[i_triindex-12] = ChVector<int> (0,2,1)+ChVector<int> (inorm_el,inorm_el,inorm_el);
				trianglemesh.getIndicesNormals()[i_triindex-11] = ChVector<int> (0,3,2)+ChVector<int> (inorm_el,inorm_el,inorm_el);
				trianglemesh.getIndicesNormals()[i_triindex-10] = ChVector<int> (4,5,6)+ChVector<int> (inorm_el,inorm_el,inorm_el);
				trianglemesh.getIndicesNormals()[i_triindex- 9] = ChVector<int> (4,6,7)+ChVector<int> (inorm_el,inorm_el,inorm_el);
				trianglemesh.getIndicesNormals()[i_triindex- 8] = ChVector<int> (8,  9,10)+ChVector<int> (inorm_el,inorm_el,inorm_el);
				trianglemesh.getIndicesNormals()[i_triindex- 7] = ChVector<int> (8, 11, 9)+ChVector<int> (inorm_el,inorm_el,inorm_el);
				trianglemesh.getIndicesNormals()[i_triindex- 6] = ChVector<int> (12, 13, 14)+ChVector<int> (inorm_el,inorm_el,inorm_el);
				trianglemesh.getIndicesNormals()[i_triindex- 5] = ChVector<int> (12, 15, 13)+ChVector<int> (inorm_el,inorm_el,inorm_el);
				trianglemesh.getIndicesNormals()[i_triindex- 4] = ChVector<int> (16, 18, 17)+ChVector<int> (inorm_el,inorm_el,inorm_el);
				trianglemesh.getIndicesNormals()[i_triindex- 3] = ChVector<int> (16, 17, 19)+ChVector<int> (inorm_el,inorm_el,inorm_el);
				trianglemesh.getIndicesNormals()[i_triindex- 2] = ChVector<int> (20, 21, 23)+ChVector<int> (inorm_el,inorm_el,inorm_el);
				trianglemesh.getIndicesNormals()[i_triindex- 1] = ChVector<int> (20, 22, 21)+ChVector<int> (inorm_el,inorm_el,inorm_el);
				i_vnorms +=24;
			}		
		}


		// ------------***TO DO*** other types of elements...

	}


	if (this->smooth_faces)
	{
		for (unsigned int itri = 0; itri < trianglemesh.getIndicesVertexes().size(); ++itri)
			TriangleNormalsCompute(trianglemesh.getIndicesNormals()[itri], trianglemesh.getIndicesVertexes()[itri], trianglemesh.getCoordsVertices(), trianglemesh.getCoordsNormals(), normal_accumulators);

		TriangleNormalsSmooth( trianglemesh.getCoordsNormals(), normal_accumulators);
	}

	// other flags
	mesh_asset->SetWireframe( this->wireframe );

	// Finally, update also the children, in case they implemented Update(), 
	// and do this by calling the parent class implementation of ChAssetLevel
	ChAssetLevel::Update();
}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


