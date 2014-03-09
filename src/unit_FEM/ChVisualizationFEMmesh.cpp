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
#include "unit_FEM/ChElementBeamEuler.h"
#include "assets/ChTriangleMeshShape.h"
#include "assets/ChGlyphs.h"



namespace chrono
{
namespace fem
{


ChVisualizationFEMmesh::ChVisualizationFEMmesh(ChMesh& mymesh) 
{
	FEMmesh = &mymesh;
	fem_data_type = E_PLOT_NODE_DISP_NORM;
	fem_glyph = E_GLYPH_NONE;

	colorscale_min= 0;
	colorscale_max= 1;

	shrink_elements = false;
	shrink_factor = 0.9;

	symbols_scale = 1.0;
	symbols_thickness = 0.002;

	wireframe = false;

	zbuffer_hide = true;

	smooth_faces = false;

	meshcolor = ChColor(1,1,1,0);
	symbolscolor = ChColor(0,0.5,0.5,0);

	undeformed_reference = false;

	ChSharedPtr<ChTriangleMeshShape> new_mesh_asset(new ChTriangleMeshShape);
	this->AddAsset(new_mesh_asset);

	ChSharedPtr<ChGlyphs> new_glyphs_asset(new ChGlyphs);
	this->AddAsset(new_glyphs_asset);

}

ChColor ChVisualizationFEMmesh::ComputeFalseColor2(double mv)
{
	ChVector<float> mcol = ComputeFalseColor(mv);
	return ChColor(mcol.x, mcol.y, mcol.z);
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

	if (this->fem_data_type == E_PLOT_SURFACE)
		c = ChVector<float>(meshcolor.R, meshcolor.G, meshcolor.B);

	return(c);
}

double ChVisualizationFEMmesh::ComputeScalarOutput( ChSharedPtr<ChNodeFEMxyz> mnode, int nodeID, ChSharedPtr<ChElementBase> melement)
{
	switch (this->fem_data_type)
	{
	case E_PLOT_SURFACE:
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
	case E_PLOT_ELEM_STRAIN_HYDROSTATIC:
		if (melement.IsType<ChElementTetra_4>())
		{
			ChSharedPtr<ChElementTetra_4> mytetra ( melement );
			return mytetra->GetStrain().GetEquivalentMeanHydrostatic();
		}
	case E_PLOT_ELEM_STRESS_HYDROSTATIC:
		if (melement.IsType<ChElementTetra_4>())
		{
			ChSharedPtr<ChElementTetra_4> mytetra ( melement );
			return mytetra->GetStress().GetEquivalentMeanHydrostatic();
		}
	default:
		return 1e30;
	}
	//***TO DO*** other types of scalar outputs
	return 0;
}

double ChVisualizationFEMmesh::ComputeScalarOutput( ChSharedPtr<ChNodeFEMxyzP> mnode, int nodeID, ChSharedPtr<ChElementBase> melement)
{
	switch (this->fem_data_type)
	{
	case E_PLOT_SURFACE:
		return 1e30; // to force 'white' in false color scale. Hack, to be improved.
	case E_PLOT_NODE_P:
		return (mnode->GetP());
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
	ChSharedPtr<ChGlyphs>			 glyphs_asset;

	// try to retrieve previously added mesh asset and glyhs asset in sublevel..
	if (this->GetAssets().size() == 2)
		if (GetAssets()[0].IsType<ChTriangleMeshShape>() &&
			GetAssets()[1].IsType<ChGlyphs>() )
		{
			mesh_asset   = GetAssets()[0];
			glyphs_asset = GetAssets()[1];
		}

	// if not available, create ...
	if (mesh_asset.IsNull())
	{
		this->GetAssets().resize(0); // this to delete other sub assets that are not in mesh & glyphs, if any

		ChSharedPtr<ChTriangleMeshShape> new_mesh_asset(new ChTriangleMeshShape);
		this->AddAsset(new_mesh_asset);
		mesh_asset = new_mesh_asset;

		ChSharedPtr<ChGlyphs> new_glyphs_asset(new ChGlyphs);
		this->AddAsset(new_glyphs_asset);
		glyphs_asset = new_glyphs_asset;
	}
	geometry::ChTriangleMeshConnected& trianglemesh = mesh_asset->GetMesh();


	int beam_resolution = 8;

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

		// ELEMENT IS A TETRAHEDRON for scalar field
		if (this->FEMmesh->GetElement(iel).IsType<ChElementTetra_4_P>() )
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

		// ELEMENT IS A BEAM
		if (this->FEMmesh->GetElement(iel).IsType<ChElementBeamEuler>() )
		{
			n_verts +=4*beam_resolution;
			n_vcols +=4*beam_resolution;
			n_vnorms +=8*beam_resolution;
			n_triangles +=8*(beam_resolution-1); // n. triangle faces
		}

		//***TO DO*** other types of elements...

	}

	if (this->fem_data_type == E_PLOT_NONE)
	{
		n_verts =0;
		n_vcols =0;
		n_vnorms =0;
		n_triangles =0;
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
			
		TriangleNormalsReset(trianglemesh.getCoordsNormals(), normal_accumulators); 
	}

	//
	// C - update mesh buffers 
	//

	unsigned int i_verts = 0;
	unsigned int i_vcols = 0;
	unsigned int i_vnorms = 0;
	unsigned int i_triindex = 0;
	unsigned int i_normindex = 0;

	if (this->fem_data_type != E_PLOT_NONE)
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
			trianglemesh.getCoordsColors()[i_vcols] =  ComputeFalseColor( ComputeScalarOutput ( node0, 0, this->FEMmesh->GetElement(iel) ) );
			++i_vcols;
			trianglemesh.getCoordsColors()[i_vcols] =  ComputeFalseColor( ComputeScalarOutput ( node1, 1, this->FEMmesh->GetElement(iel) ) );
			++i_vcols;
			trianglemesh.getCoordsColors()[i_vcols] =  ComputeFalseColor( ComputeScalarOutput ( node2, 2, this->FEMmesh->GetElement(iel) ) );
			++i_vcols;
			trianglemesh.getCoordsColors()[i_vcols] =  ComputeFalseColor( ComputeScalarOutput ( node3, 3, this->FEMmesh->GetElement(iel) ) );
			++i_vcols;

			// faces indexes
			ChVector<int> ivert_offset(ivert_el,ivert_el,ivert_el);
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (0,1,2) +  ivert_offset;
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (1,3,2) +  ivert_offset;
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (2,3,0) +  ivert_offset;
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (3,1,0) +  ivert_offset;
			++i_triindex;

			// normals indices (if not defaulting to flat triangles)
			if (this->smooth_faces)
			{
				ChVector<int> inorm_offset = ChVector<int> (inorm_el,inorm_el,inorm_el);
				trianglemesh.getIndicesNormals()[i_triindex-4] = ChVector<int> (0,0,0) + inorm_offset;
				trianglemesh.getIndicesNormals()[i_triindex-3] = ChVector<int> (1,1,1) + inorm_offset;
				trianglemesh.getIndicesNormals()[i_triindex-2] = ChVector<int> (2,2,2) + inorm_offset;
				trianglemesh.getIndicesNormals()[i_triindex-1] = ChVector<int> (3,3,3) + inorm_offset;
				i_vnorms +=4;
			}
		}


		// ------------ELEMENT IS A TETRAHEDRON 4 NODES -for SCALAR field- ?
		
		if (this->FEMmesh->GetElement(iel).IsType<ChElementTetra_4_P>() )
		{
			// downcasting 
			ChSharedPtr<ChElementTetra_4_P> mytetra ( this->FEMmesh->GetElement(iel) );
			ChSharedPtr<ChNodeFEMxyzP> node0( mytetra->GetNodeN(0) ); 
			ChSharedPtr<ChNodeFEMxyzP> node1( mytetra->GetNodeN(1) );
			ChSharedPtr<ChNodeFEMxyzP> node2( mytetra->GetNodeN(2) );
			ChSharedPtr<ChNodeFEMxyzP> node3( mytetra->GetNodeN(3) );

			unsigned int ivert_el = i_verts;
			unsigned int inorm_el = i_vnorms;

			// vertexes
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
			trianglemesh.getCoordsColors()[i_vcols] =  ComputeFalseColor( ComputeScalarOutput ( node0, 0, this->FEMmesh->GetElement(iel) ) );
			++i_vcols;
			trianglemesh.getCoordsColors()[i_vcols] =  ComputeFalseColor( ComputeScalarOutput ( node1, 1, this->FEMmesh->GetElement(iel) ) );
			++i_vcols;
			trianglemesh.getCoordsColors()[i_vcols] =  ComputeFalseColor( ComputeScalarOutput ( node2, 2, this->FEMmesh->GetElement(iel) ) );
			++i_vcols;
			trianglemesh.getCoordsColors()[i_vcols] =  ComputeFalseColor( ComputeScalarOutput ( node3, 3, this->FEMmesh->GetElement(iel) ) );
			++i_vcols;

			// faces indexes
			ChVector<int> ivert_offset(ivert_el,ivert_el,ivert_el);
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (0,1,2) +  ivert_offset;
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (1,3,2) +  ivert_offset;
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (2,3,0) +  ivert_offset;
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (3,1,0) +  ivert_offset;
			++i_triindex;

			// normals indices (if not defaulting to flat triangles)
			if (this->smooth_faces)
			{
				ChVector<int> inorm_offset = ChVector<int> (inorm_el,inorm_el,inorm_el);
				trianglemesh.getIndicesNormals()[i_triindex-4] = ChVector<int> (0,0,0) + inorm_offset;
				trianglemesh.getIndicesNormals()[i_triindex-3] = ChVector<int> (1,1,1) + inorm_offset;
				trianglemesh.getIndicesNormals()[i_triindex-2] = ChVector<int> (2,2,2) + inorm_offset;
				trianglemesh.getIndicesNormals()[i_triindex-1] = ChVector<int> (3,3,3) + inorm_offset;
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
				trianglemesh.getCoordsColors()[i_vcols] =  ComputeFalseColor( ComputeScalarOutput ( nodes[in], in, this->FEMmesh->GetElement(iel) ) );
				++i_vcols;
			}

			// faces indexes
			ChVector<int> ivert_offset(ivert_el,ivert_el,ivert_el);
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (0,2,1) +  ivert_offset;
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (0,3,2) +  ivert_offset;
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (4,5,6) +  ivert_offset;
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (4,6,7) +  ivert_offset;
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (0,7,3) +  ivert_offset;
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (0,4,7) +  ivert_offset;
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (0,5,4) +  ivert_offset;
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (0,1,5) +  ivert_offset;
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (3,7,6) +  ivert_offset;
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (3,6,2) +  ivert_offset;
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (2,5,1) +  ivert_offset;
			++i_triindex;
			trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (2,6,5) +  ivert_offset;
			++i_triindex;

			// normals indices (if not defaulting to flat triangles)
			if (this->smooth_faces)
			{
				ChVector<int> inorm_offset = ChVector<int> (inorm_el,inorm_el,inorm_el);
				trianglemesh.getIndicesNormals()[i_triindex-12] = ChVector<int> (0,2,1)+inorm_offset;
				trianglemesh.getIndicesNormals()[i_triindex-11] = ChVector<int> (0,3,2)+inorm_offset;
				trianglemesh.getIndicesNormals()[i_triindex-10] = ChVector<int> (4,5,6)+inorm_offset;
				trianglemesh.getIndicesNormals()[i_triindex- 9] = ChVector<int> (4,6,7)+inorm_offset;
				trianglemesh.getIndicesNormals()[i_triindex- 8] = ChVector<int> (8,  9,10)+inorm_offset;
				trianglemesh.getIndicesNormals()[i_triindex- 7] = ChVector<int> (8, 11, 9)+inorm_offset;
				trianglemesh.getIndicesNormals()[i_triindex- 6] = ChVector<int> (12, 13, 14)+inorm_offset;
				trianglemesh.getIndicesNormals()[i_triindex- 5] = ChVector<int> (12, 15, 13)+inorm_offset;
				trianglemesh.getIndicesNormals()[i_triindex- 4] = ChVector<int> (16, 18, 17)+inorm_offset;
				trianglemesh.getIndicesNormals()[i_triindex- 3] = ChVector<int> (16, 17, 19)+inorm_offset;
				trianglemesh.getIndicesNormals()[i_triindex- 2] = ChVector<int> (20, 21, 23)+inorm_offset;
				trianglemesh.getIndicesNormals()[i_triindex- 1] = ChVector<int> (20, 22, 21)+inorm_offset;
				i_vnorms +=24;
			}		
		}


		// ------------ELEMENT IS A BEAM?
		if (this->FEMmesh->GetElement(iel).IsType<ChElementBeam>() )
		{
			// downcasting 
			ChSharedPtr<ChElementBeam> mybeam ( this->FEMmesh->GetElement(iel) );

			double y_thick = 0.01; // line thickness default value
			double z_thick = 0.01; 

			ChSharedPtr<ChElementBeamEuler> mybeameuler ( mybeam );
			if (!mybeameuler.IsNull())
			{
				// if the beam has a section info, use section specific thickness for drawing
				y_thick = 0.5*mybeameuler->GetSection()->GetDrawThicknessY();
				z_thick = 0.5*mybeameuler->GetSection()->GetDrawThicknessZ();
			}

			unsigned int ivert_el = i_verts;
			unsigned int inorm_el = i_vnorms;

			// displacements & rotations state of the nodes:
			ChMatrixDynamic<> displ(mybeam->GetNdofs(),1);
			mybeam->GetField(displ); // for field of corotated element, u_displ will be always 0 at ends

			for (int in= 0; in < beam_resolution; ++in)
			{
				double eta = -1.0+(2.0*in/(beam_resolution-1));
				
				ChVector<> P;
				ChQuaternion<> msectionrot;
				mybeam->EvaluateSectionFrame(eta, displ, P, msectionrot);  // compute abs. pos and rot of section plane

				ChVector<> vresult;
				ChVector<> vresultB;
				double sresult = 0;
				switch(this->fem_data_type)
				{
					case E_PLOT_ELEM_BEAM_MX:
						mybeam->EvaluateSectionForceTorque(eta, displ, vresult, vresultB);
						sresult = vresultB.x; 
						break;
					case E_PLOT_ELEM_BEAM_MY:
						mybeam->EvaluateSectionForceTorque(eta, displ, vresult, vresultB);
						sresult = vresultB.y; 
						break;
					case E_PLOT_ELEM_BEAM_MZ:
						mybeam->EvaluateSectionForceTorque(eta, displ, vresult, vresultB);
						sresult = vresultB.z; 
						break;
					case E_PLOT_ELEM_BEAM_TX:
						mybeam->EvaluateSectionForceTorque(eta, displ, vresult, vresultB);
						sresult = vresult.x; 
						break;
					case E_PLOT_ELEM_BEAM_TY:
						mybeam->EvaluateSectionForceTorque(eta, displ, vresult, vresultB);
						sresult = vresult.y; 
						break;
					case E_PLOT_ELEM_BEAM_TZ:
						mybeam->EvaluateSectionForceTorque(eta, displ, vresult, vresultB);
						sresult = vresult.z; 
						break;
				}
				ChVector<float> mcol = ComputeFalseColor(sresult);

				trianglemesh.getCoordsVertices()[i_verts] = P + msectionrot.Rotate(ChVector<>(0,-y_thick,-z_thick) ); 
				++i_verts;
				trianglemesh.getCoordsVertices()[i_verts] = P + msectionrot.Rotate(ChVector<>(0, y_thick,-z_thick) ) ; 
				++i_verts;
				trianglemesh.getCoordsVertices()[i_verts] = P + msectionrot.Rotate(ChVector<>(0, y_thick, z_thick) ) ; 
				++i_verts;
				trianglemesh.getCoordsVertices()[i_verts] = P + msectionrot.Rotate(ChVector<>(0,-y_thick, z_thick) ) ; 
				++i_verts;

				trianglemesh.getCoordsColors()[i_vcols] =  mcol; 
				++i_vcols;
				trianglemesh.getCoordsColors()[i_vcols] =  mcol; 
				++i_vcols;
				trianglemesh.getCoordsColors()[i_vcols] =  mcol; 
				++i_vcols;
				trianglemesh.getCoordsColors()[i_vcols] =  mcol; 
				++i_vcols;

				if (in>0)
				{
					ChVector<int> ivert_offset(ivert_el,ivert_el,ivert_el);
					ChVector<int> islice_offset((in-1)*4,(in-1)*4,(in-1)*4);
					trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (4, 0, 1) +  islice_offset + ivert_offset;
					++i_triindex;
					trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (4, 1, 5) +  islice_offset + ivert_offset;
					++i_triindex;
					trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (5, 1, 2) +  islice_offset + ivert_offset;
					++i_triindex;
					trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (5, 2, 6) +  islice_offset + ivert_offset;
					++i_triindex;
					trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (6, 2, 3) +  islice_offset + ivert_offset;
					++i_triindex;
					trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (6, 3, 7) +  islice_offset + ivert_offset;
					++i_triindex;
					trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (7, 3, 0) +  islice_offset + ivert_offset;
					++i_triindex;
					trianglemesh.getIndicesVertexes()[i_triindex] = ChVector<int> (7, 0, 4) +  islice_offset + ivert_offset;
					++i_triindex;
					if (this->smooth_faces)
					{
						ChVector<int> islice_normoffset((in-1)*8,(in-1)*8,(in-1)*8); //***TO DO*** fix errors in normals
						ChVector<int> inorm_offset = ChVector<int> (inorm_el,inorm_el,inorm_el);
						trianglemesh.getIndicesNormals()[i_triindex- 8] = ChVector<int> (8, 0, 1) +  islice_normoffset + inorm_offset;
						trianglemesh.getIndicesNormals()[i_triindex- 7] = ChVector<int> (8, 1, 9) +  islice_normoffset + inorm_offset;
						trianglemesh.getIndicesNormals()[i_triindex- 6] = ChVector<int> (9+4, 1+4, 2+4) +  islice_normoffset + inorm_offset;
						trianglemesh.getIndicesNormals()[i_triindex- 5] = ChVector<int> (9+4, 2+4, 10+4) +  islice_normoffset + inorm_offset;
						trianglemesh.getIndicesNormals()[i_triindex- 4] = ChVector<int> (10, 2, 3) +  islice_normoffset + inorm_offset;
						trianglemesh.getIndicesNormals()[i_triindex- 3] = ChVector<int> (10, 3, 11) +  islice_normoffset + inorm_offset;
						trianglemesh.getIndicesNormals()[i_triindex- 2] = ChVector<int> (11+4, 3+4, 0+4) +  islice_normoffset + inorm_offset;
						trianglemesh.getIndicesNormals()[i_triindex- 1] = ChVector<int> (11+4, 0+4, 8+4) +  islice_normoffset + inorm_offset;
						i_vnorms +=8;
					}				
				}
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


	//
	// GLYPHS
	//

	//***TEST***
	glyphs_asset->Reserve(0); // unoptimal, should reuse buffers as much as possible
	
	glyphs_asset->SetGlyphsSize(this->symbols_thickness);

	glyphs_asset->SetZbufferHide(this->zbuffer_hide);

	if (this->fem_glyph == ChVisualizationFEMmesh::E_GLYPH_NODE_DOT_POS)
	{
		glyphs_asset->SetDrawMode(ChGlyphs::GLYPH_POINT);
		for (unsigned int inode=0; inode < this->FEMmesh->GetNnodes(); ++inode)
			if (this->FEMmesh->GetNode(inode).IsType<ChNodeFEMxyz>())
			{
				ChSharedPtr<ChNodeFEMxyz> mynode ( this->FEMmesh->GetNode(inode) );
				glyphs_asset->SetGlyphPoint(inode,  mynode->GetPos(), this->symbolscolor );
			}
	}
	if (this->fem_glyph == ChVisualizationFEMmesh::E_GLYPH_NODE_CSYS)
	{
		glyphs_asset->SetDrawMode(ChGlyphs::GLYPH_COORDSYS);
		for (unsigned int inode=0; inode < this->FEMmesh->GetNnodes(); ++inode)
			if (this->FEMmesh->GetNode(inode).IsType<ChNodeFEMxyzrot>())
			{
				ChSharedPtr<ChNodeFEMxyzrot> mynode ( this->FEMmesh->GetNode(inode) );
				glyphs_asset->SetGlyphCoordsys(inode,  mynode->Frame().GetCoord());
			}
	}
	if (this->fem_glyph == ChVisualizationFEMmesh::E_GLYPH_NODE_VECT_SPEED)
	{
		glyphs_asset->SetDrawMode(ChGlyphs::GLYPH_VECTOR);
		for (unsigned int inode=0; inode < this->FEMmesh->GetNnodes(); ++inode)
			if (this->FEMmesh->GetNode(inode).IsType<ChNodeFEMxyz>())
			{
				ChSharedPtr<ChNodeFEMxyz> mynode ( this->FEMmesh->GetNode(inode) );
				glyphs_asset->SetGlyphVector(inode, mynode->GetPos(), mynode->GetPos_dt() * this->symbols_scale, this->symbolscolor );
			}
	}
	if (this->fem_glyph == ChVisualizationFEMmesh::E_GLYPH_NODE_VECT_ACCEL)
	{
		glyphs_asset->SetDrawMode(ChGlyphs::GLYPH_VECTOR);
		for (unsigned int inode=0; inode < this->FEMmesh->GetNnodes(); ++inode)
			if (this->FEMmesh->GetNode(inode).IsType<ChNodeFEMxyz>())
			{
				ChSharedPtr<ChNodeFEMxyz> mynode ( this->FEMmesh->GetNode(inode) );
				glyphs_asset->SetGlyphVector(inode, mynode->GetPos(), mynode->GetPos_dtdt() * this->symbols_scale, this->symbolscolor );
			}
	}
	if (this->fem_glyph == ChVisualizationFEMmesh::E_GLYPH_ELEM_VECT_DP)
	{
		glyphs_asset->SetDrawMode(ChGlyphs::GLYPH_VECTOR);
		for (unsigned int iel=0; iel < this->FEMmesh->GetNelements(); ++iel)
			if (this->FEMmesh->GetElement(iel).IsType<ChElementTetra_4_P>())
			{
				ChSharedPtr<ChElementTetra_4_P> myelement ( this->FEMmesh->GetElement(iel) );
				ChMatrixNM<double, 3,1> mP = myelement->GetPgradient();
				ChVector<> mvP(mP(0), mP(1), mP(2) );
				ChSharedPtr<ChNodeFEMxyzP> n0(myelement->GetNodeN(0));
				ChSharedPtr<ChNodeFEMxyzP> n1(myelement->GetNodeN(1));
				ChSharedPtr<ChNodeFEMxyzP> n2(myelement->GetNodeN(2));
				ChSharedPtr<ChNodeFEMxyzP> n3(myelement->GetNodeN(3));
				ChVector<> mPoint = ( n0->GetPos() + 
									  n1->GetPos() + 
									  n2->GetPos() + 
									  n3->GetPos() ) * 0.25; // to do: better placement in Gauss point
				glyphs_asset->SetGlyphVector(iel, mPoint, mvP * this->symbols_scale, this->symbolscolor );
			}
	}
	if (this->fem_glyph == ChVisualizationFEMmesh::E_GLYPH_ELEM_TENS_STRAIN)
	{
		glyphs_asset->SetDrawMode(ChGlyphs::GLYPH_VECTOR);
		int nglyvect = 0;
		for (unsigned int iel=0; iel < this->FEMmesh->GetNelements(); ++iel)
			if (this->FEMmesh->GetElement(iel).IsType<ChElementTetra_4>())
			{
				ChSharedPtr<ChElementTetra_4> myelement ( this->FEMmesh->GetElement(iel) );
				ChStrainTensor<> mstrain = myelement->GetStrain();
				//mstrain.Rotate(myelement->Rotation());
				double e1,e2,e3;
				ChVector<> v1,v2,v3;
				mstrain.ComputePrincipalStrains(e1,e2,e3);
				mstrain.ComputePrincipalStrainsDirections(e1,e2,e3, v1,v2,v3);
				v1.Normalize();
				v2.Normalize();
				v3.Normalize();
				ChSharedPtr<ChNodeFEMxyz> n0(myelement->GetNodeN(0));
				ChSharedPtr<ChNodeFEMxyz> n1(myelement->GetNodeN(1));
				ChSharedPtr<ChNodeFEMxyz> n2(myelement->GetNodeN(2));
				ChSharedPtr<ChNodeFEMxyz> n3(myelement->GetNodeN(3));
				ChVector<> mPoint = ( n0->GetPos() + 
									  n1->GetPos() + 
									  n2->GetPos() + 
									  n3->GetPos() ) * 0.25; // to do: better placement in Gauss point
				glyphs_asset->SetGlyphVector(nglyvect, mPoint, myelement->Rotation()*v1*e1 * this->symbols_scale, ComputeFalseColor2(e1) );
				++nglyvect;
				glyphs_asset->SetGlyphVector(nglyvect, mPoint, myelement->Rotation()*v2*e2 * this->symbols_scale, ComputeFalseColor2(e2) );
				++nglyvect;
				glyphs_asset->SetGlyphVector(nglyvect, mPoint, myelement->Rotation()*v3*e3 * this->symbols_scale, ComputeFalseColor2(e3) );
				++nglyvect;
			}
	}
	if (this->fem_glyph == ChVisualizationFEMmesh::E_GLYPH_ELEM_TENS_STRESS)
	{
		glyphs_asset->SetDrawMode(ChGlyphs::GLYPH_VECTOR);
		int nglyvect = 0;
		for (unsigned int iel=0; iel < this->FEMmesh->GetNelements(); ++iel)
			if (this->FEMmesh->GetElement(iel).IsType<ChElementTetra_4>())
			{
				ChSharedPtr<ChElementTetra_4> myelement ( this->FEMmesh->GetElement(iel) );
				ChStressTensor<> mstress = myelement->GetStress();
				mstress.Rotate(myelement->Rotation());
				double e1,e2,e3;
				ChVector<> v1,v2,v3;
				mstress.ComputePrincipalStresses(e1,e2,e3);
				mstress.ComputePrincipalStressesDirections(e1,e2,e3, v1,v2,v3);
				v1.Normalize();
				v2.Normalize();
				v3.Normalize();
				ChSharedPtr<ChNodeFEMxyz> n0(myelement->GetNodeN(0));
				ChSharedPtr<ChNodeFEMxyz> n1(myelement->GetNodeN(1));
				ChSharedPtr<ChNodeFEMxyz> n2(myelement->GetNodeN(2));
				ChSharedPtr<ChNodeFEMxyz> n3(myelement->GetNodeN(3));
				ChVector<> mPoint = ( n0->GetPos() + 
									  n1->GetPos() + 
									  n2->GetPos() + 
									  n3->GetPos() ) * 0.25; // to do: better placement in Gauss point
				glyphs_asset->SetGlyphVector(nglyvect, mPoint, myelement->Rotation()*v1*e1 * this->symbols_scale, ComputeFalseColor2(e1) );
				++nglyvect;
				glyphs_asset->SetGlyphVector(nglyvect, mPoint, myelement->Rotation()*v2*e2 * this->symbols_scale, ComputeFalseColor2(e2) );
				++nglyvect;
				glyphs_asset->SetGlyphVector(nglyvect, mPoint, myelement->Rotation()*v3*e3 * this->symbols_scale, ComputeFalseColor2(e3) );
				++nglyvect;
			}
	}
	



	// Finally, update also the children, in case they implemented Update(), 
	// and do this by calling the parent class implementation of ChAssetLevel
	ChAssetLevel::Update();
}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


