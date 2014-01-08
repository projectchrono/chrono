//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File author: A.Tasora

#ifndef CHIRRNODEPROXYTOASSET_H
#define CHIRRNODEPROXYTOASSET_H



#include <irrlicht.h>
//#include <ITimer.h>
#include "physics/ChSystem.h"
#include "assets/ChVisualization.h"
#include "assets/ChTriangleMeshShape.h"
#include "assets/ChGlyphs.h"


#define ESNT_CHIRRNODEPROXYTOASSET 1202


namespace irr
{
namespace scene
{





/// Class for proxy to ChAsset, it is a node with mesh in Irrlicht system
/// and a shared pointer to the ChAsset to whom it corresponds.
/// Example: (with ascii art, with --> shared pointer, ...> raw pointer)
///
///   CHRONO side                  IRRLICHT side
///     ChBody  <......................._ 
///        ChIrrNodeAsset  -------->  ChIrrNode
///        ChBoxShape  <--------------   ChIrrNodeProxyToAsset
///                                           IMeshSceneNode
///        ChSphereShape  <------------  ChIrrNodeProxyToAsset
///                                           IMeshSceneNode

class ChIrrNodeProxyToAsset : public scene::ISceneNode
{
	
private:
	core::aabbox3d<f32> Box;

	chrono::ChSharedPtr<chrono::ChVisualization> visualization_asset;

	bool do_update;

public:

		
	ChIrrNodeProxyToAsset( 
					chrono::ChSharedPtr<chrono::ChVisualization> myvisualization,   ///< pointer to the Chrono::Engine visualization asset
					ISceneNode* parent  ///< the parent node in Irrlicht hierarchy
					)
		:	scene::ISceneNode(parent, parent->getSceneManager(), 0) , 						 
			do_update(true)
	{
		#ifdef _DEBUG
			setDebugName("ChIrrNodeProxyToAsset");
		#endif

		// Set the shared pointer to the asset
		visualization_asset = myvisualization;
		
	}

		/// Destructor.
	~ChIrrNodeProxyToAsset()
	{
	}


	virtual void render()
	{
		// must define member because it is abstract in base class
	}

	virtual const core::aabbox3d<f32>& getBoundingBox() const
	{
		return Box;
		// must define member because it is abstract in base class
	}

	ISceneNode* clone(ISceneNode* newParent, ISceneManager* newManager)
	{
		if (!newParent)
			newParent = Parent;
		if (!newManager)
			newManager = SceneManager;

		ChIrrNodeProxyToAsset* nb = new ChIrrNodeProxyToAsset(this->visualization_asset,newParent);

		nb->cloneMembers(this, newManager);
		nb->Box = this->Box;
		nb->visualization_asset = this->visualization_asset;

		if ( newParent )
			nb->drop();
		return nb;
	}

		//
		// CHRONO::ENGINE SPECIFIC
		//

		/// Returns reference to the shared pointer which references the
		/// ChVisualization asset to whom this is a proxy
	chrono::ChSharedPtr<chrono::ChVisualization>& GetVisualizationAsset() {return this->visualization_asset;}

		/// Returns true if the node must recompute the mesh for each time
		/// that an Update is called.
	virtual bool IsUpdateEnabled() const { return this->do_update; }
		/// Set if the node must recompute the mesh for each time
		/// that an Update is called.
	virtual void SetUpdateEnabled(const bool mup) { this->do_update = mup; }
	
		/// Updates the child mesh to reflect the ChVisualization asset
	virtual void Update()
		{
			if (!this->do_update) 
				return;

			if (this->visualization_asset.IsNull()) 
				return;

			if (this->visualization_asset.IsType<ChTriangleMeshShape>())
			{
				ChSharedPtr<ChTriangleMeshShape> trianglemesh = this->visualization_asset;

				// Fetch the 1st child, i.e. the mesh
				irr::scene::ISceneNode* mchildnode = *(this->getChildren().begin()); 
				if (!mchildnode) 
					return;

				if (!(mchildnode->getType() == ESNT_MESH))
					return;
				IMeshSceneNode* meshnode = (IMeshSceneNode*)mchildnode; // dynamic_cast not enabled in Irrlicht dll

				IMesh* amesh = meshnode->getMesh();
				if (amesh->getMeshBufferCount() == 0)
					return;

				chrono::geometry::ChTriangleMeshConnected* mmesh = &trianglemesh->GetMesh();
				unsigned int ntriangles = mmesh->getIndicesVertexes().size();
				unsigned int nvertexes = ntriangles * 3; // this is suboptimal because some vertexes might be shared, but easier now..

				SMeshBuffer* irrmesh = (SMeshBuffer*)amesh->getMeshBuffer(0);

				// smart inflating of allocated buffers, only if necessary, and once in a while shrinking
				if (irrmesh->Indices.allocated_size() > (ntriangles*3) * 1.5)
					irrmesh->Indices.clear();
				if (irrmesh->Vertices.allocated_size() > nvertexes * 1.5)
					irrmesh->Vertices.clear();

				irrmesh->Indices.set_used(ntriangles*3);
				irrmesh->Vertices.set_used(nvertexes); 

				// set buffers
				for (unsigned int itri = 0; itri < ntriangles; itri++)
				{	
					ChVector<> t1 = mmesh->getCoordsVertices()[ mmesh->getIndicesVertexes()[itri].x ];
					ChVector<> t2 = mmesh->getCoordsVertices()[ mmesh->getIndicesVertexes()[itri].y ];
					ChVector<> t3 = mmesh->getCoordsVertices()[ mmesh->getIndicesVertexes()[itri].z ];					
					ChVector<> n1, n2, n3;
					if ( mmesh->getIndicesNormals().size() == mmesh->getIndicesVertexes().size() )
					{
						n1 = mmesh->getCoordsNormals()[ mmesh->getIndicesNormals()[itri].x ];
						n2 = mmesh->getCoordsNormals()[ mmesh->getIndicesNormals()[itri].y ];
						n3 = mmesh->getCoordsNormals()[ mmesh->getIndicesNormals()[itri].z ];
					}
					else
					{
						n1 = Vcross(t2-t1, t3-t1).GetNormalized();
						n2 = n1; 
						n3 = n1;
					}

					ChVector<> uv1,uv2,uv3;
					if ( mmesh->getIndicesUV().size() == mmesh->getIndicesVertexes().size() )
					{
						uv1 = mmesh->getCoordsUV()[ mmesh->getIndicesUV()[itri].x ];
						uv2 = mmesh->getCoordsUV()[ mmesh->getIndicesUV()[itri].y ];
						uv3 = mmesh->getCoordsUV()[ mmesh->getIndicesUV()[itri].z ];
					}
					else if ( mmesh->getIndicesUV().size() == 0 && 
						     (mmesh->getCoordsUV().size()  == mmesh->getCoordsVertices().size() ) )
					{
						uv1 = mmesh->getCoordsUV()[ mmesh->getIndicesVertexes()[itri].x ];
						uv2 = mmesh->getCoordsUV()[ mmesh->getIndicesVertexes()[itri].y ];
						uv3 = mmesh->getCoordsUV()[ mmesh->getIndicesVertexes()[itri].z ];
					}
					else
					{
						uv1=uv2=uv3= VNULL;
					}

					ChVector<float> col1,col2,col3;
					if ( mmesh->getIndicesColors().size() == mmesh->getIndicesVertexes().size() )
					{
						col1 = mmesh->getCoordsColors()[ mmesh->getIndicesColors()[itri].x ];
						col2 = mmesh->getCoordsColors()[ mmesh->getIndicesColors()[itri].y ];
						col3 = mmesh->getCoordsColors()[ mmesh->getIndicesColors()[itri].z ];
					}
					else if ( mmesh->getIndicesColors().size() == 0 && 
						     (mmesh->getCoordsColors().size() == mmesh->getCoordsVertices().size() ) )
					{
						col1 = mmesh->getCoordsColors()[ mmesh->getIndicesVertexes()[itri].x ];
						col2 = mmesh->getCoordsColors()[ mmesh->getIndicesVertexes()[itri].y ];
						col3 = mmesh->getCoordsColors()[ mmesh->getIndicesVertexes()[itri].z ];
					} else
					{
						col1=col2=col3= ChVector<float>( trianglemesh->GetColor().R,
														 trianglemesh->GetColor().G, 
														 trianglemesh->GetColor().B);
					}

					irrmesh->Vertices[0+itri*3] = irr::video::S3DVertex((irr::f32)t1.x, (irr::f32)t1.y, (irr::f32)t1.z, 
																		(irr::f32)n1.x, (irr::f32)n1.y, (irr::f32)n1.z,
																		irr::video::SColor(255,
																		 (irr::u32)(col1.x*255),
																		 (irr::u32)(col1.y*255),
																		 (irr::u32)(col1.z*255)),
																		(irr::f32)uv1.x, (irr::f32)uv1.y);
					irrmesh->Vertices[1+itri*3] = irr::video::S3DVertex((irr::f32)t2.x, (irr::f32)t2.y, (irr::f32)t2.z, 
																		(irr::f32)n2.x, (irr::f32)n2.y, (irr::f32)n2.z,
																		irr::video::SColor(255,
																		 (irr::u32)(col2.x*255),
																		 (irr::u32)(col2.y*255),
																		 (irr::u32)(col2.z*255)),
																		(irr::f32)uv2.x, (irr::f32)uv2.y);
					irrmesh->Vertices[2+itri*3] = irr::video::S3DVertex((irr::f32)t3.x, (irr::f32)t3.y, (irr::f32)t3.z, 
																		(irr::f32)n3.x, (irr::f32)n3.y, (irr::f32)n3.z,
																		irr::video::SColor(255,
																		 (irr::u32)(col3.x*255),
																		 (irr::u32)(col3.y*255),
																		 (irr::u32)(col3.z*255)),
																		(irr::f32)uv3.x, (irr::f32)uv3.y);

					irrmesh->Indices[0+itri*3] = 0+itri*3;
					irrmesh->Indices[1+itri*3] = 1+itri*3;
					irrmesh->Indices[2+itri*3] = 2+itri*3;
				}
				irrmesh->setDirty(); // to force update of hardware buffers
				irrmesh->setHardwareMappingHint(irr::scene::EHM_DYNAMIC);//EHM_NEVER); //EHM_DYNAMIC for faster hw mapping
				irrmesh->recalculateBoundingBox();

				meshnode->setMaterialFlag(video::EMF_WIREFRAME,			trianglemesh->IsWireframe() ); 
				meshnode->setMaterialFlag(video::EMF_LIGHTING,			!trianglemesh->IsWireframe() ); // avoid shading for wireframes
				meshnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, trianglemesh->IsBackfaceCull() );

				meshnode->setMaterialFlag(video::EMF_COLOR_MATERIAL, true); // so color shading = vertexes  color

			}


			if (this->visualization_asset.IsType<ChGlyphs>())
			{
				ChSharedPtr<ChGlyphs> mglyphs = this->visualization_asset;

				// Fetch the 1st child, i.e. the mesh
				irr::scene::ISceneNode* mchildnode = *(this->getChildren().begin()); 
				if (!mchildnode) 
					return;

				if (!(mchildnode->getType() == ESNT_MESH))
					return;
				IMeshSceneNode* meshnode = (IMeshSceneNode*)mchildnode; // dynamic_cast not enabled in Irrlicht dll

				IMesh* amesh = meshnode->getMesh();
				if (amesh->getMeshBufferCount() == 0)
					return;

				SMeshBuffer* irrmesh = (SMeshBuffer*)amesh->getMeshBuffer(0);

				unsigned int ntriangles = 0;
				unsigned int nvertexes = 0;

				if (mglyphs->GetDrawMode() == ChGlyphs::GLYPH_POINT)
				{
					ntriangles =  12 * mglyphs->GetNumberOfGlyphs();
					nvertexes =   24 * mglyphs->GetNumberOfGlyphs();
				}
				if (mglyphs->GetDrawMode() == ChGlyphs::GLYPH_VECTOR)
				{
					ntriangles =  1 * mglyphs->GetNumberOfGlyphs();
					nvertexes =   3 * mglyphs->GetNumberOfGlyphs();
				}

				// smart inflating of allocated buffers, only if necessary, and once in a while shrinking
				if (irrmesh->Indices.allocated_size() > (ntriangles*3) * 1.5)
					irrmesh->Indices.clear();
				if (irrmesh->Vertices.allocated_size() > nvertexes * 1.5)
					irrmesh->Vertices.clear();

				irrmesh->Indices.set_used(ntriangles*3);
				irrmesh->Vertices.set_used(nvertexes); 

				// set buffers

				if (mglyphs->GetDrawMode() == ChGlyphs::GLYPH_POINT)
				{
					const u16 u[36] = {   0,1,2,   0,2,3, 
										  4,6,5,   4,7,6,
										  8,9,10,  8,10,11, 
										  12,14,13, 12,15,14,
										  16,18,17, 16,19,18,
										  20,21,22, 20,22,23 };

					int itri = 0;

					for (unsigned int ig= 0; ig< mglyphs->points.size(); ++ig)
					{
						ChVector<> t1 =  mglyphs->points[ig];
						ChColor mcol  =  mglyphs->colors[ig];
						video::SColor clr(255, (irr::u32)(mcol.R *255), (irr::u32)(mcol.G *255), (irr::u32)(mcol.B *255) );

						// create a small cube per each vertex
						unsigned int voffs = ig*24;
						irr::f32 s = (irr::f32) (mglyphs->GetGlyphsSize() *0.5);

						irrmesh->Vertices[0+voffs]  = irr::video::S3DVertex(-s,-s,-s,  0, 0,-1, clr, 0, 0);
						irrmesh->Vertices[1+voffs]  = irr::video::S3DVertex(-s, s,-s,  0, 0,-1, clr, 0, 1);
						irrmesh->Vertices[2+voffs]  = irr::video::S3DVertex( s, s,-s,  0, 0,-1, clr, 1, 1);
						irrmesh->Vertices[3+voffs]  = irr::video::S3DVertex( s,-s,-s,  0, 0,-1, clr, 1, 0);

						irrmesh->Vertices[4+voffs]  = irr::video::S3DVertex(-s,-s, s,  0, 0, 1, clr, 0, 0);
						irrmesh->Vertices[5+voffs]  = irr::video::S3DVertex(-s, s, s,  0, 0, 1, clr, 0, 1);
						irrmesh->Vertices[6+voffs]  = irr::video::S3DVertex( s, s, s,  0, 0, 1, clr, 1, 1);
						irrmesh->Vertices[7+voffs]  = irr::video::S3DVertex( s,-s, s,  0, 0, 1, clr, 1, 0);

						irrmesh->Vertices[8+voffs]  = irr::video::S3DVertex(-s,-s,-s, -1, 0, 0, clr, 0, 0);
						irrmesh->Vertices[9+voffs]  = irr::video::S3DVertex(-s,-s, s, -1, 0, 0, clr, 0, 1);
						irrmesh->Vertices[10+voffs] = irr::video::S3DVertex(-s, s, s, -1, 0, 0, clr, 1, 1);
						irrmesh->Vertices[11+voffs] = irr::video::S3DVertex(-s, s,-s, -1, 0, 0, clr, 1, 0);

						irrmesh->Vertices[12+voffs] = irr::video::S3DVertex( s,-s,-s,  1, 0, 0, clr, 0, 0);
						irrmesh->Vertices[13+voffs] = irr::video::S3DVertex( s,-s, s,  1, 0, 0, clr, 0, 1);
						irrmesh->Vertices[14+voffs] = irr::video::S3DVertex( s, s, s,  1, 0, 0, clr, 1, 1);
						irrmesh->Vertices[15+voffs] = irr::video::S3DVertex( s, s,-s,  1, 0, 0, clr, 1, 0);

						irrmesh->Vertices[16+voffs] = irr::video::S3DVertex(-s,-s,-s,  0,-1, 0, clr, 0, 0);
						irrmesh->Vertices[17+voffs] = irr::video::S3DVertex(-s,-s, s,  0,-1, 0, clr, 0, 1);
						irrmesh->Vertices[18+voffs] = irr::video::S3DVertex( s,-s, s,  0,-1, 0, clr, 1, 1);
						irrmesh->Vertices[19+voffs] = irr::video::S3DVertex( s,-s,-s,  0,-1, 0, clr, 1, 0);

						irrmesh->Vertices[20+voffs] = irr::video::S3DVertex(-s, s,-s,  0, 1, 0, clr, 0, 0);
						irrmesh->Vertices[21+voffs] = irr::video::S3DVertex(-s, s, s,  0, 1, 0, clr, 0, 1);
						irrmesh->Vertices[22+voffs] = irr::video::S3DVertex( s, s, s,  0, 1, 0, clr, 1, 1);
						irrmesh->Vertices[23+voffs] = irr::video::S3DVertex( s, s,-s,  0, 1, 0, clr, 1, 0);

						for (u32 i=0; i<24; ++i)
						{
							irrmesh->Vertices[i+voffs].Pos += core::vector3df((irr::f32)t1.x, (irr::f32)t1.y, (irr::f32)t1.z);
							//buffer->BoundingBox.addInternalPoint(buffer->Vertices[i].Pos);
						}

						for (u32 i=0; i<36; ++i)
							irrmesh->Indices[i+itri] = u[i]+voffs;
						itri += 36;

					}
				}


				if (mglyphs->GetDrawMode() == ChGlyphs::GLYPH_VECTOR)
				{
					int itri = 0;
					for (unsigned int ig= 0; ig< mglyphs->points.size(); ++ig)
					{
						ChVector<> t1 =  mglyphs->points [ig];
						ChVector<> t2 =  mglyphs->vectors[ig] + t1;
						ChColor mcol  =  mglyphs->colors [ig];
						video::SColor clr(255, (irr::u32)(mcol.R *255), (irr::u32)(mcol.G *255), (irr::u32)(mcol.B *255) );

						// create a  small line (a degenerate triangle) per each vector
						irrmesh->Vertices[0+ig*3] = irr::video::S3DVertex((irr::f32)t1.x, (irr::f32)t1.y, (irr::f32)t1.z, 
																		1, 0, 0,
																		clr,
																		0, 0);
						irrmesh->Vertices[1+ig*3] = irr::video::S3DVertex((irr::f32)t2.x, (irr::f32)t2.y, (irr::f32)t2.z, 
																		1, 0, 0,
																		clr,
																		0, 0);
						irrmesh->Vertices[2+ig*3] = irr::video::S3DVertex((irr::f32)t2.x, (irr::f32)t2.y, (irr::f32)t2.z, 
																		1, 0, 0,
																		clr,
																		0, 0);
						irrmesh->Indices[0+itri*3] = 0+ig*3;
						irrmesh->Indices[1+itri*3] = 1+ig*3;
						irrmesh->Indices[2+itri*3] = 2+ig*3;
						++itri;
					}
				}

				

				irrmesh->setDirty(); // to force update of hardware buffers
				irrmesh->setHardwareMappingHint(irr::scene::EHM_DYNAMIC);//EHM_NEVER); //EHM_DYNAMIC for faster hw mapping
				irrmesh->recalculateBoundingBox();

				if (mglyphs->GetDrawMode() == ChGlyphs::GLYPH_VECTOR)
				{
					meshnode->setMaterialFlag(video::EMF_WIREFRAME,			true ); 
					meshnode->setMaterialFlag(video::EMF_LIGHTING,			false ); // avoid shading for wireframe
					meshnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, false );
				}
				else
				{
					meshnode->setMaterialFlag(video::EMF_WIREFRAME,			false ); 
					meshnode->setMaterialFlag(video::EMF_LIGHTING,			true ); 
					meshnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, true );
				}

				if (mglyphs->GetZbufferHide() == true)
					meshnode->setMaterialFlag(video::EMF_ZBUFFER,			true );
				else
					meshnode->setMaterialFlag(video::EMF_ZBUFFER,			false );


				meshnode->setMaterialFlag(video::EMF_COLOR_MATERIAL, true); // so color shading = vertexes  color

			}





		}


		virtual ESCENE_NODE_TYPE getType() const {  return (ESCENE_NODE_TYPE) ESNT_CHIRRNODEPROXYTOASSET; }	
};






} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

#endif

