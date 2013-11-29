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
				IMeshSceneNode* meshnode = (IMeshSceneNode*)mchildnode; // dynamic_cast not enablesd in Irrlicht dll

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
					if ( mmesh->getIndicesNormals().size() == mmesh->getCoordsVertices().size() )
					{
						n1 = mmesh->getCoordsNormals()[ mmesh->getIndicesVertexes()[itri].x ];
						n2 = mmesh->getCoordsNormals()[ mmesh->getIndicesVertexes()[itri].y ];
						n3 = mmesh->getCoordsNormals()[ mmesh->getIndicesVertexes()[itri].z ];
					}
					else
					{
						n1 = Vcross(t2-t1, t3-t1).GetNormalized();
						n2 = n1; 
						n3 = n1;
					}

					ChVector<> uv1,uv2,uv3;
					if ( mmesh->getIndicesUV().size() == mmesh->getCoordsVertices().size() )
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
					if ( mmesh->getIndicesColors().size() == mmesh->getCoordsVertices().size() )
					{
						col1 = mmesh->getCoordsColors()[ mmesh->getIndicesVertexes()[itri].x ];
						col2 = mmesh->getCoordsColors()[ mmesh->getIndicesVertexes()[itri].y ];
						col3 = mmesh->getCoordsColors()[ mmesh->getIndicesVertexes()[itri].z ];
					}
					else
					{
						col1=col2=col3= ChVector<float>(1.f,1.f,1.f);
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
				irrmesh->setHardwareMappingHint(irr::scene::EHM_NEVER);//EHM_DYNAMIC); //for faster hw mapping
				irrmesh->recalculateBoundingBox();
			}


		}


		virtual ESCENE_NODE_TYPE getType() const {  return (ESCENE_NODE_TYPE) ESNT_CHIRRNODEPROXYTOASSET; }	
};






} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

#endif

