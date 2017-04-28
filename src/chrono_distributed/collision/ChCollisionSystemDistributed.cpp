// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nic Olsen
// =============================================================================

#include <climits>

#include "chrono_distributed/collision/ChCollisionSystemDistributed.h"
#include "chrono_distributed/ChDistributedDataManager.h"

#include "chrono_parallel/collision/ChCollisionSystemParallel.h"
#include "chrono_parallel/collision/ChCollisionModelParallel.h"
#include "chrono_parallel/ChDataManager.h"

using namespace chrono;
using namespace collision;

ChCollisionSystemDistributed::ChCollisionSystemDistributed(ChParallelDataManager *dm, ChDistributedDataManager *ddm) :
	ChCollisionSystemParallel(dm)
{
	this->ddm = ddm;
}

ChCollisionSystemDistributed::~ChCollisionSystemDistributed() {}


// Called by chcollisionmodel::buildmodel (if system set), chbody::setcollide(true), chbody::setsystem (if system set) (called by addbody AND addbodyexchange)
// TODO: only works for initial add. Since it is called so much, it needs to be able to work on transfer too
void ChCollisionSystemDistributed::Add(ChCollisionModel *model)
{

	//TODO: This may just need to gut chCollisionSystemParallel::Add and use it in pieces.




	ChParallelDataManager* dm = ddm->data_manager;
    ChCollisionModelParallel* pmodel = static_cast<ChCollisionModelParallel*>(model);


/*
	// Check for free spaces (of the same shape type) to insert into TODO
    for(int i = 0; i < pmodel->nObjects; i++)
    {
    	for (int j = 0; j < dm->shape_data.id_rigid.size(); j++)
    	{
    		// If the index in the data manager is open and corresponds to the same shape type
    		if (dm->shape_data.id_rigid[j] < 0 && dm->shape_data.typ_rigid[j] == pmodel->mData[i].type)
    		{
    			dm->shape_data.id_rigid[j] = pmodel->GetBody()->GetId();
    			int start = dm->
    		}
    	}
	}
*/


	// If no free spaces to insert into
    this->ChCollisionSystemParallel::Add(model); //TODO This adds ALL shapes, but we want to check if there are free spaces for individual shapes before this


   auto id = pmodel->GetBody()->GetId();
   int count = pmodel->GetNObjects();

   ddm->body_shape_count.push_back(count);
   ddm->body_shape_start.push_back(ddm->body_shapes.size()); // TODO: does this work??

/*
   ddm->body_shape_count[id] = count;
   ddm->body_shape_start[id] = ddm->body_shapes.size();
*/

   for (int i = 0; i < pmodel->GetNObjects(); i++)
   {
	   ddm->body_shapes.push_back(ddm->data_manager->num_rigid_shapes - count + i);
	   ddm->my_free_shapes.push_back(false);
	   ddm->dm_free_shapes.push_back(false);
   }
}

void ChCollisionSystemDistributed::Remove(ChCollisionModel *model)
{
	ChCollisionModelParallel* pmodel = static_cast<ChCollisionModelParallel*>(model);

	auto id = pmodel->GetBody()->GetId();
	int count = pmodel->GetNObjects();
	int start = ddm->body_shape_start[id];

	for (int i = 0; i < count; i++)
	{
		int index = start + i;
		ddm->my_free_shapes[index] = true; // Marks the spot in ddm->body_shapes as open
		ddm->dm_free_shapes[ddm->body_shapes[index]] = true; // Marks the spot in data_manager->shape_data as open

		ddm->data_manager->shape_data.id_rigid[ddm->body_shapes[index]] = UINT_MAX; // Forces the broadphase to ignore this shape
		GetLog() << "Changing shape body id to " << ddm->data_manager->shape_data.id_rigid[ddm->body_shapes[index]] << "at index " << ddm->body_shapes[index] << "\n";
	}
}
