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

#ifndef CHRONO_DISTRIBUTED_COLLISION_CHDATAMANAGERDISTR_H_
#define CHRONO_DISTRIBUTED_COLLISION_CHDATAMANAGERDISTR_H_

#include "chrono_distributed/physics/ChSystemDistr.h"
#include "chrono_distributed/collision/ChBroadphaseDistr.h"
#include "chrono/physics/ChBody.h"

#include <memory>

namespace chrono {

typedef struct triangle_node {
	// TODO
	double vertices[3][3]; // Locations of each vertex
	double force[3]; // ?
	double bari[3]; // ?
} triangle_node;


class ChDataManagerDistr
{
public:
	ChDataManagerDistr(std::shared_ptr<ChSystemDistr> my_sys, int max_triangles = 0, int max_local = 16000, int max_ghost = 10000);
	virtual ~ChDataManagerDistr();

	std::vector<std::shared_ptr<ChBody>> GetLocalList() {return local_bodylist;}
	std::vector<std::shared_ptr<ChBody>> GetSharedList() {return shared_bodylist;}
	std::vector<std::shared_ptr<ChBody>> GetGhostList() {return ghost_bodylist;}
	std::vector<triangle_node> GetTriangles() {return triangles;}

	int GetNumLocal() {return num_local;}
	int GetNumShared() {return num_shared;}
	int GetNumGhost() {return num_ghost;}

	// TODO
	int GetNumTotal() {return num_total;}

	int GetMaxLocal() {return max_local;}
	int GetMaxShared() {return max_shared;}
	int GetMaxGhost() {return max_ghost;}
	int GetMaxTriangles() {return max_triangles;}

	void AddLocal(std::shared_ptr<ChBody> body);

protected:
	std::shared_ptr<ChSystemDistr> my_sys;

	// Number of local bodies.
	int num_local;

	// Number of shared bodies.
	// ie bodies that are integrated on this rank, but exist as
	// a ghost on another rank.
	int num_shared;

	// Number of ghost bodies.
	int num_ghost;

	// Number of total bodies across the global domain.
	int num_total;

	int max_local;
	int max_shared;
	int max_ghost;

	int max_triangles;

	// Arrays of pointers to the bodies
	std::vector<std::shared_ptr<ChBody>> local_bodylist;
	std::vector<std::shared_ptr<ChBody>> shared_bodylist;
	std::vector<std::shared_ptr<ChBody>> ghost_bodylist;
	std::vector<triangle_node> triangles;

	//TODO: Outline interface
};

} /* namespace chrono */

#endif /* CHRONO_DISTRIBUTED_COLLISION_CHDATAMANAGERDISTR_H_ */
