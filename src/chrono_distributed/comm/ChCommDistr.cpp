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

#include <mpi.h>
#include <memory>

#include "chrono_distributed/comm/ChCommDistr.h"
#include "chrono_distributed/physics/ChSystemDistr.h"
#include "chrono_distributed/collision/other_types.h"

#include "chrono_parallel/ChDataManager.h"

#include "chrono/physics/ChBody.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/physics/ChMaterialSurfaceDEM.h"

namespace comm {
enum COMM_STATUS {
	OWNED,
	GHOST,
	SHARED
};
}

namespace chrono {

ChCommDistr::ChCommDistr(std::shared_ptr<ChSystemDistr> my_sys)
{
	this->my_sys = my_sys;
	this->data_manager = my_sys->data_manager;

	//TODO how large for the bufs?
	send_buf = new double[1000000];
	num_send = 0;
	recv_buf = new double[1000000];
	num_recv = 0;
	doubles_per_body = 15;
}

ChCommDistr::~ChCommDistr()
{
	delete send_buf;
	delete recv_buf;
}

// Locate each body that has left the subdomain,
// remove it,
// and pack it for communication.
//TODO
void ChCommDistr::Exchange()
{
	// Do the non-blocking recv


	// Identify bodies that need to be updated.

	int num_elements = 0;
	ChDomainDistr *domain = my_sys->domain;

	for (int i = 0; i < data_manager->num_rigid_bodies; i++)
	{
		// Skip inactive bodies
		if (data_manager->host_data.active_rigid == 0)
		{
			continue;
		}

		// If a body has passed into the ghost
		// region mark it as a shared
		if(domain->InGhost(my_sys->bodylist[i]))
		{
			// TODO: No need to send now, this could be done with the ghost update
			my_sys->bodylist[i]->SetId(comm::SHARED);
		}

		// The body has completely left
		else if (domain->InSub(my_sys->bodylist[i]))
		{

		}
	}

	//TODO:
	// Check local bodies
		// Send bodies that have left
	// Recv incoming bodies.
}


// Called when a sphere leaves this rank's subdomain.
// The body should have been removed from the local system's
// list after this function is called.
// Packages the body into buf.
// Returns the number of elements which the body took in the buffer
int ChCommDistr::PackExchange(double *buf, int index)
{
	int m = 1; // Number of doubles being packed (will be place in the front of the buffer
			   // once packing is done

	int dof = data_manager->num_dof;

	// Global Id //TODO??
	buf[m++] = (double) data_manager->body_list->at(index)->gid;

	// Position and rotation
	real3 pos = data_manager->host_data.pos_rigid[index];
	buf[m++] = pos.x;
	buf[m++] = pos.y;
	buf[m++] = pos.z;

	// Rotation
	quaternion rot = data_manager->host_data.rot_rigid[index];
	buf[m++] = rot.w;
	buf[m++] = rot.x;
	buf[m++] = rot.y;
	buf[m++] = rot.z;

	// Velocity
	real3 vel = data_manager->host_data.vel_3dof[index];
	buf[m++] = vel.x;
	buf[m++] = vel.y;
	buf[m++] = vel.z;

	// Angular Velocity
	data_manager->host_data.v[index*dof + 3];
	buf[m++] = 	data_manager->host_data.v[index*dof + 3];
	buf[m++] = 	data_manager->host_data.v[index*dof + 4];
	buf[m++] = 	data_manager->host_data.v[index*dof + 5];
	buf[m++] = 	data_manager->host_data.v[index*dof + 6];

	// Mass
	buf[m++] = data_manager->host_data.mass_rigid[index];

	// Material DEM
	buf[m++] = data_manager->host_data.elastic_moduli[index].x;
	buf[m++] = data_manager->host_data.elastic_moduli[index].y;
	buf[m++] = data_manager->host_data.mu[index];
	buf[m++] = data_manager->host_data.cr[index];
	buf[m++] = data_manager->host_data.dem_coeffs[index].w;
	buf[m++] = data_manager->host_data.dem_coeffs[index].x;
	buf[m++] = data_manager->host_data.dem_coeffs[index].y;
	buf[m++] = data_manager->host_data.dem_coeffs[index].z;
	buf[m++] = data_manager->host_data.adhesionMultDMT_data[index];

	// Fixed
	buf[m++] = (double) data_manager->body_list->at(index)->GetBodyFixed();;

	// Contact Goemetries
	// TODO how to get at the geometry in data manager??
	buf[0] = (double) m;

	return m;
}

// Unpacks a sphere body from the buffer into body.
// Note: body is meant to be a ptr into the data structure
// where the body should be unpacked.
void ChCommDistr::UnpackExchange(double *buf, std::shared_ptr<ChBody> body)
{
	//TODO:

	int m = 1;

	// Global Id //TODO??
	body->gid = buf[m++];

	//buf[m++] = data_manager->body_list->at(index)->identifier;

	// Position and rotation
	body->SetPos(ChVector<double>(buf[m],buf[m+1],buf[m+2]));
	m += 3;

	// real3 pos = data_manager->host_data.pos_rigid[index];
	// buf[m++] = pos.x;
	// buf[m++] = pos.y;
	// buf[m++] = pos.z;

	// Rotation
	body->SetRot(ChQuaternion<double>(buf[m],buf[m+1],buf[m+2],buf[m+3]));
	m += 4;

	/*
	quaternion rot = data_manager->host_data.rot_rigid[index];
	buf[m++] = rot.w;
	buf[m++] = rot.x;
	buf[m++] = rot.y;
	buf[m++] = rot.z;
	*/

	// Velocity
	body->SetPos_dt(ChVector<double>(buf[m],buf[m+1],buf[m+2]));
	m += 3;

	/*
	real3 vel = data_manager->host_data.vel_3dof[index];
	buf[m++] = vel.x;
	buf[m++] = vel.y;
	buf[m++] = vel.z;
	*/

	// Angular Velocity
	body->SetRot_dt(ChQuaternion<double>(buf[m],buf[m+1],buf[m+2],buf[m+4]));
	m += 4;

	/*
	quaternion omega = data_manager->host_data.rot_rigid[index];
	buf[m++] = omega.w;
	buf[m++] = omega.x;
	buf[m++] = omega.y;
	buf[m++] = omega.z;
	*/

	// Mass
	body->SetMass(buf[m++]);

	// buf[m++] = data_manager->host_data.mass_rigid[index];

	// Material DEM
	auto mat = std::make_shared<ChMaterialSurfaceDEM>();

	body->GetMaterialSurfaceDEM()->young_modulus = buf[m++];
	body->GetMaterialSurfaceDEM()->poisson_ratio = buf[m++];
	body->GetMaterialSurfaceDEM()->static_friction = buf[m++]; //static or sliding???
	body->GetMaterialSurfaceDEM()->restitution = buf[m++];
	body->GetMaterialSurfaceDEM()->kn = buf[m++]; //ORDER of dem_coeffs??
	body->GetMaterialSurfaceDEM()->kt = buf[m++];
	body->GetMaterialSurfaceDEM()->gn = buf[m++];
	body->GetMaterialSurfaceDEM()->gt = buf[m++];
	body->GetMaterialSurfaceDEM()->adhesionMultDMT = buf[m++];

	/*
	buf[m++] = data_manager->host_data.elastic_moduli[index].x;
	buf[m++] = data_manager->host_data.elastic_moduli[index].y;
	buf[m++] = data_manager->host_data.mu[index];
	buf[m++] = data_manager->host_data.cr[index];
	buf[m++] = data_manager->host_data.dem_coeffs[index].w;
	buf[m++] = data_manager->host_data.dem_coeffs[index].x;
	buf[m++] = data_manager->host_data.dem_coeffs[index].y;
	buf[m++] = data_manager->host_data.dem_coeffs[index].z;
	buf[m++] = data_manager->host_data.adhesionMultDMT_data[index];
	*/


	// Fixed
	body->SetBodyFixed((bool) buf[m++]);

	// buf[m++] = (double) data_manager->body_list->at(index)->GetBodyFixed();;


}

// Only packs the essentials for a body update
int ChCommDistr::PackUpdate(double *buf, int index)
{
	int m = 1;


	int dof = data_manager->num_dof;

	// Global Id
	buf[m++] = (double) data_manager->body_list->at(index)->gid;

	// Position
	buf[m++] = data_manager->host_data.pos_rigid[index].x;
	buf[m++] = data_manager->host_data.pos_rigid[index].y;
	buf[m++] = data_manager->host_data.pos_rigid[index].z;

	// Rotation
	buf[m++] = data_manager->host_data.rot_rigid[index].w;
	buf[m++] = data_manager->host_data.rot_rigid[index].x;
	buf[m++] = data_manager->host_data.rot_rigid[index].y;
	buf[m++] = data_manager->host_data.rot_rigid[index].z;

	// Velocity
	buf[m++] = data_manager->host_data.vel_3dof[index].w;
	buf[m++] = data_manager->host_data.vel_3dof[index].x;
	buf[m++] = data_manager->host_data.vel_3dof[index].y;
	buf[m++] = data_manager->host_data.vel_3dof[index].z;

	// Angular Velocity
	data_manager->host_data.v[index*dof + 3];
	buf[m++] = 	data_manager->host_data.v[index*dof + 3];
	buf[m++] = 	data_manager->host_data.v[index*dof + 4];
	buf[m++] = 	data_manager->host_data.v[index*dof + 5];
	buf[m++] = 	data_manager->host_data.v[index*dof + 6];

	buf[0] = m;
	return m;
}

void ChCommDistr::UnpackExchange(double *buf, std::shared_ptr<ChBody> body)
{
	int m = 1;

	int dof = data_manager->num_dof;

	// Global Id
	body->gid = (unsigned int) buf[m++];

	// buf[m++] = (double) data_manager->body_list->at(index)->gid;

	// Position
	body->SetPos(ChVector<double>(buf[m],buf[m+1],buf[m+2]));
	m += 3;

	/*
	buf[m++] = data_manager->host_data.pos_rigid[index].x;
	buf[m++] = data_manager->host_data.pos_rigid[index].y;
	buf[m++] = data_manager->host_data.pos_rigid[index].z;
	*/

	// Rotation
	body->SetRot(ChQuaternion<double>(buf[m],buf[m+1],buf[m+2],buf[m+3]));
	m += 4;

	/*
	buf[m++] = data_manager->host_data.rot_rigid[index].w;
	buf[m++] = data_manager->host_data.rot_rigid[index].x;
	buf[m++] = data_manager->host_data.rot_rigid[index].y;
	buf[m++] = data_manager->host_data.rot_rigid[index].z;
	 */


	// Velocity
	body->SetPos_dt(ChVector<double>(buf[m],buf[m+1],buf[m+2]));
	m += 3;

	/*
	buf[m++] = data_manager->host_data.vel_3dof[index].w;
	buf[m++] = data_manager->host_data.vel_3dof[index].x;
	buf[m++] = data_manager->host_data.vel_3dof[index].y;
	buf[m++] = data_manager->host_data.vel_3dof[index].z;
	*/


	// Angular Velocity
	body->SetRot_dt(ChQuaternion<double>(buf[m],buf[m+1],buf[m+2],buf[m+3]));
	m += 4;

	/*
	data_manager->host_data.v[index*dof + 3];
	buf[m++] = 	data_manager->host_data.v[index*dof + 3];
	buf[m++] = 	data_manager->host_data.v[index*dof + 4];
	buf[m++] = 	data_manager->host_data.v[index*dof + 5];
	buf[m++] = 	data_manager->host_data.v[index*dof + 6];
	*/
}


} /* namespace chrono */
