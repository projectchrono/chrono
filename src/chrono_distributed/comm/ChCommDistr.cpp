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
#include <new>

#include "chrono_distributed/comm/ChCommDistr.h"
#include "chrono_distributed/physics/ChSystemDistr.h"
#include "chrono_distributed/ChDataManagerDistr.h"

#include "chrono/physics/ChBody.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/physics/ChMaterialSurfaceDEM.h"

using namespace chrono;

ChCommDistr::ChCommDistr(ChSystemDistr *my_sys)
{
	this->my_sys = my_sys;
	this->data_manager = my_sys->data_manager;

	//TODO how large for the bufs?
	sendup_buf = new double[1000000];
	num_sendup = 0;
	senddown_buf = new double[1000000];
	num_senddown = 0;
}

ChCommDistr::~ChCommDistr()
{
	delete sendup_buf;
	delete senddown_buf;
}

// Locate each body that has left the subdomain,
// remove it,
// and pack it for communication.
void ChCommDistr::Exchange()
{
	GetLog() << "Exchange\n";
	num_sendup = 0;
	num_senddown = 0;

	// Identify bodies that need to be updated.

	int num_elements = 0;
	ChDomainDistr *domain = my_sys->domain;

	for (int i = 0; i < data_manager->num_rigid_bodies; i++)
	{
		// Skip inactive bodies
		if (data_manager->host_data.active_rigid[i] == 0)
		{
			continue;
		}

		// If a body has passed into the ghost
		// region, mark it as shared
		int ghost = domain->InGhost(i);
		if(ghost)
		{
			if (ghost == 1)
			{
				PackExchange(sendup_buf + num_sendup, i);
				data_manager->comm_status[i] = communication::SHARED_UP;
			}
			else
			{
				PackExchange(senddown_buf + num_senddown, i);
				data_manager->comm_status[i] = communication::SHARED_DOWN;
			}
		}

		// The body has completely left and was shared before
		else if (!(domain->InSub(i) || domain->InGhost(i)))
		{
			if (data_manager->comm_status[i] == communication::SHARED_UP)
			{
				num_sendup += PackTransferOwner(sendup_buf + num_sendup, i);
			}
			else if (data_manager->comm_status[i] == communication::SHARED_DOWN)
			{
				num_senddown += PackTransferOwner(senddown_buf + num_senddown, i);
			}
			// ERROR: timestep must be small enough that bodies do no jump over the ghost layer
			else
			{
				GetLog() << "Transfer without ghost: ChCommDistr.cpp " << data_manager->global_id[i] << "\n";
			}

			// Removes the body from the local rank
			data_manager->host_data.active_rigid[i] = 0;
		}

	} // End of send for loop

	MPI_Status statusup;
	MPI_Status statusdown;

	// send up
	if (my_sys->GetMyRank() != my_sys->GetRanks() - 1)
	{
		GetLog() << "Sending up: " << num_sendup << " from rank " << my_sys->GetMyRank() << "\n";
		MPI_Send(sendup_buf, num_sendup, MPI_DOUBLE, my_sys->GetMyRank() + 1, 1, my_sys->world);
	}
	// send down
	if (my_sys->GetMyRank() != 0)
	{
		GetLog() << "Sending down: " << num_senddown << " from rank " << my_sys->GetMyRank() << "\n";
		MPI_Send(senddown_buf, num_senddown, MPI_DOUBLE, my_sys->GetMyRank() - 1, 2, my_sys->world);
	}


	int num_recvdown;
	int num_recvup;

	// probe for message from down
	double *recvdown_buf;
	if (my_sys->GetMyRank() != 0)
	{
		MPI_Probe(my_sys->GetMyRank() - 1, 1, my_sys->world, &statusdown);
		MPI_Get_count(&statusdown, MPI_DOUBLE, &num_recvdown);
		recvdown_buf = new double[num_recvdown];
		MPI_Recv(recvdown_buf, num_recvdown, MPI_DOUBLE, my_sys->GetMyRank() - 1, 1, my_sys->world, &statusdown);
	}

	// Process message from down, then check for other message.
	int n = 0;
	int first_empty = 0; // first index in the data manager that is empty
	while (n < num_recvdown)
	{
		// Find the next empty slot in the data manager. TODO: update to a hash
		while(data_manager->host_data.active_rigid[first_empty])
		{
			first_empty++;
		}



		(*data_manager->body_list)[first_empty]->~ChBody();
		new((void*) (*data_manager->body_list)[first_empty].get()) ChBody(); 	// revert the space back to a new ChBody configuration
																// Placement new (constructs in the existing space)
																// TODO: NewBody method in system to ??

		n += UnpackExchange(recvdown_buf + n, (*data_manager->body_list)[first_empty]);
	}


	delete recvdown_buf;
	recvdown_buf = NULL;

	// probe for a message from up
	double *recvup_buf;
	if (my_sys->GetMyRank() != my_sys->GetRanks() - 1)
	{
		MPI_Probe(my_sys->GetMyRank() + 1, 2, my_sys->world, &statusup);
		MPI_Get_count(&statusup, MPI_DOUBLE, &num_recvup);
		recvup_buf = new double[num_recvup];
		MPI_Recv(recvup_buf, num_recvup, MPI_DOUBLE, my_sys->GetMyRank() + 1, 2, my_sys->world, &statusup);
	}

	// Process message from up.

	while (n < num_recvup)
	{
		// Find the next empty slot in the data manager. TODO: update to a hash
		while(data_manager->host_data.active_rigid[first_empty])
		{
			first_empty++;
		}

		(*data_manager->body_list)[first_empty]->~ChBody();
		new((void*) (*data_manager->body_list)[first_empty].get()) ChBody(); 	// revert the space back to a new ChBody configuration
																// Placement new (constructs in the existing space)
																// TODO: NewBody method in system to ??

		n += UnpackExchange(recvup_buf + n, (*data_manager->body_list)[first_empty]);
	}

	delete recvup_buf;
}





// Called when a sphere leaves this rank's subdomain into the ghost layer.
// The body should have been removed from the local system's
// list after this function is called.
// Packages the body into buf.
// Returns the number of elements which the body took in the buffer
int ChCommDistr::PackExchange(double *buf, int index)
{
	int m = 1; // Number of doubles being packed (will be placed in the front of the buffer
			   // once packing is done

	// Global Id
	buf[m++] = (double) data_manager->global_id[index];

	// Position and rotation
	real3 pos = data_manager->host_data.pos_rigid[index];
	buf[m++] = pos.x;
	buf[m++] = pos.y;
	buf[m++] = pos.z;

	// Rotation
	quaternion rot = data_manager->host_data.rot_rigid[index];
	buf[m++] = rot.x;
	buf[m++] = rot.y;
	buf[m++] = rot.z;
	buf[m++] = rot.w;

	// Velocity
	buf[m++] = data_manager->host_data.v[index*6];
	buf[m++] = data_manager->host_data.v[index*6 + 1];
	buf[m++] = data_manager->host_data.v[index*6 + 2];


	// Angular Velocity
	buf[m++] = data_manager->host_data.v[index*6 + 3];
	buf[m++] = data_manager->host_data.v[index*6 + 4];
	buf[m++] = data_manager->host_data.v[index*6 + 5];

	// Mass
	buf[m++] = data_manager->host_data.mass_rigid[index];

	// Material DEM
	buf[m++] = data_manager->host_data.elastic_moduli[index].x;
	buf[m++] = data_manager->host_data.elastic_moduli[index].y;
	buf[m++] = data_manager->host_data.mu[index];
	buf[m++] = data_manager->host_data.cr[index];
	buf[m++] = data_manager->host_data.dem_coeffs[index].x;
	buf[m++] = data_manager->host_data.dem_coeffs[index].y;
	buf[m++] = data_manager->host_data.dem_coeffs[index].z;
	buf[m++] = data_manager->host_data.dem_coeffs[index].w;
	buf[m++] = data_manager->host_data.adhesionMultDMT_data[index];

	// Contact Goemetries
	// TODO how to get at the geometry in data manager?? *******************
	buf[0] = (double) m;

	return m;
}

// Unpacks a sphere body from the buffer into body.
// Note: body is meant to be a ptr into the data structure
// where the body should be unpacked.
int ChCommDistr::UnpackExchange(double *buf, std::shared_ptr<ChBody> body)
{
	if ((unsigned int) buf[0] == 2)
	{
		return UnpackTransferOwner(buf);
	}

	int m = 1;

	// Global Id
	body->SetGid((int)buf[m++]);

	// Position and rotation
	body->SetPos(ChVector<double>(buf[m],buf[m+1],buf[m+2]));
	m += 3;

	// Rotation
	body->SetRot(ChQuaternion<double>(buf[m],buf[m+1],buf[m+2],buf[m+3]));
	m += 4;

	// Velocity
	body->SetPos_dt(ChVector<double>(buf[m],buf[m+1],buf[m+2]));
	m += 3;

	// Angular Velocity
	body->SetRot_dt(ChQuaternion<double>(buf[m],buf[m+1],buf[m+2],buf[m+4]));
	m += 4;

	// Mass
	body->SetMass(buf[m++]);

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
}

// A first buffer value of 2 indicates a transfer of ownership
// of a preexisting ghost
int ChCommDistr::PackTransferOwner(double *buf, int index)
{
	buf[0] = 2;
	buf[1] = (double) data_manager->global_id[index];
	return 2;
}

// Receiving a body whose first buffer value is 2 indicates
// that the body (which was a ghost) is now owned on this rank
int ChCommDistr::UnpackTransferOwner(double *buf)
{
	std::vector<unsigned int>& globals = data_manager->global_id;

	unsigned int gid = (unsigned int) buf[1];
	// TODO better search
	for (int i = 0; i < data_manager->num_rigid_bodies; i++)
	{
		if (globals[i] == gid)
		{
			data_manager->comm_status[i] = communication::OWNED;
			break;
		}
	}
	return 2;
}

// Only packs the essentials for a body update
int ChCommDistr::PackUpdate(double *buf, int index)
{
	int m = 1;


	// Global Id
	buf[m++] = (double) data_manager->global_id[index];

	// Position
	buf[m++] = data_manager->host_data.pos_rigid[index].x;
	buf[m++] = data_manager->host_data.pos_rigid[index].y;
	buf[m++] = data_manager->host_data.pos_rigid[index].z;

	// Rotation
	buf[m++] = data_manager->host_data.rot_rigid[index].x;
	buf[m++] = data_manager->host_data.rot_rigid[index].y;
	buf[m++] = data_manager->host_data.rot_rigid[index].z;
	buf[m++] = data_manager->host_data.rot_rigid[index].w;

	// Velocity
	buf[m++] = data_manager->host_data.v[index*6];
	buf[m++] = data_manager->host_data.v[index*6 + 1];
	buf[m++] = data_manager->host_data.v[index*6 + 2];

	// Angular Velocity
	buf[m++] = 	data_manager->host_data.v[index*6 + 3];
	buf[m++] = 	data_manager->host_data.v[index*6 + 4];
	buf[m++] = 	data_manager->host_data.v[index*6 + 5];

	buf[0] = m;
	return m;
}

int ChCommDistr::UnpackUpdate(double *buf, std::shared_ptr<ChBody> body)
{
	int m = 1;

	// Global Id
	body->SetGid((int)buf[m++]);

	// Position
	body->SetPos(ChVector<double>(buf[m],buf[m+1],buf[m+2]));
	m += 3;

	// Rotation
	body->SetRot(ChQuaternion<double>(buf[m],buf[m+1],buf[m+2],buf[m+3]));
	m += 4;


	// Velocity
	body->SetPos_dt(ChVector<double>(buf[m],buf[m+1],buf[m+2]));
	m += 3;


	// Angular Velocity
	body->SetRot_dt(ChQuaternion<double>(buf[m],buf[m+1],buf[m+2],buf[m+3]));
	m += 4;

	return m;
}
