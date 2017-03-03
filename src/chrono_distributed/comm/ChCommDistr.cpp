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
#include "chrono_distributed/ChDistributedDataManager.h"

#include "chrono_parallel/ChDataManager.h"

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
	ddm = my_sys->ddm;
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
	num_sendup = 0;
	num_senddown = 0;

	// Identify bodies that need to be updated.
	int num_elements = 0;
	ChDomainDistr *domain = my_sys->domain;

	// TODO OpenMP ??
	// PACKING and UPDATING comm_status of existing bodies
	for (int i = 0; i < data_manager->num_rigid_bodies; i++)
	{
		// Skip empty bodies
		int curr_status = ddm->comm_status[i];
		if (curr_status == chrono::EMPTY)
		{
			continue;
		}

		int location = domain->GetCommStatus(i);

		// If the body now affects the next subdomain:
		if (location == chrono::GHOST_UP || location == chrono::SHARED_UP)
		{
			if (curr_status == chrono::SHARED_UP)
			{
				// If the body has already been shared, it need only update its corresponding ghost
				num_sendup += PackUpdate(sendup_buf + num_sendup, i);
			}
			else
			{
				// If the body is being marked as shared for the first time, the whole body must
				// be packed to create a ghost on another rank
				num_sendup += PackExchange(sendup_buf + num_sendup, i);
				ddm->comm_status[i] = chrono::SHARED_UP;
			}
		}
		// If the body now affects the previous subdomain:
		else if (location == chrono::GHOST_DOWN || location == chrono::SHARED_DOWN)
		{
			if (curr_status == chrono::SHARED_DOWN)
			{
				// If the body has already been shared, it need only update its corresponding ghost
				num_senddown += PackUpdate(senddown_buf + num_senddown, i);
			}
			else
			{
				// If the body is being marked as shared for the first time, the whole body must
				// be packed to create a ghost on another rank
				num_senddown += PackExchange(senddown_buf + num_senddown, i);
				ddm->comm_status[i] = chrono::SHARED_DOWN;
			}
		}
		// If the body is no longer involved with this rank, it must be removed from this rank
		else if (location == chrono::UNOWNED_UP || location == chrono::UNOWNED_DOWN)
		{
			ddm->comm_status[i] = chrono::EMPTY;
			data_manager->host_data.active_rigid[i] = 0;
		}
		// If the body no longer affects either of its neighbor subdomains:
		else if (location == chrono::OWNED)
		{
			if (curr_status == chrono::SHARED_UP || curr_status == chrono::SHARED_DOWN)
			{
				ddm->comm_status[i] = chrono::OWNED;
			}
		}
	} // End of packing for loop


	// SENDING
	MPI_Status statusup;
	MPI_Status statusdown;

	// Send empty message if there is nothing to send
	if (num_sendup == 0)
	{
		sendup_buf[0] = 0;
		num_sendup = 1;
	}

	// send up
	if (my_sys->GetMyRank() != my_sys->GetRanks() - 1)
	{
		GetLog() << "Sending up: " << num_sendup << " from rank " << my_sys->GetMyRank() << "\n";
		MPI_Send(sendup_buf, num_sendup, MPI_DOUBLE, my_sys->GetMyRank() + 1, 1, my_sys->world);
	}

	// Send empty message if there is nothing to send
	if (num_senddown == 0)
	{
		senddown_buf[0] = 0;
		num_senddown = 1;
	}

	// send down
	if (my_sys->GetMyRank() != 0)
	{
		GetLog() << "Sending down: " << num_senddown << " from rank " << my_sys->GetMyRank() << "\n";
		MPI_Send(senddown_buf, num_senddown, MPI_DOUBLE, my_sys->GetMyRank() - 1, 2, my_sys->world);
	}

	GetLog() << "Done sending on rank " << my_sys->GetMyRank() << "\n";



	// RECEIVING // TODO: add the comm status on unpacking ***********************
	int num_recvdown;
	int num_recvup;
	int first_empty = 0; // First index in the data manager that is empty
	int n = 0; // Current index in the recved buffer

	// probe for message from down
	double *recvdown_buf = NULL;


	if (my_sys->GetMyRank() != 0)
	{
		MPI_Probe(my_sys->GetMyRank() - 1, 1, my_sys->world, &statusdown);
		MPI_Get_count(&statusdown, MPI_DOUBLE, &num_recvdown);
		recvdown_buf = new double[num_recvdown];
		MPI_Recv(recvdown_buf, num_recvdown, MPI_DOUBLE, my_sys->GetMyRank() - 1, 1, my_sys->world, &statusdown);
	}

	// Process message from down, then check for message from up. (or thread to do both at the same time)
	if (my_sys->GetMyRank() != 0 && recvdown_buf[0] != 0)
	{
		while (n < num_recvdown)
		{
			// Find the next empty slot in the data manager. TODO: update to a hash
			while(data_manager->host_data.active_rigid[first_empty] && first_empty < data_manager->num_rigid_bodies)
			{
				first_empty++;
			}

			// TODO: anything to clear the space??
			GetLog() << "Processing GID: " << (unsigned int) recvdown_buf[n+2] << "on rank " << my_sys->GetMyRank() << "\n";

			std::shared_ptr<ChBody> body;
			// If this body is being added to the rank
			if ((int) recvdown_buf[n+1] == chrono::EXCHANGE)
			{
				if (first_empty == data_manager->num_rigid_bodies)
				{
					body = std::make_shared<ChBody>();
				}
				else
				{
					body = (*data_manager->body_list)[first_empty];
				}

				n += UnpackExchange(recvdown_buf + n, body);

				if (first_empty == data_manager->num_rigid_bodies)
				{
					my_sys->AddBody(body);
				}
			}
			// If this body is an update for an existing body
			else
			{
				// Find the existing body // TODO: Better search
				for (int i = 0; i < data_manager->num_rigid_bodies; i++)
				{
					if (ddm->global_id[i] == (unsigned int) recvdown_buf[n+2] && ddm->comm_status[i] != chrono::EMPTY)
					{
						body = (*data_manager->body_list)[i];
					//	GetLog() << "A Found/updating GID: " << (unsigned int) recvdown_buf[n+2] << " at index " << i << " on rank " << my_sys->GetMyRank() << "\n";
						break;
					}
				}
				n += UnpackUpdate(recvdown_buf + n, body);
			}
		}
	}

	delete recvdown_buf;
	recvdown_buf = NULL;

	// probe for a message from up
	double *recvup_buf = NULL;
	if (my_sys->GetMyRank() != my_sys->GetRanks() - 1)
	{
		MPI_Probe(my_sys->GetMyRank() + 1, 2, my_sys->world, &statusup);
		MPI_Get_count(&statusup, MPI_DOUBLE, &num_recvup);
		recvup_buf = new double[num_recvup];
		MPI_Recv(recvup_buf, num_recvup, MPI_DOUBLE, my_sys->GetMyRank() + 1, 2, my_sys->world, &statusup);
	}

	n = 0;
	// Process message from up.
	if (my_sys->GetMyRank() != my_sys->GetRanks() - 1 && recvup_buf[0] != 0)
	{
		while (n < num_recvup)
		{
			// Find the next empty slot in the data manager. TODO: update to a hash
			while(data_manager->host_data.active_rigid[first_empty] && first_empty < data_manager->num_rigid_bodies)
			{
				first_empty++;
			}

			GetLog() << "Processing GID: " << (unsigned int) recvup_buf[n+2] << "on rank " << my_sys->GetMyRank() << "\n";
			// TODO: anything to clear the space??

			std::shared_ptr<ChBody> body;
			// If this body is being added to the rank
			if ((int) recvup_buf[n+1] == chrono::EXCHANGE)
			{
				if (first_empty == data_manager->num_rigid_bodies)
				{
					body = std::make_shared<ChBody>();
				}
				else
				{
					body = (*data_manager->body_list)[first_empty];
				}
				n += UnpackExchange(recvup_buf + n, body);
				if (first_empty == data_manager->num_rigid_bodies)
				{
					my_sys->AddBody(body);
				}
			}
			// If this body is an update for an existing body
			else
			{
				// Find the existing body todo better search
				bool found = false;
				for (int i = 0; i < data_manager->num_rigid_bodies; i++)
				{
					if (ddm->global_id[i] == (unsigned int) recvup_buf[n+2] && ddm->comm_status[i] != chrono::EMPTY)
					{
						body = (*data_manager->body_list)[i];
						//GetLog() << "B Found/updating GID: " << (unsigned int) recvup_buf[n+2] << " at index " << i << " on rank " << my_sys->GetMyRank() << "\n";
						found = true;
						break;
					}
				}
				if (found == false) GetLog() << "ERROR: body not found for upating. GID " << (unsigned int) recvup_buf[n+2] << " on rank " << my_sys->GetMyRank() << "\n";
				n += UnpackUpdate(recvup_buf + n, body); // TODO: SEGFAULT body null?? ADD AN INHERITED UPDATE THAT ADDS THE GID FROM THE BODYLIST TO THE DATA_MANAGER
			}
		}
	}

	delete recvup_buf;

	GetLog() << "Done Receiving on rank " << my_sys->GetMyRank() << "\n";
	MPI_Barrier(my_sys->world);
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

	GetLog() << "PackExchange GID: " << (unsigned int) ddm->global_id[index] << " on rank " << my_sys->GetMyRank() << "\n";

	buf[m++] = (double) chrono::EXCHANGE;

	// Global Id
	buf[m++] = (double) ddm->global_id[index];

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
	buf[m++] = data_manager->host_data.mu[index];
	buf[m++] = data_manager->host_data.adhesionMultDMT_data[index];

    if (data_manager->settings.solver.use_material_properties) {
    	buf[m++] = data_manager->host_data.elastic_moduli[index].x;
    	buf[m++] = data_manager->host_data.elastic_moduli[index].y;
    	buf[m++] = data_manager->host_data.cr[index];
    }
    else
    {
    	buf[m++] = data_manager->host_data.dem_coeffs[index].x;
    	buf[m++] = data_manager->host_data.dem_coeffs[index].y;
    	buf[m++] = data_manager->host_data.dem_coeffs[index].z;
    	buf[m++] = data_manager->host_data.dem_coeffs[index].w;
    }


	// Contact Goemetries
	// TODO how to get at the geometry in data manager?? *******************



	buf[0] = (double) m;

	return m;
}



// TODO check if all of these things will be automatically updated into the data_manager ********************************************


// Unpacks a sphere body from the buffer into body.
// Note: body is meant to be a ptr into the data structure
// where the body should be unpacked.
int ChCommDistr::UnpackExchange(double *buf, std::shared_ptr<ChBody> body)
{
	int m = 2; // Skip the size value and the type value

	// Global Id
	body->SetGid((unsigned int) buf[m++]);

	// Position and rotation
	body->SetPos(ChVector<double>(buf[m],buf[m+1],buf[m+2]));
	m += 3;

	// Rotation
	body->SetRot(ChQuaternion<double>(buf[m],buf[m+1],buf[m+2],buf[m+3]));
	m += 4;

	// Linear and Angular Velocities
    body->Variables().Get_qb().SetElementN(0, buf[m++]);
    body->Variables().Get_qb().SetElementN(1, buf[m++]);
    body->Variables().Get_qb().SetElementN(2, buf[m++]);

    body->Variables().Get_qb().SetElementN(3, buf[m++]);
    body->Variables().Get_qb().SetElementN(4, buf[m++]);
    body->Variables().Get_qb().SetElementN(5, buf[m++]);

	// Mass
	body->SetMass(buf[m++]);

	// Material DEM
	body->GetMaterialSurfaceDEM()->static_friction = buf[m++]; //TODO: static or sliding???

	body->GetMaterialSurfaceDEM()->adhesionMultDMT = buf[m++];

    if (data_manager->settings.solver.use_material_properties) {
    	body->GetMaterialSurfaceDEM()->young_modulus = buf[m++];
    	body->GetMaterialSurfaceDEM()->poisson_ratio = buf[m++];
    	body->GetMaterialSurfaceDEM()->restitution = buf[m++];

    }
    else
    {
    	body->GetMaterialSurfaceDEM()->kn = buf[m++]; //TODO ORDER of dem_coeffs from packing??
    	body->GetMaterialSurfaceDEM()->kt = buf[m++];
    	body->GetMaterialSurfaceDEM()->gn = buf[m++];
    	body->GetMaterialSurfaceDEM()->gt = buf[m++];
    }

	return m;
}


// Only packs the essentials for a body update
int ChCommDistr::PackUpdate(double *buf, int index)
{
	int m = 1;

	GetLog() << "PackUpdate GID: " << (unsigned int) ddm->global_id[index] << " on rank " << my_sys->GetMyRank() << "\n";

	buf[m++] = (double) chrono::UPDATE;

	// Global Id
	buf[m++] = (double) ddm->global_id[index];

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

	buf[0] = (double) m;
	return m;
}

int ChCommDistr::UnpackUpdate(double *buf, std::shared_ptr<ChBody> body)
{
	int m = 2; // Skip size value and type value

	// Global Id
	body->SetGid((unsigned int) buf[m++]);

	// Position
	body->SetPos(ChVector<double>(buf[m],buf[m+1],buf[m+2]));
	m += 3;

	// Rotation
	body->SetRot(ChQuaternion<double>(buf[m],buf[m+1],buf[m+2],buf[m+3]));
	m += 4;

	// Linear and Angular Velocities
    body->Variables().Get_qb().SetElementN(0, buf[m++]);
    body->Variables().Get_qb().SetElementN(1, buf[m++]);
    body->Variables().Get_qb().SetElementN(2, buf[m++]);

    body->Variables().Get_qb().SetElementN(3, buf[m++]);
    body->Variables().Get_qb().SetElementN(4, buf[m++]);
    body->Variables().Get_qb().SetElementN(5, buf[m++]);

	return m;
}
