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

#include "chrono_distributed/comm/ChCommDistributed.h"
#include "chrono_distributed/physics/ChSystemDistributed.h"
#include "chrono_distributed/ChDistributedDataManager.h"
#include "chrono_distributed/collision/ChCollisionModelDistributed.h"
#include "chrono_distributed/collision/ChCollisionSystemDistributed.h"
#include "chrono_distributed/other_types.h"

#include "chrono_parallel/ChDataManager.h"

#include "chrono/physics/ChBody.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/physics/ChMaterialSurfaceSMC.h"

using namespace chrono;
using namespace collision;

ChCommDistributed::ChCommDistributed(ChSystemDistributed *my_sys)
{
	this->my_sys = my_sys;
	this->data_manager = my_sys->data_manager;

	//TODO how large for the bufs? put on stack?
	sendup_buf = new double[1000000];
//	sendup_buf = new unsigned char[1000000];
	num_sendup = 0;
	senddown_buf = new double[1000000];
//	senddown_buf = new unsigned char[1000000];
	num_senddown = 0;
	ddm = my_sys->ddm;
}

ChCommDistributed::~ChCommDistributed()
{
	delete sendup_buf;
	delete senddown_buf;
}

// Processes an incoming message from another rank.
// updown determines if this buffer was received from the rank above this rank
// or below this rank. updown = 1 ==> up, updown = 0 ==> down
void ChCommDistributed::ProcessBuffer(int num_recv, double* buf, int updown)
{
	int n = 0; // Current index in the recved buffer
	while (n < num_recv)
	{
		// Find the next empty slot in the data manager.
		if (ddm->comm_status[ddm->first_empty] != distributed::EMPTY)
		{
			while (ddm->first_empty < data_manager->num_rigid_bodies
					&& ddm->comm_status[ddm->first_empty] != distributed::EMPTY)
			{
				ddm->first_empty++;
			}
		}

		std::shared_ptr<ChBody> body;
		// If this body is being added to the rank
//		int message_type;
//		std::memcpy(&message_type, buf + sizeof(int), sizeof(int));
//		if (message_type == distributed::EXCHANGE)
		if ((int) (buf[n+1]) == distributed::EXCHANGE)
		{
			// If there are no empty spaces in the data manager, create
			// a new body to add
			if (ddm->first_empty == data_manager->num_rigid_bodies)
			{
				body = std::make_shared<ChBody>(std::make_shared<ChCollisionModelDistributed>(), ChMaterialSurface::SMC);
				body->SetId(data_manager->num_rigid_bodies);
			}
			// If an empty space was found in the body manager
			else
			{
				body = (*data_manager->body_list)[ddm->first_empty];
			}

			n += UnpackExchange(buf + n, body);

			// Add the new body
			distributed::COMM_STATUS status = (updown == 1) ? distributed::GHOST_UP : distributed::GHOST_DOWN;
			if (ddm->first_empty == data_manager->num_rigid_bodies)
			{
				my_sys->AddBodyExchange(body, status);
			}
			else
			{
				ddm->comm_status[ddm->first_empty] = status;
				body->SetBodyFixed(false);
			}
		}

		// If this body is an update for an existing body
//		else if (message_type == distributed::UPDATE || message_type == distributed::FINAL_UPDATE_GIVE)
		else if ((int) buf[n+1] == distributed::UPDATE || (int) buf[n+1] == distributed::FINAL_UPDATE_GIVE)
		{
			// Find the existing body // TODO: Better search
			bool found = false;
			unsigned int gid = (unsigned int) buf[n+2];
//			unsigned int gid;
//			std::memcpy(&gid, buf + 2*sizeof(int), sizeof(uint));
			for (int i = 0; i < data_manager->num_rigid_bodies; i++)
			{
				if (ddm->global_id[i] == gid
						&& ddm->comm_status[i] != distributed::EMPTY)
				{
					if (ddm->comm_status[i] != distributed::GHOST_UP && ddm->comm_status[i] != distributed::GHOST_DOWN)
					{
						GetLog() << "Trying to update a non-ghost body on rank " << my_sys->GetMyRank() << "\n";
					}
					body = (*data_manager->body_list)[i];
					found = true;
//					if (message_type == distributed::FINAL_UPDATE_GIVE)
					if ((int) buf[n+1] == distributed::FINAL_UPDATE_GIVE)
					{
						ddm->comm_status[i] = distributed::OWNED;
//						GetLog() << "Owning gid " << gid << " on rank " << my_sys->GetMyRank() << "\n";
//						my_sys->PrintBodyStatus();
					}
					break;
				}
			}

			if (found)
			{
				n += UnpackUpdate(buf + n, body);
			}
			else
			{
				GetLog() << "GID " << gid << " NOT found rank "
						<< my_sys->GetMyRank() << "\n";
				my_sys->ErrorAbort("Body to be updated not found\n");
			}
		}
//		else if (message_type == distributed::FINAL_UPDATE_TAKE)
		else if ((int) buf[n+1] == distributed::FINAL_UPDATE_TAKE)
		{
			unsigned int gid = (unsigned int) buf[n+2];
//			unsigned int gid;
//			std::memcpy(&gid, buf + 2*sizeof(int), sizeof(uint));
			int id = ddm->GetLocalIndex(gid);
//			GetLog() << "Take:  " << gid << ", id " << id << " rank " << my_sys->GetMyRank() << "\n";

			my_sys->RemoveBodyExchange(id);
//			n += 2*sizeof(int) + sizeof(uint);
			n += 3;
		}
		else
		{
			GetLog() << "Undefined message type rank " << my_sys->GetMyRank() << "\n";
		}
	}
}

// TODO could do packing with openmp and paste the buffers together
// Locate each body that has left the sub-domain,
// remove it,
// and pack it for communication.
void ChCommDistributed::Exchange()
{
	MPI_Barrier(my_sys->world);
	num_sendup = 0;
	num_senddown = 0;
	int my_rank = my_sys->GetMyRank();
	int num_ranks = my_sys->GetNumRanks();

	ddm->data_manager->system_timer.start("Send");
	// PACKING and UPDATING comm_status of existing bodies
	for (int i = 0; i < data_manager->num_rigid_bodies; i++)
	{
		// Skip empty bodies or those that this rank isn't responsible for
		int curr_status = ddm->comm_status[i];
		int location = my_sys->domain->GetBodyRegion(i);

		if (curr_status == distributed::GLOBAL
				|| curr_status == distributed::EMPTY
				|| curr_status == distributed::GHOST_UP
				|| curr_status == distributed::GHOST_DOWN)
			continue;


		// If the body now affects the next sub-domain
		if (location == distributed::GHOST_UP || location == distributed::SHARED_UP)
		{
			// If the body has already been shared, it need only update its corresponding ghost
			if (curr_status == distributed::SHARED_UP)
			{
//				GetLog() << "Update: rank " << my_rank
//						<< " -- " << ddm->global_id[i] << " --> rank "
//						<< my_rank + 1 << "\n";
				num_sendup += PackUpdate(sendup_buf + num_sendup, i, distributed::UPDATE);
			}
			// If the body is being marked as shared for the first time, the whole body must
			// be packed to create a ghost on another rank
			else if (curr_status == distributed::OWNED)
			{
				GetLog() << "Exchange: rank " << my_rank
						<< " -- " << ddm->global_id[i] << " --> rank " <<
						my_rank + 1 << "\n";
				num_sendup += PackExchange(sendup_buf + num_sendup, i);
				ddm->comm_status[i] = distributed::SHARED_UP;
			}
		}

		// If the body now affects the previous sub-domain:
		else if (location == distributed::GHOST_DOWN || location == distributed::SHARED_DOWN)
		{
			if (curr_status == distributed::SHARED_DOWN)
			{
				// If the body has already been shared, it need only update its corresponding ghost
//				GetLog() << "Update: rank " << my_rank << " -- "
//						<< ddm->global_id[i] << " --> rank "
//						<< my_rank - 1 << "\n";
				num_senddown += PackUpdate(senddown_buf + num_senddown, i, distributed::UPDATE);
			}
			else if (curr_status == distributed::OWNED)
			{
				// If the body is being marked as shared for the first time, the whole body must
				// be packed to create a ghost on another rank
				GetLog() << "Exchange: rank " << my_rank
						<< " -- " << ddm->global_id[i] << " --> rank "
						<< my_rank - 1 << "\n";
				num_senddown += PackExchange(senddown_buf + num_senddown, i);
				ddm->comm_status[i] = distributed::SHARED_DOWN;
			}
		}

		// If the body is no longer involved with this rank, it must be removed from this rank
		else if (location == distributed::UNOWNED_UP || location == distributed::UNOWNED_DOWN)
		{
			int up;
			if (location == distributed::UNOWNED_UP && my_rank != num_ranks - 1)
			{
				num_sendup += PackUpdate(sendup_buf + num_sendup, i, distributed::FINAL_UPDATE_GIVE);
				up = 1;
			}
			else if (location == distributed::UNOWNED_DOWN && my_rank != 0)
			{
				num_senddown += PackUpdate(senddown_buf + num_senddown, i, distributed::FINAL_UPDATE_GIVE);
				up = -1;
			}

			GetLog() << "Give: " << my_rank << " -- " << ddm->global_id[i] << " --> " << my_rank + up  << "\n";

			my_sys->RemoveBodyExchange(i);
		}

		// If the body is no longer involved with either neighbor, remove it from the other rank
		// and take ownership on this rank
		else if (location == distributed::OWNED)
		{
			if (curr_status == distributed::SHARED_UP)
			{
				num_sendup += PackUpdateTake(sendup_buf + num_sendup, i);
			}
			else if (curr_status == distributed::SHARED_DOWN)
			{
				num_senddown += PackUpdateTake(senddown_buf + num_senddown, i);
			}
			ddm->comm_status[i] = distributed::OWNED;
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

	MPI_Request up_rq;
	// send up
	if (my_rank != num_ranks - 1)
	{
		MPI_Isend(sendup_buf, num_sendup, MPI_DOUBLE, my_rank + 1, 1, my_sys->world, &up_rq);
		//MPI_Send(sendup_buf, num_sendup, MPI_DOUBLE, my_rank + 1, 1, my_sys->world);
//		MPI_Isend(sendup_buf, num_sendup, MPI_UNSIGNED_CHAR, my_rank + 1, 1, my_sys->world, &up_rq);
	}

	// Send empty message if there is nothing to send
	if (num_senddown == 0)
	{
		senddown_buf[0] = 0;
//		int zero = 0;
//		std::memcpy(senddown_buf, &zero, sizeof(int));
		num_senddown = 1;
	}

	MPI_Request down_rq;
	// send down
	if (my_rank != 0)
	{
		MPI_Isend(senddown_buf, num_senddown, MPI_DOUBLE, my_rank - 1, 2, my_sys->world, &down_rq);
		//MPI_Send(senddown_buf, num_senddown, MPI_DOUBLE, my_rank - 1, 2, my_sys->world);
//		MPI_Isend(senddown_buf, num_senddown, MPI_UNSIGNED_CHAR, my_rank - 1, 2, my_sys->world, &down_rq);
	}
	ddm->data_manager->system_timer.stop("Send");


	ddm->data_manager->system_timer.start("Recv");
	// RECEIVING
	int num_recvdown;
	int num_recvup;

	// probe for message from down
	double *recvdown_buf = NULL;
//	unsigned char *recvdown_buff = NULL;

	if (my_sys->GetMyRank() != 0)
	{
		MPI_Probe(my_rank - 1, 1, my_sys->world, &statusdown);
		MPI_Get_count(&statusdown, MPI_DOUBLE, &num_recvdown);
//		MPI_Get_count(&statusdown, MPI_UNSIGNED_CHAR, &num_recvdown);
		recvdown_buf = new double[num_recvdown];
//		recvdown_buff = new unsigned char[num_recvdown];
		MPI_Recv(recvdown_buf, num_recvdown, MPI_DOUBLE, my_rank - 1, 1, my_sys->world, &statusdown);
//		MPI_Recv(recvdown_buf, num_recvdown, MPI_UNSIGNED_CHAR, my_rank - 1, 1, my_sys->world, &statusdown);
	}

	// Process message from down, then check for message from up.
	// (or thread to do both at the same time)
//	int first;
//	std::memcpy(&first, recvdown_buf, sizeof(int);
//	if (my_rank != num_ranks - 1 && first != 0)
	if (my_rank != 0 && recvdown_buf[0] != 0)
	{
		ProcessBuffer(num_recvdown, recvdown_buf, 0);
	}

	delete recvdown_buf;
//	delete recvdown_buf;
	recvdown_buf = NULL;
//	recvdown_buf = NULL;

	// probe for a message from up
	double *recvup_buf = NULL;
//	unsigned char *recvup_buf = NULL;
	if (my_rank != num_ranks - 1)
	{
		MPI_Probe(my_rank + 1, 2, my_sys->world, &statusup);
		MPI_Get_count(&statusup, MPI_DOUBLE, &num_recvup);
//		MPI_Get_count(&statusup, MPI_UNSIGNED_CHAR, &num_recvup);
		recvup_buf = new double[num_recvup];
//		recvup_buf = new unsigned char[num_recvup];
		MPI_Recv(recvup_buf, num_recvup, MPI_DOUBLE, my_rank + 1, 2, my_sys->world, &statusup);
//		MPI_Recv(recvup_buf, num_recvup, MPI_UNSIGNED_CHAR, my_rank + 1, 2, my_sys->world, &statusup);
	}

	// Process message from up.
//	int first;
//	std::memcpy(&first, recvup_buff, sizeof(int);
//	if (my_rank != num_ranks - 1 && first != 0)
	if (my_rank != num_ranks - 1 && recvup_buf[0] != 0)
	{
		ProcessBuffer(num_recvup, recvup_buf, 1);
	}

	delete recvup_buf;

	ddm->data_manager->system_timer.stop("Recv");
}


void ChCommDistributed::CheckExchange()
{
	int *checkup_buf = new int[10000]; // TODO buffer sizes based on num_shared and num_ghost
	int iup = 0;
	int *checkdown_buf = new int[10000];
	int idown = 0;

	//my_sys->PrintBodyStatus();
	for (int i = 0; i < data_manager->num_rigid_bodies; i++)
	{
		switch(ddm->comm_status[i])
		{
		case distributed::SHARED_DOWN:
			checkdown_buf[idown++] = distributed::SHARED_DOWN;
			checkdown_buf[idown++] = (int) ddm->global_id[i];
			break;
		case distributed::SHARED_UP:
			checkup_buf[iup++] = distributed::SHARED_UP;
			checkup_buf[iup++] = (int) ddm->global_id[i];
			break;
		case distributed::GHOST_DOWN:
			checkdown_buf[idown++] = distributed::GHOST_DOWN;
			checkdown_buf[idown++] = (int) ddm->global_id[i];
			break;
		case distributed::GHOST_UP:
			checkup_buf[iup++] = distributed::GHOST_UP;
			checkup_buf[iup++] = (int) ddm->global_id[i];
			break;
		default:
			break;
		}
	}

	int my_rank = my_sys->GetMyRank();
	int num_ranks = my_sys->GetNumRanks();
	if (iup == 0)
	{
		checkup_buf[0] = distributed::EMPTY;
		iup = 1;
	}
	if (idown == 0)
	{
		checkdown_buf[0] = distributed::EMPTY;
		idown = 1;
	}

	MPI_Request up_rq;
	if (my_rank != num_ranks - 1)
	{
		MPI_Isend(checkup_buf, iup, MPI_INT, my_rank + 1, 1, my_sys->world, &up_rq);
		//MPI_Send(checkup_buf, iup, MPI_INT, my_rank + 1, 1, my_sys->world);
	}
	MPI_Request down_rq;
	if (my_rank != 0)
	{
		MPI_Isend(checkdown_buf, idown, MPI_INT, my_rank - 1, 2, my_sys->world, &down_rq);
		//MPI_Send(checkdown_buf, idown, MPI_INT, my_rank - 1, 2, my_sys->world);
	}

	delete checkup_buf;
	delete checkdown_buf;

	MPI_Status statusdown;
	MPI_Status statusup;
	int num_recvdown = 1;
	int num_recvup = 1;
	int *recvdown_buf = NULL;
	int *recvup_buf = NULL;

	if (my_rank != 0)
	{
		MPI_Probe(my_rank - 1, 1, my_sys->world, &statusdown);
		MPI_Get_count(&statusdown, MPI_INT, &num_recvdown);
		recvdown_buf = new int[num_recvdown];
		MPI_Recv(recvdown_buf, num_recvdown, MPI_INT, my_rank - 1, 1, my_sys->world, &statusdown);
	}

	if (num_recvdown != 1)
	{
		for (int i = 0; i < num_recvdown; i += 2)
		{
			int local_id = ddm->GetLocalIndex((unsigned int) recvdown_buf[i+1]);
			int status_to_match = (recvdown_buf[i] == distributed::SHARED_UP) ? distributed::GHOST_DOWN : distributed::SHARED_DOWN;
			if (local_id == -1)
			{
				GetLog() << "ERROR: GID " << (unsigned int) recvdown_buf[i+1] << " not on rank " << my_rank << " and on rank "
										<< my_rank - 1 << " as " << recvdown_buf[i] << "\n";
			}
			else if (ddm->comm_status[local_id] == status_to_match)
			{
				continue;
			}
			else
			{
				// TODO Handle Mismatch
				GetLog() << "MISMATCH: GID " << (unsigned int) recvdown_buf[i+1] << "seen on "
						<< my_rank << " as " << (int) ddm->comm_status[local_id] << " and on rank "
						<< my_rank - 1 << " as " << recvdown_buf[i] << "\n";
				my_sys->PrintBodyStatus();
			}
		}
	}

	delete recvdown_buf;

	if (my_rank != num_ranks - 1)
	{
		MPI_Probe(my_rank + 1, 2, my_sys->world, &statusup);
		MPI_Get_count(&statusup, MPI_INT, &num_recvup);
		recvup_buf = new int[num_recvup];
		MPI_Recv(recvup_buf, num_recvup, MPI_INT, my_rank + 1, 2, my_sys->world, &statusup);
	}

	if (my_rank != num_ranks - 1 && num_recvup != 1)
	{
		for (int i = 0; i < num_recvup; i += 2)
		{
			int local_id = ddm->GetLocalIndex((unsigned int) recvup_buf[i+1]);
			int status_match = (recvup_buf[i] == distributed::SHARED_DOWN) ? distributed::GHOST_UP : distributed::SHARED_UP;
			if (local_id == -1)
			{
				GetLog() << "ERROR: GID: " << (unsigned int) recvup_buf[i+1] << " not on rank " << my_rank << " and on rank "
							<< my_rank + 1 << " as " << recvup_buf[i] << "\n";
			}
			else if (ddm->comm_status[local_id] == status_match)
			{
				continue;
			}
			else
			{
				// TODO handle mismatch
				GetLog() << "MISMATCH: GID " << (unsigned int) recvup_buf[i+1] << "seen on "
						<< my_rank << " as " << (int) ddm->comm_status[local_id] << " and on rank "
						<< my_rank + 1 << " as " << recvup_buf[i] << "\n";
				my_sys->PrintBodyStatus();
			}
		}
	}

	delete recvup_buf;
	MPI_Barrier(my_sys->world);
}



// Called when a sphere leaves this rank's subdomain into the ghost layer.
// The body should have been removed from the local system's
// list after this function is called.
// Packages the body into buf.
// Returns the number of elements which the body took in the buffer
int ChCommDistributed::PackExchange(double *buf, int index)
{
//	unsigned char *buf;

	int m = 1; // Number of doubles being packed (will be placed in the front of the buffer
			   // once packing is done)

//	int n = sizeof(int);

	buf[m++] = (double) distributed::EXCHANGE;

//	int message_type = distributed::EXCHANGE;
//	std::memcpy(buf+n, &message_type, sizeof(int));
//	n += sizeof(int);

	// Global Id
	buf[m++] = (double) ddm->global_id[index];

//	std::memcpy(buf+n, &ddm->global_id[index], sizeof(uint));
//	n += sizeof(uint);


	// Position and rotation
	real3 pos = data_manager->host_data.pos_rigid[index];
	buf[m++] = pos.x;
	buf[m++] = pos.y;
	buf[m++] = pos.z;

//	std::memcpy(buf+n, pos.array, 3*sizeof(real));
//	n += 3*sizeof(real);


	// Rotation
	quaternion rot = data_manager->host_data.rot_rigid[index];
	buf[m++] = rot.x;
	buf[m++] = rot.y;
	buf[m++] = rot.z;
	buf[m++] = rot.w;

//	std::memcpy(buf+n, rot.array, 4*sizeof(real));
//	n += 4*sizeof(real);


	// Velocity
	buf[m++] = data_manager->host_data.v[index*6];
	buf[m++] = data_manager->host_data.v[index*6 + 1];
	buf[m++] = data_manager->host_data.v[index*6 + 2];


	// Angular Velocity
	buf[m++] = data_manager->host_data.v[index*6 + 3];
	buf[m++] = data_manager->host_data.v[index*6 + 4];
	buf[m++] = data_manager->host_data.v[index*6 + 5];

//	for (int i = 0; i < 6; i++)
//	{
//		std::memcpy(buf+n, &data_manager->host_data.v[index*6 + i], sizeof(real));
//		n += sizeof(real);
//	}

	// Mass
	buf[m++] = data_manager->host_data.mass_rigid[index];

//	std::memcpy(buf+n, &data_manager->host_data.mass_rigid[index], sizeof(real));
//	n += sizeof(real);


	//Inertia
	ChMatrix33<double> inertia = my_sys->bodylist[index]->GetInertia();
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			buf[m++] = inertia.GetElement(i,j);
		}
	}

//	for (int i = 0; i < 3; i++)
//	{
//		for (int j = 0; j < 3; j++)
//		{
//			std::memcpy(buf+n, &(inertia.GetElement(i,j)), sizeof(real));
//			n += sizeof(real);
//		}
//	}


	// Material SMC
	buf[m++] = data_manager->host_data.mu[index]; // Static Friction

//	std::memcpy(buf+n, &(data_manager->host_data.mu[index]), sizeof(real));
//	n += sizeof(real);


	buf[m++] = data_manager->host_data.adhesionMultDMT_data[index]; // Adhesion
	//					int message_type;

//	std::memcpy(buf+n, &(data_manager->host_data.adhesionMultDMT_data[index]), sizeof(real));
//	n += sizeof(real);


    if (data_manager->settings.solver.use_material_properties)
    {
    	buf[m++] = data_manager->host_data.elastic_moduli[index].x; // Young's Modulus

//    	std::memcpy(buf+n, &(data_manager->host_data.elastic_moduli[index].x), sizeof(real));
//    	n += sizeof(real);


    	buf[m++] = data_manager->host_data.elastic_moduli[index].y; // Poisson Ratio

//    	std::memcpy(buf+n, &(data_manager->host_data.elastic_moduli[index].y), sizeof(real));
//    	n += sizeof(real);


    	buf[m++] = data_manager->host_data.cr[index]; // Coefficient of Restitution

//    	std::memcpy(buf+n, &(data_manager->host_data.cr[index]), sizeof(real));
//    	n += sizeof(real);
    }
    else
    {
    	buf[m++] = data_manager->host_data.dem_coeffs[index].x; // kn
    	buf[m++] = data_manager->host_data.dem_coeffs[index].y; // kt
    	buf[m++] = data_manager->host_data.dem_coeffs[index].z; // gn
    	buf[m++] = data_manager->host_data.dem_coeffs[index].w; // gt

//    	std::memcpy(buf+n, data_manager->host_data.dem_coeffs[index].array, 4*sizeof(real));
//    	n += 4*sizeof(real);
    }


    // Collision
    buf[m++] = (double) data_manager->host_data.collide_rigid[index];

//    std::memcpy(buf+n, &(data_manager->host_data.collide_rigid[index]), sizeof(char));
//    n += sizeof(char);

    int shape_count = ddm->body_shape_count[index];
    buf[m++] = (double) shape_count;

//    std::memcpy(buf+n, &shape_count, sizeof(int));
//    n += sizeof(int);


    for (int i = 0; i < shape_count; i++)
    {
    	int shape_index = ddm->body_shapes[ddm->body_shape_start[index] + i];
    	int type = data_manager->shape_data.typ_rigid[shape_index];
    	int start = data_manager->shape_data.start_rigid[shape_index];
    	buf[m++] = (double) type;

//    	std::memcpy(buf+n, &type, sizeof(int));
//    	n += sizeof(int);


		buf[m++] = data_manager->shape_data.ObA_rigid[shape_index].x;
		buf[m++] = data_manager->shape_data.ObA_rigid[shape_index].y;
		buf[m++] = data_manager->shape_data.ObA_rigid[shape_index].z;

//		std::memcpy(buf+n, data_manager->shape_data.ObA_rigid[shape_index].array, 3*sizeof(real));
//		n += 3*sizeof(real);


		buf[m++] = data_manager->shape_data.ObR_rigid[shape_index].x; // TOOD the order of array is wxyz
		buf[m++] = data_manager->shape_data.ObR_rigid[shape_index].y;
		buf[m++] = data_manager->shape_data.ObR_rigid[shape_index].z;
		buf[m++] = data_manager->shape_data.ObR_rigid[shape_index].w;

//		std::memcpy(buf+n, data_manager->shape_data.ObR_rigid[shape_index].array, 4*sizeof(real));
//		n += 4*sizeof(real);


		//buf[m++] = data_manager->shape_data.fam_rigid[shape_index]; short2??

    	switch(type)
    	{
    	case chrono::collision::SPHERE:
    		buf[m++] = data_manager->shape_data.sphere_rigid[start];

//    		std::memcpy(buf+n, &(data_manager->shape_data.sphere_rigid[start]), sizeof(real));
//    		n += sizeof(real);
    		break;
    	case chrono::collision::BOX:
    		buf[m++] = data_manager->shape_data.box_like_rigid[start].x;
    		buf[m++] = data_manager->shape_data.box_like_rigid[start].y;
    		buf[m++] = data_manager->shape_data.box_like_rigid[start].z;

//    		std::memcpy(buf+n, data_manager->shape_data.box_like_rigid[start].array, 3*sizeof(real));
//    		n += 3*sizeof(real);
    		break;
    	default:
    		GetLog() << "Invalid shape for transfer\n";
    	}
    }

	buf[0] = (double) m;

//	std::memcpy(buf, &n, sizeof(int));

	//return n;
	return m;
}


// Unpacks a sphere body from the buffer into body.
// Note: body is meant to be a ptr into the data structure
// where the body should be unpacked.
// The body must be in the bodylist already so that GetId is valid
int ChCommDistributed::UnpackExchange(double *buf, std::shared_ptr<ChBody> body)
{
//	unsigned char *buf;

	int m = 2; // Skip the size value and the type value

//	int n = 2*sizeof(int);

	// Global Id
	uint gid = (unsigned int) buf[m++];
	body->SetGid(gid);

//	uint gid;
//	std::memcpy(&gid, buf+n, sizeof(uint));
//	body->SetGid(gid);

	int index = body->GetId();
	ddm->global_id[index] = gid;

	// Position and rotation
	body->SetPos(ChVector<double>(buf[m],buf[m+1],buf[m+2]));
	m += 3;

//	double pos[3];
//	std::memcpy(pos, buf+n, 3*sizeof(double));
//	n += 3*sizeof(double);
//	body->SetPos(pos[0], pos[1], pos[2]);

	// Rotation
	body->SetRot(ChQuaternion<double>(buf[m],buf[m+1],buf[m+2],buf[m+3]));
	m += 4;

//	ChQuaternion<double> rot;
//	std::memcpy(rot.data, buf+n, 4*sizeof(double));
//	n += 4*sizeof(double);
//	body->SetRot(rot);

	// Linear and Angular Velocities
	body->SetPos_dt(ChVector<double>(buf[m],buf[m+1],buf[m+2]));
	m += 3;

//	ChVector<double> vel;
//	std::memcpy(vel.data, buf+n, 3*sizeof(double));
//	n += 3*sizeof(double);
//	body->SetPos_dt(vel);

	body->SetWvel_par(ChVector<double>(buf[m],buf[m+1],buf[m+2]));
	m += 3;

//	ChVector<double> omega;
//	std::memcpy(omega.data, buf+n, 3*sizeof(double));
//	n += 3*sizeof(double);
//	body->SetWvel_par(omega);

	// Mass
	body->SetMass(buf[m++]);

//	double mass;
//	std::memcpy(&mass, buf+n, sizeof(double));
//	n += sizeof(double);
//	body->SetMass(mass);

	//Inertia
	ChMatrix33<double> inertia;
	for (int i = 0; i < 3; i++) // TODO matrix is symmetric
	{
		for (int j = 0; j < 3; j++)
		{
			inertia.SetElement(i, j, buf[m++]);
		}
	}
	body->SetInertia(inertia);

	// Material SMC
	std::shared_ptr<ChMaterialSurfaceSMC> mat = std::make_shared<ChMaterialSurfaceSMC>();

	mat->SetFriction((float) buf[m++]);  //Static Friction
	mat->adhesionMultDMT = (float) buf[m++];

    if (data_manager->settings.solver.use_material_properties)
    {
    	mat->young_modulus = (float) buf[m++];
    	mat->poisson_ratio = (float) buf[m++];
    	mat->restitution = (float) buf[m++];
    }
    else
    {
    	mat->kn = (float) buf[m++];
    	mat->kt = (float) buf[m++];
    	mat->gn = (float) buf[m++];
    	mat->gt = (float) buf[m++];
    }

	body->SetMaterialSurface(mat);

	bool collide = (bool) buf[m++];

    // Collision
	int shape_count = (int) buf[m++];

	body->GetCollisionModel()->ClearModel();
	for (int i = 0; i < shape_count; i++)
	{
		int type = (int) buf[m++];
		ChVector<double> A(buf[m],buf[m+1],buf[m+2]);
		m += 3;

		ChQuaternion<double> R(buf[m],buf[m+1],buf[m+2],buf[m+3]);
		m += 4;

		switch(type)
		{
		case chrono::collision::SPHERE:
			GetLog() << "Unpacking Sphere\n";
			body->GetCollisionModel()->AddSphere(buf[m++], A); //TODO does anything with pos?
			break;
		case chrono::collision::BOX:
			GetLog() << "Unpacking Box\n";
			body->GetCollisionModel()->AddBox(buf[m++],buf[m++],buf[m++], A); // TODO does anything with pos ?? rotation as quaternion vs matrix??
			break;
		default:
			GetLog() << "Unpacking undefined collision shape\n";
		}
	}

	body->GetCollisionModel()->BuildModel(); // Doesn't call collisionsystem::add since system isn't set yet.
	body->SetCollide(collide);
	return m;
}


// Only packs the essentials for a body update
int ChCommDistributed::PackUpdate(double *buf, int index, int update_type)
{
//	unsigned char *buf;

	int m = 1;
//	int n = sizeof(int);

	buf[m++] = (double) update_type;

//	std::memcpy(buf+n, &update_type, sizeof(int));

	// Global Id
	buf[m++] = (double) ddm->global_id[index];

//	std::memcpy(buf+n, &(ddm->global_id[index]), sizeof(uint));
//	n += sizeof(uint);

	// Position
	buf[m++] = data_manager->host_data.pos_rigid[index].x;
	buf[m++] = data_manager->host_data.pos_rigid[index].y;
	buf[m++] = data_manager->host_data.pos_rigid[index].z;

//	memcpy(buf+n, data_manager->host_data.pos_rigid[index].array, 3*sizeof(real));
//	n += 3*sizeof(real);


	// Rotation
	buf[m++] = data_manager->host_data.rot_rigid[index].x;
	buf[m++] = data_manager->host_data.rot_rigid[index].y;
	buf[m++] = data_manager->host_data.rot_rigid[index].z;
	buf[m++] = data_manager->host_data.rot_rigid[index].w;

//	std::memcpy(buf+n, data_manager->host_data.rot_rigid[index].array, 4*sizeof(real));
//	n += 4*sizeof(real);

	// Velocity
	buf[m++] = data_manager->host_data.v[index*6];
	buf[m++] = data_manager->host_data.v[index*6 + 1];
	buf[m++] = data_manager->host_data.v[index*6 + 2];

	// Angular Velocity
	buf[m++] = 	data_manager->host_data.v[index*6 + 3];
	buf[m++] = 	data_manager->host_data.v[index*6 + 4];
	buf[m++] = 	data_manager->host_data.v[index*6 + 5];

//	for (int i = 0; i < 6; i++)
//	{
//		std::memcpy(buf+n, &data_manager->host_data.v[index*6 + i], sizeof(real));
//		n += sizeof(real);
//	}


	buf[0] = (double) m;

//	std::memcpy(buf, &n, sizeof(int));

	//return n;
	return m;
}

int ChCommDistributed::UnpackUpdate(double *buf, std::shared_ptr<ChBody> body)
{
//	unsigned char *buf;

	int m = 2; // Skip size value and type value

//	int n = 2*sizeof(int);

	// Global Id //TODO not needed??
	body->SetGid((unsigned int) buf[m++]);

//	uint gid;
//	std::memcpy(&gid, buf+n, sizeof(uint));
//	body->SetGid(gid);
//	n += sizeof(uint);

	// Position
	body->SetPos(ChVector<double>(buf[m],buf[m+1],buf[m+2]));
	m += 3;

//	double data[4];
//	std::memcpy(data, buf+n, 3*sizeof(real));
//	ChVector<real> pos;
//	pos.Set(data[0], data[1], data[2]);
//	n += 3*sizeof(real);
//	body->SetPos(pos);


	// Rotation
	body->SetRot(ChQuaternion<double>(buf[m],buf[m+1],buf[m+2],buf[m+3]));
	m += 4;

//	std::memcpy(data, buf+n, 4*sizeof(double));
//	ChQuaternion<double> rot;
//	rot.Set(data[0], data[1], data[2], data[3]);
//	n += 4*sizeof(double);
//	body->SetRot(rot);

	// Linear Velocity
	body->SetPos_dt(ChVector<double>(buf[m],buf[m+1],buf[m+2]));
	m += 3;

//	std::memcpy(data, buf+n, 3*sizeof(double));
//	ChVector<double> vel;
//	vel.Set(data[0],data[1],data[2]);
//	body->SetPos_dt(vel);
//	n += 3*sizeof(double);


	// Angular Velocity
	body->SetWvel_par(ChVector<double>(buf[m],buf[m+1],buf[m+2]));
	m += 3;

//	std::memcpy(data, buf+n, 3*sizeof(double));
//	ChVector<double> omega;
//	omega.Set(data[0], data[1], data[2]);
//	body->SetWvel_par(omega);
//	n += 3*sizeof(double);

//	return n;
	return m;
}

int ChCommDistributed::PackUpdateTake(double *buf, int index)
{
//	unsigned char *buf;
//	int n = sizeof(int);

	buf[0] = (double) 3;
	buf[1] = (double) distributed::FINAL_UPDATE_TAKE;

//	int update_type = distributed::FINAL_UPDATE_TAKE;
//	std::memcpy(buf+n, &update_type, sizeof(int));
//	n += sizeof(int);

	buf[2] = (double) ddm->global_id[index];

//	std::memcpy(buf+n, &(ddm->global_id[index]), sizeof(uint));
//	n += sizeof(uint);

//	std::memcpy(buf, &n, sizeof(int));

//	return n;
	return 3;
}
