///////////////////////////////////////////////////
//
//   ChSystemMPI.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "mpi.h"
#include "ChMpi.h"
#include "ChSystemMPI.h"
#include "ChBodyMPI.h"



// Shortcuts for hierarchy-handling functions

#define Bpointer		    (*ibody)
#define HIER_BODY_INIT      std::vector<ChBody*>::iterator ibody = bodylist.begin();
#define HIER_BODY_NOSTOP    (ibody != bodylist.end())
#define HIER_BODY_NEXT	    ibody++;

#define Lpointer		    (*iterlink)
#define HIER_LINK_INIT      std::list<ChLink*>::iterator iterlink = linklist.begin();
#define HIER_LINK_NOSTOP    (iterlink != linklist.end())
#define HIER_LINK_NEXT	    iterlink++;

#define PHpointer		    (*iterotherphysics)
#define HIER_OTHERPHYSICS_INIT    std::list<ChPhysicsItem*>::iterator iterotherphysics = otherphysicslist.begin();
#define HIER_OTHERPHYSICS_NOSTOP  (iterotherphysics != otherphysicslist.end())
#define HIER_OTHERPHYSICS_NEXT	  iterotherphysics++;




using namespace std;


namespace chrono
{

ChSystemMPI::ChSystemMPI()
{
	GetLog() << "creating system";
}

ChSystemMPI::~ChSystemMPI()
{
}

void ChSystemMPI::CustomEndOfStep()
{
	unsigned int num_interfaces = this->nodeMPI.interfaces.size();

	for (unsigned int i=0; i < num_interfaces; i++)
	{
		this->nodeMPI.interfaces[i].mstreami->clear();
		this->nodeMPI.interfaces[i].mstreamo->clear();
		this->nodeMPI.interfaces[i].mchstreami->Seek(0);
		this->nodeMPI.interfaces[i].mchstreamo->Seek(0);
	}


	// 1 - serialize objects that 'spill out' to interface buffers
	//  Also: 
	// - items that goes out of domain will be deleted from ChSystem and erased from interface hash keys
	// - items that are overlapping with interfaces will be serialized and sent to neighbouring domains

	bool to_delete;
/*
	HIER_BODY_INIT
	while HIER_BODY_NOSTOP
	{
		if (ChBodyMPI* mpibody = dynamic_cast<ChBodyMPI*>(Bpointer))
		{
			int oldflags = mpibody->GetOverlapFlags();
			GetLog() << "ID=" << this->nodeMPI.id_MPI << " CustomEndOfStep 3 oldflags = " << oldflags <<  "\n";
			int newflags = mpibody->ComputeOverlapFlags(this->nodeMPI);
			GetLog() << "ID=" << this->nodeMPI.id_MPI << " CustomEndOfStep 4 newflags = " << newflags <<  "\n";
			int changeflags = newflags & ~oldflags; // deal only with last overlapped domains 
			if (changeflags)
			{
				for (int bi = 0; bi<num_interfaces; bi++)
				{
					if ( (0x1 << bi) & changeflags )	
					{
						if (this->nodeMPI.interfaces[bi].id_MPI != -1) // exclude unexisting domains
						{
							// serialize to interface stream
							GetLog() << "ID=" << this->nodeMPI.id_MPI << " SEND: " << mpibody->GetRTTI()->GetName() << "  to ID=" << this->nodeMPI.interfaces[bi].id_MPI << "\n";
							this->nodeMPI.interfaces[bi].mchstreamo->AbstractWrite(mpibody);
							GetLog() << "ID=" << this->nodeMPI.id_MPI << " .SENT: " << mpibody->GetRTTI()->GetName() << "  to ID=" << this->nodeMPI.interfaces[bi].id_MPI << "\n";
						}
					}
				}
			}
		}
		HIER_BODY_NEXT
	}
*/

	HIER_OTHERPHYSICS_INIT
	while HIER_OTHERPHYSICS_NOSTOP
	{	
		to_delete = false;

		if (ChBodyMPI* mpibody = dynamic_cast<ChBodyMPI*>(PHpointer))
		{
			ChVector<> bbmin, bbmax;
			mpibody->GetCollisionModel()->GetAABB(bbmin, bbmax);

			if (this->nodeMPI.IsAABBoutside(bbmin, bbmax))
			{
				// delete (at the end of while loop) the item, because it
				// went off the domain
				to_delete = true;

				// erase body from shared hash tables, if it was previously shared 
				for (unsigned int n_interface = 0; n_interface < num_interfaces; n_interface++)
					this->nodeMPI.interfaces[n_interface].shared_items.erase(mpibody->GetIdentifier());
			}
			else
			 if (!this->nodeMPI.IsAABBinside(bbmin, bbmax))
			{
				// Early fast checks failed: it was not completely outside, it was not 
				// completely inside, so this means that aabb is overlapping with some interface; 
				// hence now spend some time finding the specific overlap with n-th interfaces.

				for (unsigned int n_interface = 0; n_interface < num_interfaces; n_interface++)
				{
					if (this->nodeMPI.interfaces[n_interface].id_MPI != -1) // do not deal with inactive interfaces
					 if (this->nodeMPI.IsAABBoverlappingInterface(n_interface, bbmin, bbmax))
					{
						// Ok, the object must be shared with the node at interface 'n_interface'.
						// Do not stream if already added - use hash table - 
						if (! this->nodeMPI.interfaces[n_interface].shared_items.present(mpibody->GetIdentifier()))
						{
							// Must be sent to neighbour domain, so:
							//  1- add to interface hash table
							this->nodeMPI.interfaces[n_interface].shared_items.insert(mpibody->GetIdentifier(), mpibody);

							//  2- serialize to persistent data to be sent via MPI
							try 
							{
								GetLog() << "ID=" << this->nodeMPI.id_MPI << " SERIALIZE: " << mpibody->GetRTTI()->GetName() << "  to ID=" << this->nodeMPI.interfaces[n_interface].id_MPI << "\n";
								this->nodeMPI.interfaces[n_interface].mchstreamo->AbstractWrite(mpibody);
							}
							catch (ChException myex)
							{
								GetLog() << "ERROR serializing MPI item:\n " << myex.what() << "\n";
							}
						}
					}
				} // end interfaces loop
			}
			/*
			int oldflags = mpibody->GetOverlapFlags();
			int newflags = mpibody->ComputeOverlapFlags(this->nodeMPI);

			int changeflags = newflags & ~oldflags; // deal only with last overlapped domains 
			if (changeflags)
			{
				for (int bi = 0; bi<num_interfaces; bi++)
				{
					if ( (0x1 << bi) & changeflags )	
					{
						if (this->nodeMPI.interfaces[bi].id_MPI != -1) // exclude unexisting domains
						{
							try 
							{
								// serialize to interface stream
								GetLog() << "ID=" << this->nodeMPI.id_MPI << " SERIALIZE: " << mpibody->GetRTTI()->GetName() << "  to ID=" << this->nodeMPI.interfaces[bi].id_MPI << "\n";
								this->nodeMPI.interfaces[bi].mchstreamo->AbstractWrite(mpibody);
							}
							catch (ChException myex)
							{
								GetLog() << "ERROR serializing MPI item:\n " << myex.what() << "\n";
							}


						}
					}
				}
			}
			*/
		}

		ChPhysicsItem* olditem = PHpointer;
		HIER_OTHERPHYSICS_NEXT
		
		// Maybe that the item must be removed from the ChSystem because
		// it went away, so do it now:
		if (to_delete)
		{
			ChSharedPtr<ChPhysicsItem> shpointer(olditem);
			olditem->AddRef(); // because wrapping normal (not new) ptr with shared pointer
			this->Remove(shpointer);
		}
		
	}

	// Purge interface shared hash tables from items that aren't 
	// overlapping any longer:

	for (unsigned int n_interface = 0; n_interface < num_interfaces; n_interface++)
	{
		if (this->nodeMPI.interfaces[n_interface].id_MPI != -1) 
		{
			ChHashTable<int,ChPhysicsItem*>::iterator hiterator = this->nodeMPI.interfaces[n_interface].shared_items.begin();
			while (hiterator != this->nodeMPI.interfaces[n_interface].shared_items.end())
			{
				int current_key = (*hiterator).first;
				ChPhysicsItem* mitem = hiterator->second;
				++hiterator;
				if (ChBodyMPI* mpibody = dynamic_cast<ChBodyMPI*>(PHpointer))
				{
					ChVector<> bbmin, bbmax;
					mpibody->GetCollisionModel()->GetAABB(bbmin, bbmax);
					bool erase = false;
					
					if (this->nodeMPI.IsAABBoutside(bbmin, bbmax))
						erase = true;
					else 
					 if (this->nodeMPI.IsAABBinside(bbmin, bbmax))
						erase = true;
					 else 
					  if (!this->nodeMPI.IsAABBoverlappingInterface(n_interface, bbmin, bbmax))
						erase = true;

					if (erase)
						this->nodeMPI.interfaces[n_interface].shared_items.erase(current_key);
				}
			}
		}
	}

	// 2 - send buffers using MPI

	std::vector<ChMPIrequest> mrequest(num_interfaces);

	for (unsigned int bi = 0; bi<num_interfaces; bi++)
	{
		if  ( this->nodeMPI.interfaces[bi].id_MPI != -1) // exclude unexisting domains
		{
			int err = ChMPI::SendBuffer( this->nodeMPI.interfaces[bi].id_MPI, 
							   *(this->nodeMPI.interfaces[bi].mstreamo), 
							   ChMPI::MPI_STANDARD, 
							   true,	// non blocking send, as  MPI_Isend
							   &(mrequest[bi]));				   
		}
	}

	// 3 - receive buffers using MPI, deserialize and add to system
	for (unsigned int bi = 0; bi<num_interfaces; bi++)
	{
		if ( this->nodeMPI.interfaces[bi].id_MPI != -1) // exclude unexisting domains
		{
			ChMPIstatus mstatus;
			
			int err = ChMPI::ReceiveBuffer(   this->nodeMPI.interfaces[bi].id_MPI, 
											*(this->nodeMPI.interfaces[bi].mstreami), 
											&mstatus 
										   );				
			
			if (this->nodeMPI.interfaces[bi].mstreami->size())
			{
				while (! this->nodeMPI.interfaces[bi].mchstreami->End_of_stream())
				{
					// Must receive items from neighbour domain, so:

					// 1-deserialize received data, with class factory
					ChPhysicsItem* newitem = 0;
					try 
					{
						this->nodeMPI.interfaces[bi].mchstreami->AbstractReadCreate(&newitem);
						GetLog() << "ID=" << this->nodeMPI.id_MPI << " DESERIALIZED: " << newitem->GetRTTI()->GetName() << " from neighbour=" << nodeMPI.interfaces[bi].id_MPI << "\n";
					}
					catch (ChException myex)
					{
						GetLog() << "ERROR deserializing MPI item:\n " << myex.what() << "\n";
					}
					ChSharedPtr<ChPhysicsItem> ptritem(newitem);

					// 2-add to system
					if (newitem)
						this->Add(ptritem);

					// 3-add to interface hash table, to avoid sending back in next steps
					this->nodeMPI.interfaces[bi].shared_items.insert(newitem->GetIdentifier(), newitem);

				}
			}

		}
	}

	// wait that all messages are sent before proceeding
	for (unsigned int bi = 0; bi<num_interfaces; bi++)
	{
		if  ( this->nodeMPI.interfaces[bi].id_MPI != -1) // exclude unexisting domains
		{
			ChMPIstatus mstatus;
			ChMPI::Wait(&mrequest[bi], &mstatus);
		}
	}
}



} // END_OF_NAMESPACE____


////// end
