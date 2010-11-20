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
#include "physics/ChContactContainer.h"
#include "ChLcpSystemDescriptorMPI.h"
#include <algorithm>
#include <typeinfo>

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
	//GetLog() << "creating system";
}

ChSystemMPI::~ChSystemMPI()
{
}

void ChSystemMPI::CustomEndOfStep()
{
	InterDomainSyncronizeStates();
	InterDomainSyncronizeFlags();
	InterDomainSetup();
}



void ChSystemMPI::LCPprepare_inject(ChLcpSystemDescriptor& mdescriptor)
{

	mdescriptor.BeginInsertion(); // This resets the vectors of constr. and var. pointers.

	HIER_LINK_INIT
	while HIER_LINK_NOSTOP
	{
		Lpointer->InjectConstraints(mdescriptor);
		HIER_LINK_NEXT
	}
	HIER_BODY_INIT
	while HIER_BODY_NOSTOP
	{
		Bpointer->InjectVariables(mdescriptor);
		HIER_BODY_NEXT
	}
	HIER_OTHERPHYSICS_INIT
	while HIER_OTHERPHYSICS_NOSTOP
	{
		PHpointer->InjectVariables(mdescriptor);
		PHpointer->InjectConstraints(mdescriptor);
		HIER_OTHERPHYSICS_NEXT
	}


	// Now also fills the 'shared interfaces' for MPI solver
	if (ChLcpSystemDescriptorMPI* mpi_descriptor = dynamic_cast<ChLcpSystemDescriptorMPI*> (this->LCP_descriptor) )
	{
		for (int ni = 0; ni < this->nodeMPI.interfaces.size(); ni++)
		{
			if (nodeMPI.interfaces[ni].id_MPI != -1)
			 if (nodeMPI.interfaces[ni].shared_items.size())
			{
				ChLcpSharedInterfaceMPI* lcp_interface = &mpi_descriptor->GetSharedInterfacesList()[ni];

				// The interface of the LCP system descriptor will have the same MPI target as nodeMPI neighbours:
				lcp_interface->SetMPIfriend( this->nodeMPI.interfaces[ni].id_MPI );
				
				// Instance a temporary system descriptor, it will be used to manage variables of the interface.
				ChLcpSystemDescriptor auxdescr;
				auxdescr.BeginInsertion();

				// Scan all the shared ChPhysicsItem objects in the interfaces of nodeMPI
				// and let them inject their ChLcpVariable pointers into a temporary ChLcpSystemDescriptor:
				ChHashTable<int,ChInterfaceItem>::iterator hiterator = this->nodeMPI.interfaces[ni].shared_items.begin();
				while (hiterator != this->nodeMPI.interfaces[ni].shared_items.end())
				{
					if ((*hiterator).second.type != ChInterfaceItem::INTERF_SLAVESLAVE)
					{
						auxdescr.GetVariablesList().clear();
						(*hiterator).second.item->InjectVariables(auxdescr);
						// Use for() because single ChPhysicsItem might have injected _multiple_ ChLcpVariable objects..
						for (int iva = 0; iva < auxdescr.GetVariablesList().size(); iva++)
						{	
							ChLcpSharedVarMPI msharedvar;
							msharedvar.uniqueID = (*hiterator).second.item->GetIdentifier();
							msharedvar.var = auxdescr.GetVariablesList()[iva];
							if ((*hiterator).second.type == ChInterfaceItem::INTERF_MASTER)
								msharedvar.master = true;
							else 
								msharedvar.master = false;  // case of ChInterfaceItem::INTERF_SLAVE
							lcp_interface->InsertSharedVariable(msharedvar);		
						}
					}
					++hiterator;
				}
				auxdescr.EndInsertion(); // might be not needed? 
			}
		}
	}


	this->contact_container->InjectConstraints(mdescriptor);

	mdescriptor.EndInsertion(); 

}






///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////


void ChSystemMPI::InterDomainSyncronizeStates()
{
	HIER_BODY_INIT
	while HIER_BODY_NOSTOP
	{
		Bpointer->SyncCollisionModels();
		HIER_BODY_NEXT
	}
	HIER_OTHERPHYSICS_INIT
	while HIER_OTHERPHYSICS_NOSTOP
	{
		PHpointer->SyncCollisionModels();
		HIER_OTHERPHYSICS_NEXT
	}

	// Prepare the sorted list of shared items (bacause hash maps aren't necessarily
	// sorted even if they have the same number of elements)
	for (unsigned int ni = 0; ni < this->nodeMPI.interfaces.size(); ni++)
	{
		if (this->nodeMPI.interfaces[ni].id_MPI != -1) 
		{
			this->nodeMPI.interfaces[ni].sorted_items.clear();

			ChHashTable<int,ChInterfaceItem>::iterator hiterator = this->nodeMPI.interfaces[ni].shared_items.begin();
			while (hiterator != this->nodeMPI.interfaces[ni].shared_items.end())
			{
				this->nodeMPI.interfaces[ni].sorted_items.push_back(
						std::pair<int, ChInterfaceItem*> (hiterator->first, &hiterator->second)
					);

				++hiterator;
			}
		
			// Perform the sort
			std::sort(this->nodeMPI.interfaces[ni].sorted_items.begin(), 
					  this->nodeMPI.interfaces[ni].sorted_items.end()    );
		}
	}


	unsigned int num_interfaces = this->nodeMPI.interfaces.size();

	////// STEP 1
	// 
	// Synchronize positions to old master because on different domains the 
	// states might be slightly different after timestep integration (numerical roundoff, etc.)

	// Reset buffers to be used for MPI communication
	for (unsigned int ni=0; ni < num_interfaces; ni++)
	{
		this->nodeMPI.interfaces[ni].mstreami->clear();
		this->nodeMPI.interfaces[ni].mstreamo->clear();
		this->nodeMPI.interfaces[ni].mchstreami->Seek(0);
		this->nodeMPI.interfaces[ni].mchstreamo->Seek(0);
	}

	// 1 - Send states of items of interfaces,
	//     to synchronize master-slaves :

	for (unsigned int ni = 0; ni < num_interfaces; ni++)
	{
		if (this->nodeMPI.interfaces[ni].id_MPI != -1) 
		{
			//***TEST*** check interface matching correctness: a) size of shared objs.
	 		(*this->nodeMPI.interfaces[ni].mchstreamo) << (int)this->nodeMPI.interfaces[ni].shared_items.size();

			//ChHashTable<int,ChInterfaceItem>::iterator hiterator = this->nodeMPI.interfaces[ni].shared_items.begin();
			//while (hiterator != this->nodeMPI.interfaces[ni].shared_items.end())
			std::vector< std::pair<int,ChInterfaceItem*> >::iterator hiterator = this->nodeMPI.interfaces[ni].sorted_items.begin();
			while (hiterator != this->nodeMPI.interfaces[ni].sorted_items.end())
			{
				int item_key =  hiterator->second->item->GetIdentifier();
				ChPhysicsItem* item = hiterator->second->item;
				ChInterfaceItem::eChInterfaceItemType type = hiterator->second->type;
				
				//***TEST*** check interface matching correctness: b) allkeys
				(*this->nodeMPI.interfaces[ni].mchstreamo) << item_key;
			
				// Serialize the state from master to slave
				if (type == ChInterfaceItem::INTERF_MASTER)
					item->StreamOUTstate(*this->nodeMPI.interfaces[ni].mchstreamo);
				
				++hiterator;
			}
		}
	}

	// 2 - send buffers using MPI

	std::vector<ChMPIrequest> mrequestA(num_interfaces);

	for (unsigned int ni = 0; ni<num_interfaces; ni++)
	{
		if  ( this->nodeMPI.interfaces[ni].id_MPI != -1) // exclude unexisting domains
		{
			int err = ChMPI::SendBuffer( this->nodeMPI.interfaces[ni].id_MPI, 
							   *(this->nodeMPI.interfaces[ni].mstreamo), 
							   ChMPI::MPI_STANDARD, 
							   true,	// non blocking send, as  MPI_Isend
							   &(mrequestA[ni]));				   
		}
	}

	// 3 - receive buffers using MPI, 
	//     and synchronize slaves to master

	for (unsigned int ni = 0; ni<num_interfaces; ni++)
	{
		if ( this->nodeMPI.interfaces[ni].id_MPI != -1) // exclude unexisting domains
		{
			ChMPIstatus mstatus;
			
			int err = ChMPI::ReceiveBuffer(   this->nodeMPI.interfaces[ni].id_MPI, 
											*(this->nodeMPI.interfaces[ni].mstreami), 
											&mstatus 
										   );				
			
			if (this->nodeMPI.interfaces[ni].mstreami->size())
			{
				//***TEST*** check interface matching correctness
				int check_size;
				(*this->nodeMPI.interfaces[ni].mchstreami) >> check_size;
				if (check_size != this->nodeMPI.interfaces[ni].shared_items.size())
				{
					GetLog() << "ID=" << this->nodeMPI.id_MPI;
					GetLog() << " ERROR! InterDomainSyncronizeStates in interface to dom.=" 
							 << this->nodeMPI.interfaces[ni].id_MPI
							 << " DIFF.DIM!! n.keys=" <<  this->nodeMPI.interfaces[ni].shared_items.size()
							 << " but receive n.keys=" << check_size << "\n";
					break;
				}

				//ChHashTable<int,ChInterfaceItem>::iterator hiterator = this->nodeMPI.interfaces[ni].shared_items.begin();
				//while (hiterator != this->nodeMPI.interfaces[ni].shared_items.end())
				std::vector< std::pair<int,ChInterfaceItem*> >::iterator hiterator = this->nodeMPI.interfaces[ni].sorted_items.begin();
				while (hiterator != this->nodeMPI.interfaces[ni].sorted_items.end())
				{
					int item_key =  hiterator->first;
					ChPhysicsItem* item = hiterator->second->item;
					ChInterfaceItem::eChInterfaceItemType type = hiterator->second->type;

					//***TEST*** check interface matching correctness
					int check_key;
					(*this->nodeMPI.interfaces[ni].mchstreami) >> check_key;
					if (item_key != check_key)
					{
						GetLog() << "ID=" << this->nodeMPI.id_MPI;
						GetLog() << " ERROR! InterDomainSyncronizeStates in interface to dom.=" 
							 << this->nodeMPI.interfaces[ni].id_MPI
							 << " own key=" << item_key
							 << " but got key=" << check_key << "\n";
							break;
					}

					// Deserialize the state
					if (type == ChInterfaceItem::INTERF_SLAVE)
						item->StreamINstate(*this->nodeMPI.interfaces[ni].mchstreami);

					++hiterator;
				}

			}

		}
	}

	// wait that all messages are sent before proceeding
	for (unsigned int ni = 0; ni<num_interfaces; ni++)
	{
		if  ( this->nodeMPI.interfaces[ni].id_MPI != -1) // exclude unexisting domains
		{
			ChMPIstatus mstatus;
			ChMPI::Wait(&mrequestA[ni], &mstatus);
		}
	}
}





void ChSystemMPI::InterDomainSyncronizeFlags()
{
	unsigned int num_interfaces = this->nodeMPI.interfaces.size();

	//  After InterDomainSyncronizeStates() all objects have the same position of last known 'master'. 
	//  However now it might happen that the COG moved beyond some interface, so
	//  the master role must be passed to another copy of item, in other domain.
	//  Must reset types master/slave/slaveslave if some aabb center moved into 
	//  another domain, so the master role could be switched to another domain

	// Reset buffers to be used for MPI communication
	for (unsigned int ni=0; ni < num_interfaces; ni++)
	{
		this->nodeMPI.interfaces[ni].mstreami->clear();
		this->nodeMPI.interfaces[ni].mstreamo->clear();
		this->nodeMPI.interfaces[ni].mchstreami->Seek(0);
		this->nodeMPI.interfaces[ni].mchstreamo->Seek(0);
	}

	// 1 - Send flags of interfaces,
	//     to reset master-slave roles:

	for (unsigned int ni = 0; ni < num_interfaces; ni++)
	{
		if (this->nodeMPI.interfaces[ni].id_MPI != -1) 
		{
			//ChHashTable<int,ChInterfaceItem>::iterator hiterator = this->nodeMPI.interfaces[ni].shared_items.begin();
			//while (hiterator != this->nodeMPI.interfaces[ni].shared_items.end())
			std::vector< std::pair<int,ChInterfaceItem*> >::iterator hiterator = this->nodeMPI.interfaces[ni].sorted_items.begin();
			while (hiterator != this->nodeMPI.interfaces[ni].sorted_items.end())
			{
				int item_key =  hiterator->first;
				ChPhysicsItem* item = hiterator->second->item;

				// default fallback:
				hiterator->second->type = ChInterfaceItem::INTERF_SLAVESLAVE;
				
				// Send "is master" info
				int master = 0;
				ChVector<> mcenter;
				item->GetCenter(mcenter);
				if (this->nodeMPI.IsInto(mcenter))
				{
					master = 1;
					hiterator->second->type = ChInterfaceItem::INTERF_MASTER;
				}
				(*this->nodeMPI.interfaces[ni].mchstreamo) << master;
				
				++hiterator;
			}
		}
	}

	// 2 - send buffers using MPI

	std::vector<ChMPIrequest> mrequestB(num_interfaces);

	for (unsigned int ni = 0; ni<num_interfaces; ni++)
	{
		if  ( this->nodeMPI.interfaces[ni].id_MPI != -1) // exclude unexisting domains
		{
			int err = ChMPI::SendBuffer( this->nodeMPI.interfaces[ni].id_MPI, 
							   *(this->nodeMPI.interfaces[ni].mstreamo), 
							   ChMPI::MPI_STANDARD, 
							   true,	// non blocking send, as  MPI_Isend
							   &(mrequestB[ni]));				   
		}
	}

	// 3 - receive buffers using MPI, 
	//     and find 'slaves' when receiving '1' flags from masters

	for (unsigned int ni = 0; ni<num_interfaces; ni++)
	{
		if ( this->nodeMPI.interfaces[ni].id_MPI != -1) // exclude unexisting domains
		{
			ChMPIstatus mstatus;
			
			int err = ChMPI::ReceiveBuffer(   this->nodeMPI.interfaces[ni].id_MPI, 
											*(this->nodeMPI.interfaces[ni].mstreami), 
											&mstatus 
										   );				

			if (this->nodeMPI.interfaces[ni].mstreami->size())
			{
				//ChHashTable<int,ChInterfaceItem>::iterator hiterator = this->nodeMPI.interfaces[ni].shared_items.begin();
				//while (hiterator != this->nodeMPI.interfaces[ni].shared_items.end())
				std::vector< std::pair<int,ChInterfaceItem*> >::iterator hiterator = this->nodeMPI.interfaces[ni].sorted_items.begin();
				while (hiterator != this->nodeMPI.interfaces[ni].sorted_items.end())
				{
					int item_key =  hiterator->first;
					ChPhysicsItem* item = hiterator->second->item;

					// Deserialize the state
					int near_master;
					(*this->nodeMPI.interfaces[ni].mchstreami) >> near_master;

					if (near_master)
						hiterator->second->type = ChInterfaceItem::INTERF_SLAVE;

					++hiterator;
				}
			}
		}
	}

	// wait that all messages are sent before proceeding
	for (unsigned int ni = 0; ni<num_interfaces; ni++)
	{
		if  ( this->nodeMPI.interfaces[ni].id_MPI != -1) // exclude unexisting domains
		{
			ChMPIstatus mstatus;
			ChMPI::Wait(&mrequestB[ni], &mstatus);
		}
	}


}


///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////


void ChSystemMPI::InterDomainSetup()
{
	unsigned int num_interfaces = this->nodeMPI.interfaces.size();

	// Reset buffers to be used for MPI communication
	for (unsigned int ni=0; ni < num_interfaces; ni++)
	{
		this->nodeMPI.interfaces[ni].mstreami->clear();
		this->nodeMPI.interfaces[ni].mstreamo->clear();
		this->nodeMPI.interfaces[ni].mchstreami->Seek(0);
		this->nodeMPI.interfaces[ni].mchstreami->Init(); 
		this->nodeMPI.interfaces[ni].mchstreamo->Seek(0);
		this->nodeMPI.interfaces[ni].mchstreamo->Init();
	}


	// Purge interface shared hash tables from items that aren't 
	// overlapping any longer:

	for (unsigned int ni = 0; ni < num_interfaces; ni++)
	{
		if (this->nodeMPI.interfaces[ni].id_MPI != -1) 
		{
			ChHashTable<int,ChInterfaceItem>::iterator hiterator = this->nodeMPI.interfaces[ni].shared_items.begin();
			while (hiterator != this->nodeMPI.interfaces[ni].shared_items.end())
			{
				int current_key =  hiterator->first;
				ChPhysicsItem* item = hiterator->second.item;
				++hiterator;

				ChVector<> bbmin, bbmax;
				item->GetTotalAABB(bbmin, bbmax);
				bool erase = false;
				
				// first two checks are meant for early bailout, otherwise it could be only the third one 
				if (this->nodeMPI.IsAABBoutside(bbmin, bbmax))
					erase = true;
				else 
				 if (this->nodeMPI.IsAABBinside(bbmin, bbmax))
					erase = true;
				 else 
				  if (!this->nodeMPI.IsAABBoverlappingInterface(ni, bbmin, bbmax))
					erase = true;

				if (erase)
				{
					//GetLog() << "ID=" << this->nodeMPI.id_MPI << "    must purge key=" << current_key << "\n";
					this->nodeMPI.interfaces[ni].shared_items.erase(current_key);
				}
				
			}
		}
	}

	// 1 - serialize objects that 'spill out' to interface buffers
	//  Also: 
	// - items that goes out of domain will be deleted from ChSystem and erased from interface hash keys
	// - items that are overlapping with interfaces, that previouly were not:
	//     - will be serialized and sent to neighbouring domains, if master role (center is in domain)
	//     - will be added to interfaces hash tables 

	bool to_delete;
/*
	HIER_BODY_INIT
	while HIER_BODY_NOSTOP
	{
		//***TO DO*** as for HIER_OTHERPHYSICS
		HIER_BODY_NEXT
	}
*/
	//GetLog() << "ID=" << this->nodeMPI.id_MPI << "   step1 \n";

	HIER_OTHERPHYSICS_INIT
	while HIER_OTHERPHYSICS_NOSTOP
	{	
		to_delete = false;
		ChPhysicsItem* item = PHpointer;

		ChVector<> bbmin, bbmax, mcenter;
		item->GetTotalAABB(bbmin, bbmax);
		item->GetCenter(mcenter);

		if (this->nodeMPI.IsAABBoutside(bbmin, bbmax))
		{
			// delete (at the end of while loop) the item, because it
			// went off the domain
			to_delete = true;
		}
		else
		 if (!this->nodeMPI.IsAABBinside(bbmin, bbmax))
		{
			// Early fast checks failed: it was not completely outside, it was not 
			// completely inside, so this means that aabb is overlapping with some interface; 
			// hence now spend some time finding the specific overlap with n-th interfaces.

			for (unsigned int ni = 0; ni < num_interfaces; ni++)
			{
				if (this->nodeMPI.interfaces[ni].id_MPI != -1) // do not deal with inactive interfaces
				 if (this->nodeMPI.IsAABBoverlappingInterface(ni, bbmin, bbmax))
				  if (!this->nodeMPI.interfaces[ni].shared_items.present(item->GetIdentifier())) // not yet added: add to interface!
				   if (this->nodeMPI.IsInto(mcenter)) 
				   {
					    // Ok, it is overlapping to some interface to whom it was not added to, and 
						// this wil play the role of 'master'. Hence to deserialize and clone.
						// Must be sent to neighbour domain, so:
						//  1- Add to interface hash table, as 'master'
						this->nodeMPI.interfaces[ni].shared_items.insert(item->GetIdentifier(), 
							ChInterfaceItem(item,ChInterfaceItem::INTERF_MASTER));

						//  2- Serialize to persistent data to be sent via MPI.
						//     Only the 'owner' domain (the one with the center of aabb) 
						//     will send data, to avoid n-uple sends at the corners.
						try 
						{
							/*
							GetLog() << "ID=" << this->nodeMPI.id_MPI 
									 << " SERIALIZE: " << item->GetRTTI()->GetName() 
									 << "  key=" << item->GetIdentifier()
									 << "  to ID=" << this->nodeMPI.interfaces[ni].id_MPI << "\n";
							*/
							this->nodeMPI.interfaces[ni].mchstreamo->AbstractWrite(item);
						}
						catch (ChException myex)
						{
							GetLog() << "ERROR serializing MPI item:\n " << myex.what() << "\n";
						}
				   }
				   else
				   {
					    // Ok, it is overlapping to some interface to whom it was not added to,
					    // but it is not a 'master' because center is outside. So add it to interface 
					    // anyway, as slave-slave because previously not added.
						this->nodeMPI.interfaces[ni].shared_items.insert(item->GetIdentifier(), 
							ChInterfaceItem(item,ChInterfaceItem::INTERF_SLAVESLAVE));
				   }

			} // end interfaces loop
			
			
		}

		ChPhysicsItem* olditem = PHpointer;
		HIER_OTHERPHYSICS_NEXT
		
		// Maybe that the item must be removed from the ChSystem because
		// it went away, so do it now:
		if (to_delete)
		{
			//GetLog() << "ID=" << this->nodeMPI.id_MPI << "   must Remove obj with key="<< olditem->GetIdentifier() << "\n";
			ChSharedPtr<ChPhysicsItem> shpointer(olditem);
			olditem->AddRef(); // because wrapping normal (not new) ptr with shared pointer
			this->Remove(shpointer);
		}
		
	}

	// 2 - send buffers using MPI

	std::vector<ChMPIrequest> mrequest(num_interfaces);

	for (unsigned int ni = 0; ni<num_interfaces; ni++)
	{
		if  ( this->nodeMPI.interfaces[ni].id_MPI != -1) // exclude unexisting domains
		{
			int err = ChMPI::SendBuffer( this->nodeMPI.interfaces[ni].id_MPI, 
							   *(this->nodeMPI.interfaces[ni].mstreamo), 
							   ChMPI::MPI_STANDARD, 
							   true,	// non blocking send, as  MPI_Isend
							   &(mrequest[ni]));				   
		}
	}

	// 3 - receive buffers using MPI, deserialize and add to system
	for (unsigned int ni = 0; ni<num_interfaces; ni++)
	{
		if ( this->nodeMPI.interfaces[ni].id_MPI != -1) // exclude unexisting domains
		{
			ChMPIstatus mstatus;
			
			int err = ChMPI::ReceiveBuffer(   this->nodeMPI.interfaces[ni].id_MPI, 
											*(this->nodeMPI.interfaces[ni].mstreami), 
											&mstatus 
										   );				

			if (this->nodeMPI.interfaces[ni].mstreami->size())
			{
				while (! this->nodeMPI.interfaces[ni].mchstreami->End_of_stream())
				{
					// Must receive items from neighbour domain, so:

					// 1-deserialize received data, with class factory
					ChPhysicsItem* newitem = 0;
					try 
					{
						this->nodeMPI.interfaces[ni].mchstreami->AbstractReadCreate(&newitem);
						/*
						GetLog() << "ID=" << this->nodeMPI.id_MPI 
								 << " DESERIALIZED: " << newitem->GetRTTI()->GetName() 
								 << "  key=" << newitem->GetIdentifier()
								 << "  from ID=" << nodeMPI.interfaces[ni].id_MPI << "\n";
						*/
					}
					catch (ChException myex)
					{
						GetLog() << "ERROR deserializing MPI item:\n " << myex.what() << "\n";
					}
					ChSharedPtr<ChPhysicsItem> ptritem(newitem);

					// 2-add to system
					if (newitem)
						this->Add(ptritem);

					ChVector<> bbmin, bbmax;
					newitem->SyncCollisionModels();
					newitem->GetTotalAABB(bbmin, bbmax);

					// 3-add to other interfaces hash table, 
					for (unsigned int sui = 0; sui<num_interfaces; sui++)
						if  ( this->nodeMPI.interfaces[sui].id_MPI != -1) // exclude unexisting domains
						{
							// Set as 'slave' for the interface with master (avoid sending back in next steps).
							if (sui==ni)
								this->nodeMPI.interfaces[sui].shared_items.insert(newitem->GetIdentifier(), 
									ChInterfaceItem(newitem,ChInterfaceItem::INTERF_SLAVE) );
							else
							{
								if (this->nodeMPI.IsAABBoverlappingInterface(sui, bbmin, bbmax))
									this->nodeMPI.interfaces[sui].shared_items.insert(newitem->GetIdentifier(), 
									     ChInterfaceItem(newitem,ChInterfaceItem::INTERF_SLAVESLAVE) );
							}
						}

				}
			}

		}
	}

	// wait that all messages are sent before proceeding
	for (unsigned int ni = 0; ni<num_interfaces; ni++)
	{
		if  ( this->nodeMPI.interfaces[ni].id_MPI != -1) // exclude unexisting domains
		{
			ChMPIstatus mstatus;
			ChMPI::Wait(&mrequest[ni], &mstatus);
		}
	}


}





void ChSystemMPI::WriteOrderedDumpAABB(ChMPIfile& output)
{
	// save items contained in domain on file, if any, as simple xyz position
	std::string mstring = "";
	ChSystem::IteratorOtherPhysicsItems miterator = IterBeginOtherPhysicsItems();
	while (miterator != IterEndOtherPhysicsItems())
	{
		ChVector<> mmin, mmax;
		(*miterator)->GetTotalAABB(mmin, mmax);
		
		int mshared=-100;
		if (this->nodeMPI.IsAABBinside(mmin, mmax))	// for quick bailout
		{
			mshared = 0;
		}
		else
		{
			// Test if it was shared with some interface
			for (int i=0; i<this->nodeMPI.interfaces.size(); i++)
			{
				if (nodeMPI.interfaces[i].shared_items.present( (*miterator)->GetIdentifier() ) )
				{
					ChHashTable<int, ChInterfaceItem>::iterator mhashiter = nodeMPI.interfaces[i].shared_items.find( (*miterator)->GetIdentifier() );
					if ((*mhashiter).second.type == ChInterfaceItem::INTERF_MASTER)
					{
						mshared = 1;
						break;
					}
					if ((*mhashiter).second.type == ChInterfaceItem::INTERF_SLAVE)
					{
						mshared = 2;
						break;
					}
					mshared = 3; //  slaveslave case, should never happen. At least one interface should set as 1 or 2.
				}
			}
		}
		
		char buffer[100];
		sprintf(buffer, "%d, %d, %d, %g, %g, %g, %g, %g, %g ,\n", this->nodeMPI.id_MPI, (*miterator)->GetIdentifier(), mshared, mmin.x, mmin.y, mmin.z, mmax.x, mmax.y, mmax.z);
		mstring.append(buffer);
		++miterator;
	}

	output.WriteOrdered((char*)mstring.c_str(), strlen(mstring.c_str()));
}





void ChSystemMPI::WriteOrderedDumpDebugging(ChMPIfile& output)
{
	std::string mstring = "";

	char sbuffer[300];
	sprintf(sbuffer, "Node ID=%d\n", this->nodeMPI.id_MPI);
	mstring.append(sbuffer);

	// Test if it was shared with some interface
	for (int i=0; i<this->nodeMPI.interfaces.size(); i++)
	{
		if (nodeMPI.interfaces[i].id_MPI != -1)
		 if (nodeMPI.interfaces[i].shared_items.size())
		{
			sprintf(sbuffer, "    Interface to node ID=%d   has %d items\n", nodeMPI.interfaces[i].id_MPI, nodeMPI.interfaces[i].shared_items.size());
			mstring.append(sbuffer);

			ChHashTable<int,ChInterfaceItem>::iterator hiterator = this->nodeMPI.interfaces[i].shared_items.begin();
			while (hiterator != this->nodeMPI.interfaces[i].shared_items.end())
			{
				sprintf(sbuffer, "          Item ID=%d  -  type=", (*hiterator).first);
				mstring.append(sbuffer);
				if ((*hiterator).second.type == ChInterfaceItem::INTERF_MASTER)
					mstring.append("MASTER       *");
				if ((*hiterator).second.type == ChInterfaceItem::INTERF_SLAVE)
					mstring.append("Slave        .");
				if ((*hiterator).second.type == ChInterfaceItem::INTERF_SLAVESLAVE)
					mstring.append("slave/slave");
				mstring.append("\n");
				++hiterator;
			}
			
		}
	}
	mstring.append("\n");

	output.WriteOrdered((char*)mstring.c_str(), strlen(mstring.c_str()));
}






} // END_OF_NAMESPACE____


////// end
