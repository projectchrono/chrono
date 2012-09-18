///////////////////////////////////////////////////
//
//   ChSystemMPI.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChMpi.h"
#include "ChSystemMPI.h"
#include "ChBodyMPI.h"
#include "ChBodyDEMMPI.h"
#include "ChAssemblyMPI.h"
#include "physics/ChContactContainer.h"
#include "unit_MPI/ChContactContainerDEMMPI.h"
#include "collision/ChCCollisionSystemBullet.h"
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


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChSystemMPI> a_registration_ChSystemMPI;



ChSystemMPI::ChSystemMPI(unsigned int max_objects, double scene_size)
{
	ChSystem::ChSystem(max_objects, scene_size, false);
	collision_system = new ChCollisionSystemBullet(max_objects, scene_size);
	parallel_thread_number=0;
	//GetLog() << "creating system";
}

ChSystemMPI::~ChSystemMPI()
{
}

void ChSystemMPI::CustomEndOfStep()
{
	InterDomainSynchronizeStates();
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

	// Do not forget that there could be 'constraint' LCP objects
	// created by the collision detection:
	this->contact_container->InjectConstraints(mdescriptor);

	// The following will finally sort the list of interfacing variables in the LCP descriptor, 
	// according to their unique ID :

	mdescriptor.EndInsertion(); 

}






///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////


void ChSystemMPI::InterDomainRemoveOtherPhysicsItem (int id)
{
	//remove any references in shared items
	for (unsigned int ni = 0; ni < this->nodeMPI.interfaces.size(); ni++)
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
				
				if (current_key==id)
					erase = true;

				if (erase)
				{
					this->nodeMPI.interfaces[ni].shared_items.erase(current_key);
				}
				
			}
		}
	}


	//remove the body from the system if it exists
	bool to_delete;
	HIER_OTHERPHYSICS_INIT
	while HIER_OTHERPHYSICS_NOSTOP
	{	
		to_delete = false;
		ChPhysicsItem* item = PHpointer;

		ChVector<> bbmin, bbmax, mcenter;
		item->GetTotalAABB(bbmin, bbmax);
		item->GetCenter(mcenter);

		if (id == item->GetIdentifier())
		{
			// delete (at the end of while loop) the item
			to_delete = true;
		}

		ChPhysicsItem* olditem = PHpointer;
		HIER_OTHERPHYSICS_NEXT
		
		// Maybe that the item must be removed from the ChSystem
		if (to_delete)
		{
			ChSharedPtr<ChPhysicsItem> shpointer(olditem);
			olditem->AddRef(); // because wrapping normal (not new) ptr with shared pointer
			this->Remove(shpointer);
		}
		
	}
}




void ChSystemMPI::InterDomainSynchronizeStates()
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

	// Prepare the sorted list of shared items (because hash maps aren't necessarily
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
		this->nodeMPI.interfaces[ni].mchstreami->Init();
		this->nodeMPI.interfaces[ni].mchstreamo->Seek(0);
		this->nodeMPI.interfaces[ni].mchstreamo->Init();
	}

	// 1 - Send states of items of interfaces,
	//     to synchronize master-slaves :

	for (unsigned int ni = 0; ni < num_interfaces; ni++)
	{
		if (this->nodeMPI.interfaces[ni].id_MPI != -1) 
		{
			//***TEST*** check interface matching correctness: a) size of shared objs.
	 		(*this->nodeMPI.interfaces[ni].mchstreamo) << (int)this->nodeMPI.interfaces[ni].shared_items.size();

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
				   if (this->nodeMPI.IsInto(mcenter)) // only master can be cloned
				  {
						// Must be sent to neighbour domain, so:
						//  1- Add to interface hash table (if not exited completely)
						//if (to_delete != true)
						 this->nodeMPI.interfaces[ni].shared_items.insert(item->GetIdentifier(), 
							ChInterfaceItem(item,ChInterfaceItem::INTERF_NOT_INITIALIZED));

						//  2- Serialize to persistent data to be sent via MPI.
						//     Only the 'owner' domain (the one with the center of aabb) 
						//     will send data, to avoid n-uple sends at the corners.
						try 
						{
								//GetLog() << "ID=" << this->nodeMPI.id_MPI 
								//	 << "  SERIALIZE: " << item->GetRTTI()->GetName() 
								//	 << "  key=" << item->GetIdentifier() << " ptr=" << (int)item 
								//	 << "  to ID=" << this->nodeMPI.interfaces[ni].id_MPI << "\n";
							this->nodeMPI.interfaces[ni].mchstreamo->AbstractWrite(item);
						}
						catch (ChException myex)
						{
							GetLog() << "ERROR serializing MPI item:\n " << myex.what() << "\n";
						}
				  }
				   else
				  {
						this->nodeMPI.interfaces[ni].shared_items.insert(item->GetIdentifier(), 
							ChInterfaceItem(item,ChInterfaceItem::INTERF_NOT_INITIALIZED));
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
					
							// GetLog() << "ID=" << this->nodeMPI.id_MPI 
							//	 << "  DESERIALIZED: " << newitem->GetRTTI()->GetName() 
							//	 << "  key=" << newitem->GetIdentifier() << " ptr=" << (int)newitem
							//	 << "  from ID=" << nodeMPI.interfaces[ni].id_MPI << "\n";
					}
					catch (ChException myex)
					{
						GetLog() << "ERROR deserializing MPI item:\n " << myex.what() << "\n";
					}
					ChSharedPtr<ChPhysicsItem> ptritem(newitem);

					// 2-add to system
					if (newitem)
						this->AddSafely(ptritem);

					ChVector<> bbmin, bbmax;
					newitem->SyncCollisionModels();
					newitem->GetTotalAABB(bbmin, bbmax);

					// 3-add to interfaces hash table, 
					for (unsigned int sui = 0; sui<num_interfaces; sui++)
						if  ( this->nodeMPI.interfaces[sui].id_MPI != -1) // exclude unexisting domains
						{
							if (this->nodeMPI.IsAABBoverlappingInterface(sui, bbmin, bbmax))
								this->nodeMPI.interfaces[sui].shared_items.insert(newitem->GetIdentifier(), 
									ChInterfaceItem(newitem,ChInterfaceItem::INTERF_NOT_INITIALIZED) );
							
						}

				}
			}

		}
	}


	// Set MASTER/SLAVE flags
	for (unsigned int ni = 0; ni<num_interfaces; ni++)
	{
		if ( this->nodeMPI.interfaces[ni].id_MPI != -1) // exclude unexisting domains
		{
			ChHashTable<int,ChInterfaceItem>::iterator hiterator = this->nodeMPI.interfaces[ni].shared_items.begin();
			while (hiterator != this->nodeMPI.interfaces[ni].shared_items.end())
			{
				ChVector<> bbmin, bbmax, center;
				hiterator->second.item->GetTotalAABB(bbmin, bbmax);
				hiterator->second.item->GetCenter(center);
				
				hiterator->second.type = ChInterfaceItem::INTERF_SLAVESLAVE;

				if (this->nodeMPI.IsIntoInterface(ni,center))
					hiterator->second.type = ChInterfaceItem::INTERF_SLAVE;

				if (this->nodeMPI.IsInto(center))
					hiterator->second.type = ChInterfaceItem::INTERF_MASTER;

				++hiterator;
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
	std::list<ChPhysicsItem*>::iterator miterator = this->Get_otherphysicslist()->begin();
	while (miterator != this->Get_otherphysicslist()->end())
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

		if (ChAssemblyMPI* assem = dynamic_cast<ChAssemblyMPI*>(*miterator))
		{
			std::vector<ChBody*>::iterator assem_bod = assem->Get_bodylist()->begin();
			while ( assem_bod != assem->Get_bodylist()->end() )
			{
				ChBody* cbod = (ChBody*)*assem_bod;
				cbod->GetTotalAABB(mmin, mmax);
				sprintf(buffer, "%d, %d, %d, %g, %g, %g, %g, %g, %g ,\n", this->nodeMPI.id_MPI, cbod->GetIdentifier(), mshared, mmin.x, mmin.y, mmin.z, mmax.x, mmax.y, mmax.z);
				mstring.append(buffer);
				assem_bod++;
			}
		}
		++miterator;
	}

	output.WriteOrdered((char*)mstring.c_str(), strlen(mstring.c_str()));
}


void ChSystemMPI::WriteOrderedDumpState(ChMPIfile& output)
{
	// save items contained in domain on file, if any, as xyz position and rotation
	std::string mstring = "";
	chrono::Vector pos;
	chrono::Quaternion rot;
	chrono::Vector angs;
	std::list<ChPhysicsItem*>::iterator iterbod = this->Get_otherphysicslist()->begin();
	while (iterbod != this->Get_otherphysicslist()->end())
	{
		ChVector<> mmin, mmax;
		(*iterbod)->GetTotalAABB(mmin, mmax);
		
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
				if (nodeMPI.interfaces[i].shared_items.present( (*iterbod)->GetIdentifier() ) )
				{
					ChHashTable<int, ChInterfaceItem>::iterator mhashiter = nodeMPI.interfaces[i].shared_items.find( (*iterbod)->GetIdentifier() );
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

		if (ChBodyDEMMPI* bod = dynamic_cast<ChBodyDEMMPI*>(*iterbod))
		{
			pos = bod->GetPos();
			rot = bod->GetRot();
			angs = rot.Q_to_NasaAngles();
			char buffer[100];
			sprintf(buffer, "%d, %d, %d, %g, %g, %g, %g, %g, %g,\n", this->nodeMPI.id_MPI, bod->GetIdentifier(), mshared, pos.x, pos.y, pos.z, angs.x, angs.y, angs.z);
			mstring.append(buffer);
		}
		else if (ChAssemblyMPI* assem = dynamic_cast<ChAssemblyMPI*>(*iterbod))
		{
			std::vector<ChBody*>::iterator assem_bod = assem->Get_bodylist()->begin();
			while ( assem_bod != assem->Get_bodylist()->end() )
			{
				ChBody* cbod = (ChBody*)*assem_bod;
				pos = cbod->GetPos();
				rot = cbod->GetRot();
				angs = rot.Q_to_NasaAngles();
				char buffer[100];
				sprintf(buffer, "%d, %d, %d, %g, %g, %g, %g, %g, %g,\n", this->nodeMPI.id_MPI, cbod->GetIdentifier(), mshared, pos.x, pos.y, pos.z, angs.x, angs.y, angs.z);
				mstring.append(buffer);
				assem_bod++;
			}
		}
		iterbod++;
	}

	output.WriteOrdered((char*)mstring.c_str(), strlen(mstring.c_str()));
}



void ChSystemMPI::WriteOrderedDumpContacts(ChMPIfile& output)
{
	// save contacts in domain on file, if any
	std::string mstring = "";
	chrono::Vector pos;
	chrono::Quaternion rot;
	chrono::Vector angs;

	if (ChContactContainerDEMMPI* c_con = dynamic_cast<ChContactContainerDEMMPI*>(this->GetContactContainer()))
	{
		std::list<ChContactDEM*>::iterator itercon = c_con->Get_contactlist()->begin();
		while (itercon != c_con->Get_contactlist()->end() )
		{
			ChContactDEM* cnct = (ChContactDEM*)*itercon;
			char buffer[100];
			sprintf(buffer, "%d, %d, %d, %g, %g, %g, %g, %g, %g,\n", this->nodeMPI.id_MPI, cnct->GetModelA()->GetPhysicsItem()->GetIdentifier(), cnct->GetModelB()->GetPhysicsItem()->GetIdentifier(), cnct->GetContactForce().x, cnct->GetContactForce().y, cnct->GetContactForce().z, cnct->GetContactP1().x, cnct->GetContactP1().y, cnct->GetContactP1().z);
			mstring.append(buffer);
			itercon++;
		}

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
			int nmaster = 0;
			int nslave = 0;
			int nslaveslave = 0;
			this->nodeMPI.interfaces[i].sorted_items.clear();
			ChHashTable<int,ChInterfaceItem>::iterator hiterator = this->nodeMPI.interfaces[i].shared_items.begin();
			while (hiterator != this->nodeMPI.interfaces[i].shared_items.end())
			{
				if ((*hiterator).second.type == ChInterfaceItem::INTERF_MASTER) ++nmaster;
				if ((*hiterator).second.type == ChInterfaceItem::INTERF_SLAVE)  ++nslave;

				this->nodeMPI.interfaces[i].sorted_items.push_back(
						std::pair<int, ChInterfaceItem*> (hiterator->first, &hiterator->second)
					);
				++hiterator;
			}
			std::sort(this->nodeMPI.interfaces[i].sorted_items.begin(), 
					  this->nodeMPI.interfaces[i].sorted_items.end()    );

			sprintf(sbuffer, "    Interface to node ID=%d   has %d items (%d masters, %d slaves)\n", nodeMPI.interfaces[i].id_MPI, nodeMPI.interfaces[i].shared_items.size(), nmaster, nslave);
			mstring.append(sbuffer);

			std::vector< std::pair<int,ChInterfaceItem*> >::iterator siterator = this->nodeMPI.interfaces[i].sorted_items.begin();
			while (siterator != this->nodeMPI.interfaces[i].sorted_items.end())
			{
				sprintf(sbuffer, "          Item ID=%d  -  type=", (*siterator).first);
				mstring.append(sbuffer);
				if ((*siterator).second->type == ChInterfaceItem::INTERF_MASTER)
					mstring.append("MASTER       *");
				if ((*siterator).second->type == ChInterfaceItem::INTERF_SLAVE)
					mstring.append("Slave        .");
				if ((*siterator).second->type == ChInterfaceItem::INTERF_SLAVESLAVE)
					mstring.append("slave/slave");
				if ((*siterator).second->type == ChInterfaceItem::INTERF_NOT_INITIALIZED)
					mstring.append("NOT_INITIALIZED!!!");
				
				ChVector<> mcenter;
				(*siterator).second->item->GetCenter(mcenter);

				sprintf(sbuffer, "      center %g %g %g", mcenter.x, mcenter.y, mcenter.z);
				mstring.append(sbuffer);

				mstring.append("\n");
				++siterator;
			}
			
		}
	}
	mstring.append("\n");

	output.WriteOrdered((char*)mstring.c_str(), strlen(mstring.c_str()));
}



void ChSystemMPI::AddSafely (ChSharedPtr<ChPhysicsItem> newitem)
{
	int mid = newitem->GetIdentifier();
	bool duplicate = false;

	HIER_BODY_INIT
	while HIER_BODY_NOSTOP
	{
		if (Bpointer->GetIdentifier() == mid)
			duplicate = true;
		HIER_BODY_NEXT
	}
	HIER_OTHERPHYSICS_INIT
	while HIER_OTHERPHYSICS_NOSTOP
	{
		if (PHpointer->GetIdentifier() == mid)
			duplicate = true;
		HIER_OTHERPHYSICS_NEXT
	}

	if(duplicate)
		GetLog() << "    WARNING! adding two times an object with same ID=" << mid <<"\n";
	
	//if(!this->nodeMPI.IsInto(newitem->GetCenter())

	this->Add(newitem);
}



} // END_OF_NAMESPACE____


////// end
