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
	for (int i=0; i < 27; i++)
	{
		this->nodeMPI.interfaces[i].mstreami->clear();
		this->nodeMPI.interfaces[i].mstreamo->clear();
		this->nodeMPI.interfaces[i].mchstreami->Seek(0);
		this->nodeMPI.interfaces[i].mchstreamo->Seek(0);
	}

	// 1 - serialize objects that 'spill out' to interface buffers

GetLog() << "ID=" << this->nodeMPI.id_MPI << " CustomEndOfStep 1 \n";
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
				for (int bi = 0; bi<27; bi++)
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
		if (ChBodyMPI* mpibody = dynamic_cast<ChBodyMPI*>(PHpointer))
		{
			int oldflags = mpibody->GetOverlapFlags();
			GetLog() << "ID=" << this->nodeMPI.id_MPI << " CustomEndOfStep 3 oldflags = " << oldflags <<  "\n";
			int newflags = mpibody->ComputeOverlapFlags(this->nodeMPI);
			GetLog() << "ID=" << this->nodeMPI.id_MPI << " CustomEndOfStep 4 newflags = " << newflags <<  "\n";
			int changeflags = newflags & ~oldflags; // deal only with last overlapped domains 
			if (changeflags)
			{
				for (int bi = 0; bi<27; bi++)
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
		}
		HIER_OTHERPHYSICS_NEXT
	}

	// 2 - send buffers using MPI

	ChMPIrequest mrequest[27];

	for (int bi = 0; bi<27; bi++)
	{
		if  ( this->nodeMPI.interfaces[bi].id_MPI != -1) // exclude unexisting domains
		{
			GetLog() << "ID=" << this->nodeMPI.id_MPI << " send buffer to " << this->nodeMPI.interfaces[bi].id_MPI << " \n";

			int err = ChMPI::SendBuffer( this->nodeMPI.interfaces[bi].id_MPI, 
							   *(this->nodeMPI.interfaces[bi].mstreamo), 
							   ChMPI::MPI_STANDARD, 
							   true,	// non blocking send, as  MPI_Isend
							   &(mrequest[bi]));				   
		}
	}

	// 3 - receive buffers using MPI, deserialize and add to system
	for (int bi = 0; bi<27; bi++)
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
					// deserialize received data, with class factory
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

					// add to system
					if (newitem)
						this->Add(ptritem);
				}
			}

		}
	}

	// wait that all messages are sent before proceeding
	for (int bi = 0; bi<27; bi++)
	{
		if  ( this->nodeMPI.interfaces[bi].id_MPI != -1) // exclude unexisting domains
		{
			ChMPIstatus mstatus;
			ChMPI::Wait(&mrequest[bi], &mstatus);
		}
	}

GetLog() << "ID=" << this->nodeMPI.id_MPI << " CustomEndOfStep END \n";
}



} // END_OF_NAMESPACE____


////// end
