///////////////////////////////////////////////////
//
//   ChLinkContact.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 
  
#include "physics/ChLinkContact.h"
#include "physics/ChSystem.h"
#include "physics/ChCollide.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{


using namespace collision;
using namespace geometry;






////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
//
//   CLASS FOR CONTACT
//
//

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegisterABSTRACT<ChLinkContact> a_registration_ChLinkContact;
 
ChLinkContact::ChLinkContact ()
{ 
    kin_fri = 0.0;
    sta_fri = 0.0;
    restitution = 0.0;
}


ChLinkContact::~ChLinkContact ()
{

}

void ChLinkContact::Copy(ChLinkContact* source)
{
    // first copy the parent class data...
    //
    ChLink::Copy(source);

    // copy custom data:
    kin_fri =       source->kin_fri;
    sta_fri =       source->sta_fri;
    restitution =   source->restitution;
}





} // END_OF_NAMESPACE____


