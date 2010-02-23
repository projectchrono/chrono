#ifndef CHCONTACTCONTAINERBASE_H
#define CHCONTACTCONTAINERBASE_H

///////////////////////////////////////////////////
//
//   ChContactContainerBase.h
//
//   Class for container of many contacts
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "physics/ChPhysicsItem.h"
#include "physics/ChContact.h"
#include "physics/ChMaterialCouple.h"
#include "collision/ChCCollisionInfo.h"

namespace chrono
{

/// Class to be used as a callback interface for some user defined
/// action to be taken each time a contact is added to the container.
/// It can be used to modify the friction value (as default it is the
/// average of the friction of the two bodies).
/// The user should implement an inherited class and
/// implement a custom ContactCallback() function.

class ChAddContactCallback
{
public:
			/// Callback, used to report contact points being added to the container.
			/// This must be implemented by a child class of ChAddContactCallback
	virtual void ContactCallback (const  collision::ChCollisionInfo& mcontactinfo, ///< get info about contact (cannot change it)
								  ChMaterialCouple&  material 			  		   ///< you can modify this! 
								) = 0;			
};


/// Class to be used as a callback interface for some user defined
/// action to be taken for each contact (already added to the container,
/// maybe with already computed forces).
/// The user should implement an inherited class and
/// implement a custom ReportContactCallback() function.

class ChReportContactCallback
{
public:
			/// Callback, used to report contact points already added to the container.
			/// This must be implemented by a child class of ChReportContactCallback.
			/// If returns false, the contact scanning will be stopped.
	virtual bool ReportContactCallback (
					const ChVector<>& pA,				///< get contact pA
					const ChVector<>& pB,				///< get contact pB
					const ChMatrix33<>& plane_coord,	///< get contact plane coordsystem (A column 'X' is contact normal)
					const double& distance,				///< get contact distance
					const float& mfriction,			  	///< get friction info
					const ChVector<>& react_forces,		///< get react.forces (if already computed). In coordsystem 'plane_coord'
					const ChVector<>& react_torques,	///< get react.torques, if rolling friction (if already computed).
					collision::ChCollisionModel* modA,	///< get model A (note: some containers may not support it and could be zero!)
					collision::ChCollisionModel* modB	///< get model B (note: some containers may not support it and could be zero!)
										) = 0;			
};





///
/// Class representing a container of many contacts.
/// There might be implementations of this interface
/// in form of plain CPU linked lists of ChContact objects,
/// or highly optimized GPU buffers, etc. etc. 
/// This is only the basic interface with the features that are in common.
///

class ChContactContainerBase : public ChPhysicsItem {

	CH_RTTI(ChContactContainerBase,ChPhysicsItem);

protected:
				//
	  			// DATA
				//

	ChAddContactCallback* add_contact_callback;	
	ChReportContactCallback* report_contact_callback; 
public:
				//
	  			// CONSTRUCTORS
				//

	ChContactContainerBase () 
				{ 
					add_contact_callback =0;
					report_contact_callback =0;
				};

	virtual ~ChContactContainerBase () {};

				//
	  			// FUNCTIONS 
				//

					/// Tell the number of added contacts. To be implemented by child classes.
	virtual int  GetNcontacts  () = 0;

					/// Remove (delete) all contained contact data. To be implemented by child classes.
	virtual void RemoveAllContacts() = 0;

					/// The collision system will call BeginAddContact() before adding
					/// all contacts (for example with AddContact() or similar). By default
					/// it deletes all previous contacts. Custom more efficient implementations
					/// might reuse contacts if possible.
	virtual void BeginAddContact() { RemoveAllContacts(); }

					/// Add a contact between two models, storing it into this container.
					/// To be implemented by child classes.
					/// Some specialized child classes (ex. one that uses GPU buffers)
					/// could implement also other more efficient functions to add many contacts
					/// in a batch (so that, for example, a special GPU collision system can exploit it);
					/// yet most collision system might still fall back to this function if no other 
					/// specialized add-functions are found.
	virtual void AddContact(const collision::ChCollisionInfo& mcontact) =0;

					/// The collision system will call EndAddContact() after adding
					/// all contacts (for example with AddContact() or similar). By default
					/// it does nothing.
	virtual void EndAddContact() {};




					/// Sets a callback to be used each time a contact point is 
					/// added to the container. Note that not all child classes can
					/// support this function in all circumstances (example, the GPU container
					/// won't launch the callback for all its points because of performance optimization)
	void SetAddContactCallback(ChAddContactCallback* mcallback) {add_contact_callback = mcallback;}

					/// Scans all the contacts and for each contact exacutes the ReportContactCallback()
					/// function of the user object inherited from ChReportContactCallback.
					/// Child classes of ChContactContainerBase should try to implement this (although
					/// in some highly-optimized cases as in ChContactContainerGPU it could be impossible to
					/// report all contacts).
	virtual void ReportAllContacts(ChReportContactCallback* mcallback) =0;

};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
