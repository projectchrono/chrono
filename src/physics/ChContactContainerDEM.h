//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHCONTACTCONTAINERDEM_H
#define CHCONTACTCONTAINERDEM_H

///////////////////////////////////////////////////
//
//   ChContactContainerDEM.h
//
//   Class for container of many contacts, as CPU
//   typical linked list of ChContactDEM objects
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "physics/ChContactContainerBase.h"
#include "physics/ChContactDEM.h"
#include <list>

namespace chrono
{


///
/// Class representing a container of many contacts, 
/// implemented as a typical linked list of ChContactNode
/// objects (contacts between 3DOF nodes and 6DOF bodies)
///

class ChApi ChContactContainerDEM : public ChContactContainerBase {

	CH_RTTI(ChContactContainerDEM,ChContactContainerBase);

protected:
				//
	  			// DATA
				//

	std::list<ChContactDEM*>   contactlist; 

	int n_added;

	std::list<ChContactDEM*>::iterator lastcontact;


public:
				//
	  			// CONSTRUCTORS
				//

	ChContactContainerDEM ();

	virtual ~ChContactContainerDEM ();


				//
	  			// FUNCTIONS
				//

					/// Gets the list of contacts -low level function-.
					/// NOTE! use this list only to enumerate etc., but NOT to
					/// remove or add items (use the appropriate Remove.. and Add..
					/// functions instead!)
	std::list<ChContactDEM*>* Get_contactlist() {return &contactlist;}

					/// Tell the number of added contacts
	virtual int GetNcontacts  () {return n_added;};

					/// Remove (delete) all contained contact data.
	virtual void RemoveAllContacts();

					/// The collision system will call BeginAddContact() before adding
					/// all contacts (for example with AddContact() or similar). Instead of
					/// simply deleting all list of the previous contacts, this optimized implementation
					/// rewinds the link iterator to begin and tries to reuse previous contact objects
					/// until possible, to avoid too much allocation/deallocation.
	virtual void BeginAddContact();

					/// Add a contact between two frames.
	virtual void AddContact(const collision::ChCollisionInfo& mcontact);

					/// The collision system will call BeginAddContact() after adding
					/// all contacts (for example with AddContact() or similar). This optimized version
					/// purges the end of the list of contacts that were not reused (if any).
	virtual void EndAddContact();

					/// Scans all the contacts and for each contact exacutes the ReportContactCallback()
					/// function of the user object inherited from ChReportContactCallback.
					/// Child classes of ChContactContainerBase should try to implement this (although
					/// in some highly-optimized cases as in ChContactContainerGPU it could be impossible to
					/// report all contacts).
	virtual void ReportAllContacts(ChReportContactCallback* mcallback);



					/// In detail, it computes jacobians, violations, etc. and stores 
					/// results in inner structures of contacts.
	virtual void Update (double mtime);		


			//
			// STATE FUNCTIONS
			//

				// (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
	virtual void IntLoadResidual_F(const unsigned int off,	ChVectorDynamic<>& R, const double c );

			//
			// LCP INTERFACE
			//

	virtual void ConstraintsFbLoadForces(double factor);
};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
