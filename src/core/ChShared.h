//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHSHARED_H
#define CHSHARED_H

//////////////////////////////////////////////////
//  
//   ChShared.h
//
// Base class for shared objects, i.e. objects
// which must be managed with smart pointers of
// intrusive type (where the reference counting 
// method is used to understand when to delete 
// the object)
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChApiCE.h"
#include "ChRunTimeType.h"
#include "ChSmartpointers.h"



namespace chrono 
{




/// Base class for shared objects, i.e. objects
/// which must be managed with smart pointers of
/// intrusive type (where the reference counting 
/// method is used to understand when to delete 
/// the object).
///
/// Intrusive smart pointers must call AddRef()
/// each time they reference such kind of object,
/// and call RemoveRef() each time that they do
/// not reference it anymore. When the reference
/// count falls to 0, the object is automatically
/// deleted. 



class ChApi ChShared {
						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI_ROOT(ChShared);

private:
			// 
			// DATA
			//

	mutable unsigned int m_ref_counter;


public:
			//
			//	CONSTRUCTORS
			// 

	ChShared()	
			: m_ref_counter(1) 
			{
			}


	virtual ~ChShared() 
			{	
			}

			//
			//	FUNCTIONS
			// 

				/// Add a reference to this object (tell that someone, in addition to
				/// the creator, is using it. For example some pointer is pointing to
				/// it... 
				/// In fact intrusive smart pointers automatically call this when copied.
	void AddRef() 
			{
				m_ref_counter++;
			}
	
				/// Remove a reference to this object (tell that someone
				/// who was using it, now is not interested in it anymore).
				/// If the reference count drops to zero, the last time
				/// RemoveRef() is called, the object is automatically
				/// deleted. Also the creator must call this, instead of delete().
				/// In fact intrusive smart pointers automatically call this.
	void RemoveRef()
			{
				m_ref_counter--; 
				
				if (m_ref_counter==0)
					delete this;
			}

				/// Tells how many items are sharing this object, i.e. the
				/// reference count.
	int ReferenceCount() {return m_ref_counter;}

};

 



// The following functions are needed as interfaces to the smart_ptr 
// implementation when using the intrusive policy.

//inline void intrusive_ptr_add_ref(ChShared * p){p->AddRef();}
//inline void intrusive_ptr_release(ChShared * p){p->RemoveRef();}
//inline bool intrusive_ptr_is_ref (ChShared * p){return (p->ReferenceCount() >= 1);}






} // END_OF_NAMESPACE____

#endif
