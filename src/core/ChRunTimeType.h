//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHRUNTIMETYPE_H
#define CHRUNTIMETYPE_H


//////////////////////////////////////////////////
//  
//   ChRunTimeType.h
//
//   A toolkit to simulate the C++ RTTI (run time type 
//   information) features, without having to enable 
//   RTTI in compilation - for faster performance.
//   Each class can use these RTTI features, by inserting
//   just a simple macro into its header.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <stdio.h>
#include <string>
#include "core/ChLog.h"

namespace chrono
{



/// ChRunTimeType is a base class containing information
/// about type of a generic C++ class.

class ChRunTimeType
{
private:
		//
		// DATA
		//
	
				/// The base class RTTI object
    const ChRunTimeType* m_pkBaseRTTI;

				/// The class name ID.
    std::string m_kName;

public:

		// 
		// CONSTRUCTORS
		//
			
				/// Build the RTTI infos given the name ID of this class
				/// and the base class, if any.
    ChRunTimeType (const char* acName, const ChRunTimeType* pkBaseRTTI)
    {
        m_kName = /*"chrono::"+ */ std::string(acName);
        m_pkBaseRTTI = pkBaseRTTI;
    }

		// 
		// METHODS
		//
			
	bool operator==(const ChRunTimeType& other) const 
					{ 
						return other.m_kName == m_kName; 
					}

				/// Gets the run-time type information of the parent class.
    const ChRunTimeType* GetBaseRTTI () const
					{
						return m_pkBaseRTTI;
					}


				/// Get the name ID of the class, at run-time.
    const char* GetName () const
					{
						return m_kName.c_str();
					}

				/// Tells if this type is exactly the same of another
	bool IsExactlyClass (const ChRunTimeType* pkQueryRTTI) const 
					{ 
						return (pkQueryRTTI->m_kName == m_kName);
						//return ( this == pkQueryRTTI );
					} 
		
				/// Tells if this type is derived from some other type
				/// Note: for the double inheritance, only the 1st parent tree is scanned.
	bool IsDerivedFromClass (const ChRunTimeType* pkQueryRTTI)  const\
					{  
						const ChRunTimeType* pkRTTI = this; 
						while ( pkRTTI ) 
						{ 
							if (pkRTTI->m_kName == pkQueryRTTI->m_kName)//if ( pkRTTI == pkQueryRTTI ) 
								return true; 
							pkRTTI = (ChRunTimeType*)pkRTTI->GetBaseRTTI(); 
						} 
						return false; 
					} 

};




///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

//
//  MACROS
//
//  These macros can be inserted into the class definition of
//  whatever C++ code, in order to enable the 'simulated RTTI'
//  features of the ChRunTimeType() class.
//


#define CH_RTTI_ROOT(classname) \
	\
	public: \
		virtual const chrono::ChRunTimeType* GetRTTI () const { static ChRunTimeType m_RTTI(#classname,NULL); return &m_RTTI; } \
		static  const chrono::ChRunTimeType* GetClassRTTI()   { static ChRunTimeType m_RTTI(#classname,NULL); return &m_RTTI; }; 

#define CH_RTTI(classname, parentclassname) \
	\
	public: \
		virtual const chrono::ChRunTimeType* GetRTTI () const { static ChRunTimeType m_RTTI(#classname, parentclassname::GetClassRTTI()); return &m_RTTI; } \
		static  const chrono::ChRunTimeType* GetClassRTTI()   { static ChRunTimeType m_RTTI(#classname, parentclassname::GetClassRTTI()); return &m_RTTI; }; 



//////////////////////////////////////////////////////////////
//
// MISC FUNCTIONS for runtime type testing and casting
//

		/// Check if an object has some class type of another.
#define ChIsExactlyClass(classname,pObj) \
	( (pObj) ? (pObj)->GetRTTI()->IsExactlyClass(classname::GetClassRTTI()) : false )

		/// Check if some object is derived from some class
#define ChIsDerivedFromClass(classname,pObj) \
	( (pObj) ? (pObj)->GetRTTI()->IsDerivedFromClass(classname::GetClassRTTI()) : false )

		/// Perform a static cast from a type to another.
#define ChStaticCast(classname,pObj) \
	((classname*)(void*)(pObj))

		/// Performs a dynamic cast.
#define ChDynamicCast(classname,pObj) \
	( (pObj) ? ( (pObj)->GetRTTI()->IsDerivedFromClass(classname::GetClassRTTI()) ? (classname*)pObj : NULL ) : NULL )






};  // END_OF_NAMESPACE____

#endif 


