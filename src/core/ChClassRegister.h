#ifndef CHCLASSREGISTER_H
#define CHCLASSREGISTER_H


//////////////////////////////////////////////////
//  
//   ChClassRegister.h
//
//   A class factory to register C++ classes. This can
//   be used, for example, for dynamic object creation 
//   from a type ID textual name, in serialization etc.
//   (the so called 'class factory' mechanism).
//
//   These classes are based on a modified version of 
//   the "Eternity" persistence library, copyright 
//   1999-2003 Nicola Santi.
//
//   This functionality can be applied ONLY to classes 
//   which implement the custom Chrono Run-time type 
//   information (RTTI), that is classes with the 
//   CH_RTTI_xxx macros from the file ChRunTimeType.h
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

///////////////////////////////////////////////////
//
//  HOW TO REGISTER A CLASS IN THE CLASS FACTORY
//
//   Remember: you can register only classes which have the 
//  'chrono run-time-type-checking' enabled, that is classes
//  which have the macro  CH_RTTI(...) inside their class{...}
//  declaration!!!
//   Assuming you have such a class, say it is named 'myEmployee',
//  you just have to put the following line in your .cpp code
//  (for example in myEmployee.cpp, but not in myEmployee.h!):
//
//  chrono::ChClassRegister<myEmployee> my_registration;
//
/////////////////////////////////////////////////////



#include <stdio.h>
#include <string>
#include <typeinfo>
#include "core/ChLog.h"
#include "core/ChRunTimeType.h"

namespace chrono
{



/// ChClassRegisterCommon is a base abstract class which allows
/// a common management of all template classes ChClassRegister<t>.

class ChClassRegisterCommon
{
public:

			//
			// DATA
			//

				/// Each item in the gloabl list of ChClassRegister<t>
				/// object has a pointer to the next item
				/// named m_pNext. The last element has a NULL
				/// value inside m_pNext.
	ChClassRegisterCommon	*m_pNext;
		
			//
			// METHODS
			//

				/// All instancied ChClassRegister<t> are collect inside
				/// an unique global list. this 'head' is the head of
				/// that STATIC list.
	static ChClassRegisterCommon** GetStaticHeadAddr();

				/// The signature of create method for derived classes.
	virtual void* create( std::string& class_name ) = 0 ;

				/// The signature of get_conventional_name method for derived classes.
	virtual std::string get_conventional_name (std::string& compiler_name ) = 0 ;

};




/// ChClassRegisterABSTRACT<t> is like the more specialized 
/// ChClassRegister<t>, which you will use more often, but the 
/// ..ABSTRACT version is needed to register abstract classes (those
/// with some pure virutal member, which cannot be instantiated with 'new'. )

template <class t>
class ChClassRegisterABSTRACT: public ChClassRegisterCommon
{
protected:

		//
		// DATA
		//

			/// Name of the class for dynamic creation
	std::string		 m_sConventionalName;
												  
public:

		//
		// CONSTRUCTORS
		//

			/// Default constructor: uses the chrono simulated RTTI 
			/// features to get a name for the class. 
			/// IT CAN BE USED ONLY ON CHRONO CLASSES WITH RTTI MACROS 
	
	 ChClassRegisterABSTRACT()
					{
						m_pNext = *GetStaticHeadAddr(); 
						*GetStaticHeadAddr() = this;

						// set name using the 'fake' RTTI system of Chrono
						this->m_sConventionalName = t::GetClassRTTI()->GetName(); 
					}

 			/// Destructor (removes this from the global list of 
			/// ChClassRegister<t> object.
	virtual ~ChClassRegisterABSTRACT()
					{
						ChClassRegisterCommon	**ppNext = GetStaticHeadAddr();
						for( ; *ppNext ; ppNext = &(*ppNext)->m_pNext )
						{
							if( *ppNext == this )
							{
								*ppNext = (*ppNext)->m_pNext;
								break;
							}
						}
					}
	

		//
		// METHODS
		//

	virtual bool IsAbstract() {return true;}

	virtual void* create( std::string& class_name )
					{
						assert(this->m_sConventionalName!=class_name); // cannot instance an abstract class 
						return 0;
					}

			/// Return the conventional name ID of class t if 
			/// typeid(t).name() match with compiler_name.
			///	Otherwise return an empty.
	virtual std::string get_conventional_name (std::string& compiler_name )
					{
 						return ( compiler_name==typeid(t).name() ) 
								? this->m_sConventionalName
								: std::string("")	;
					}

};

/// ChClassRegister<t> is responsable for registration of C++ classes
/// at compile time (each class name ID enters a static list of 
/// items when the program starts). 
/// During run time, it can be used in whatever moment to create
/// classes given their name ID.

template <class t>
class ChClassRegister: public ChClassRegisterABSTRACT <t>
{
												  
public:

	virtual bool IsAbstract() {return false;}

			/// Create an instance from class t if the name ID is the same 
			/// specified in class_name. If the name does not match, returns NULL.
	virtual void* create( std::string& class_name )
					{
 						return (this->m_sConventionalName==class_name)
								? (void*)( new t )
								: 0	;
					}

};



/// This function return a pointer to an object
///	that belong to the class specified by the
///	string cls_name. If success put in *ppObj
///	that pointer, otherwise put a NULL value
///	in *ppObj.

template<class T>
void create( std::string cls_name, T** ppObj)
{
	ChClassRegisterCommon* pCurrent = *ChClassRegisterCommon::GetStaticHeadAddr();

		//the follow line reinterpret several times
		//invalid null pointer so compilers could
		//warn about type mismatch.
	for( ; pCurrent ; pCurrent = pCurrent->m_pNext )
    if( (*ppObj = reinterpret_cast<T*>( pCurrent->create(cls_name)))!=NULL ) break;

}






};  // END_OF_NAMESPACE____

#endif 


