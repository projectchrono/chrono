///////////////////////////////////////////////////
//   
//   ChClassRegister.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "core/ChClassRegister.h"
     
        
namespace chrono
{    
   
/// The root of the list of ChClassRegister<t> objects, each 
/// will contain the name ID of the class,and other useful things
/// such as the method which can create a 't' object in runtime.
    

ChClassRegisterCommon** ChClassRegisterCommon::GetStaticHeadAddr()
{
	static ChClassRegisterCommon* mlocalHead = 0;		// A STATIC DATA
	return &mlocalHead;//&m_pHead;
}





}  // END_OF_NAMESPACE____









