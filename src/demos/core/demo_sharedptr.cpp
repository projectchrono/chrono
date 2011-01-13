///////////////////////////////////////////////////
//
//   Demos code about 
//
//     - smart pointers
//     - shared objects
//
//	 CHRONO 
//   ------
//   Multibody dinamics engine
// 
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 
  

#include <stdio.h>
#include <vector>

#include "core/ChException.h"
#include "core/ChShared.h"
#include "core/ChFrame.h"


using namespace chrono;


// A simple class, inherited from the ChShared class.
//  Since it is inherited from the ChShared class, we
// will be able to use 'intrusive' smart pointers,
// that is a very fast type of smart pointers based
// on reference counting, the ChSharedPtr
//  Note that you can still use smart pointers with 
// other classes (not inherited from ChShared), but
// using the non-intrusive smart pointers of type 
// 'ChSmartPtr' (a bit slower than ChSharedPtr..)

class cTest : public ChShared
{
public:  
	int mfoo;
	
	cTest ()  
		{	GetLog() <<"  +Building object of class cTest \n"; }

	~cTest () 
		{	GetLog() <<"  -Deleting object of class cTest \n"; }
};




int main(int argc, char* argv[])
{


	{
		// EXAMPLE 1: 
		//
		//  In C++ you create and delete dynamic data using the
		// new ... delete   pair. 
		//  This is good in most cases, as in the simple example above..

		GetLog() << " Example: use the classic new - delete method: \n";

		cTest* mtest1 = new cTest;

		delete mtest1;

	}
	

 
	{
		// EXAMPLE 2: 
		//
		//  Well, if your oject is inherited from ChShared class, you
		// can also use the  RemoveRef() function, which automatically
		// deletes the object, instead of calling 'delete':

		GetLog() << "\n Example: use AddRef() and RemoveRef() of ChShared class \n";

		cTest* mtest2 = new cTest;	// creation of ChShared objects sets to 1 the ref.count

		mtest2->RemoveRef();		// drops ref.count to 0, and automatically deletes
	}



	{
		// EXAMPLE 3: 
		//
		//  What's the use of this RemoveRef() stuff? Simple: if multiple
		// pointers are referencing the same object, you can use the AddRef() 
		// function each time, then call RemoveRef() when you aren't interested 
		// anymore, and deletion will happen automatically.
		//  In detail, AddRef() increments a counter (initialized to 1 at obj.
		// creation) and each RemoveRef() decrements it, until reaches 0 ->auto deletion.

		GetLog() << "\n Example: use multiple AddRef() and RemoveRef() of ChShared class \n";

		cTest* mtest2b = new cTest;	//  ref count =1

		cTest* mtest3b = mtest2b;
		mtest3b->AddRef();			//  ref count =2


		mtest2b->RemoveRef();		//  ref count =1
		mtest3b->RemoveRef();		//  ref count =0 ->automatically delete!
	}

     
	{
		// EXAMPLE 4: 
		//
		//  Ok, if you look at the previous example, there's still the risk of
		// forgetting a RemoveRef() or doing an extra RemoveRef() - in the first
		// case you get memory leaks, in the second, double deletion. Bad...
		//  That's why here come the 'smart pointers', which take care AUTOMATICALLY
		// of memory management! 
		//  In this example, we use the ChSharedPtr smart pointers, which can be
		// used for classes inherited from ChShared. It uses intrusive policy, for
		// maximum speed. 
		//  When smart pointers are deleted (as in this example, the compiler knows
		// to get rid of them automatically, when the {..} block ends ) they also 
		// take care of deleting the object they share, if no one remains interested 
		// in it. 
		//  Create a pointer as    ChSharedPtr<my_class>  pointer_name;

		GetLog() << "\n Example: use ChSharedPtr smart pointers \n";


			// Create an object which will be referenced (shared) by three 
			// pointers. During creation set the first smart pointer to it...
		ChSharedPtr<cTest> mtest3(new cTest);

			// other pointer to the object can be created by () copy construction or..
		ChSharedPtr<cTest> mtest4(mtest3);

			// other pointer to the object can be assigned by = operator
		ChSharedPtr<cTest> mtest5 = mtest3;


			// smart pointers behave exactly as traditional pointers... -> and . operators
		mtest4->mfoo   = 120;
		(*mtest4).mfoo = 120;
		
			// convert to raw pointer with get_ptr(), if needed (avoid if possible)
		cTest* raw_pt = mtest4.get_ptr();
		raw_pt->mfoo = 120;
			

			// HERE'S THE NICE PART!!!
			//  Finally, you DO NOT NEED TO CALL delete(), because the 
			// deletion will happen _automatically_ when the mtest3 and mtest4 
			// smart pointers will die (when exiting from the scope of this {..}
			// context). 
			//  This automatic deletion of shared objects is the MAIN REASON
			// of using the smart pointers. Memory handling is much easier (no leaks etc.).

	}

 
	{
		// EXAMPLE 5: 
		//
		//  Should you use smart pointers only with objects inherited
		// from the ChShared class? No..  
		//  In fact, you can also use non-intrusive smart pointers, called 
		// ChSmartPtr , so that your referenced object does NOT need to be 
		// inherited from ChShared class (but remember this is a bit slower..)


		GetLog() << "\n Example: use non-intrusive ChSmartPtr smart pointers \n";

			
		ChSmartPtr<Quaternion> mtest3(new Quaternion);

			// other pointer to the object can be created by () copy construction or..
		ChSmartPtr<Quaternion> mtest4(mtest3);

			// other pointer to the object can be assigned by = operator
		ChSmartPtr<Quaternion> mtest5 = mtest3;


			// smart pointers behave exactly as traditional pointers... -> and . operators
		mtest4->Normalize();
		

		
			//  Finally, you DO NOT NEED TO CALL delete(), because the 
			// deletion of the instanced Quaternion object will happen _automatically_ 
	}

   
	// NOTE: some common errors with smart pointers:
	//    
	//  -1-
	//
	//  it's ok to assign a raw pointer to a smart pointer..
	//
	//       cTest* mrawptr = new cTest;
	//       ChSharedPtr<cTest> mtestA(mrawptr);
	//
	//  ..but if you assign AGAIN the same 'raw pointer' to ANOTHER smart ptr
	//   you broke the automatic deletion mechanism (memory leak/corruption)
	//
	//       ChSharedPtr<cTest> mtestB(mrawptr);   BAD!!!!!!!!
	//
	//  ..so you should do rather:
	//
	//       ChSharedPtr<cTest> mtestB(mtestA);    OK!!!
	//
	// 
	//  -2- 
	// 
	//   Another error is to create circular dependancies between
	//  objects with shared pointers. This would cause memory
	//  leakages.
	//   There's a workaround. For example it's ok to point from
	//  object A to object B with a shared pointer, than from B to
	//  A with a normal 'raw' pointer. It would be a mistake to 
	//  to point 'back' from B to A with a second shared pointer
	//  becuase it would mean circular dependancy.




	return 0;
}


