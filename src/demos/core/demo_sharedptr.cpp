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
			
			// just curious about how many references to the same underlying original object?		
		GetLog() << "    Number of references to object: " << mtest4.ReferenceCounter() << "\n"; 


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
		
			// just curious about how many references to the same underlying original object?		
		GetLog() << "    Number of references to object: " << mtest5.ReferenceCounter() << "\n"; 

		
			//  Finally, you DO NOT NEED TO CALL delete(), because the 
			// deletion of the instanced Quaternion object will happen _automatically_ 
	}


	{
		// EXAMPLE 6: 
		//
		//  How to use ChSharedPtr and ChSmartPtr in case you 
		// need casting.

		GetLog() << "\n Example: casting between ChSharedPtr pointers \n";

			// Define a hierarchy of classes, just for this example.
			// Note the 'virtual'! (otherwise dynamic casting won't work)
		class cTestA : public ChShared
		{
		public:  
			cTestA () {}
			virtual ~cTestA () {}
			virtual void PrintMe () 
				{	GetLog() <<"     Hallo! I am a cTestA object!\n"; }
		};

			// This is inherited from cTestA
		class cTestB : public cTestA
		{
		public:  
			cTestB () {} 
			virtual ~cTestB () {}
			virtual void PrintMe () 
				{	GetLog() <<"     Hallo! I am a cTestB object!\n"; }
		};

			// This is NOT inherited from cTestA, it is completely unrelated
		class cTestC : public ChShared
		{
		public:  
			cTestC ()  {}
			virtual ~cTestC () {}
			virtual void PrintMe () 
				{	GetLog() <<"     Hallo! I am a cTestC object!\n"; }
		};

			// Now instantiate three objects from the three classes, 
			// and manage them via shared pointers, as learned in previous examples:
		ChSharedPtr<cTestA> pA(new cTestA);
		ChSharedPtr<cTestB> pB(new cTestB);
		ChSharedPtr<cTestC> pC(new cTestC);

			// OK, no we test some casting between shared pointers.
			// Casting is expected to be similar to casting for normal C++ pointers.
			// There are small differences, though, and one is the fact that compiler-type 
			// errors for wrong conversions are more verbose and difficult to decypher because
			// the ChSharedPtr is based on templates, that are expanded in the error message.

			// Test 1: convert a shared pointer between two different UNRELATED classes:
		//ChSharedPtr<cTestA> pAnw(pC);  // NO! compile-time error! "Types pointed to are unrelated.."
		//ChSharedPtr<cTestA> pAn = pC;  // NO! compile-time error! "Types pointed to are unrelated.."

			// Test 2: convert a shared pointer from a CHILD class to a PARENT class (UPCASTING):
		ChSharedPtr<cTestA> pAn(pB);    // OK! because cTestA is base class of cTestB, upcasting is automatic
		ChSharedPtr<cTestA> qAn = pB;   // OK! another way of doing the same...

			// Test 3: convert a shared pointer from a PARENT class to a CHILD class (DOWNCASTING):
		//ChSharedPtr<cTestB> pBnw(pA);  // NO! compile-time error! "Cast from base to derived requires dynamic_cast.."
		//ChSharedPtr<cTestB> qBn = pA;  // NO! compile-time error! "Cast from base to derived requires dynamic_cast.."

			// Test 3.b: convert a shared pointer from a PARENT class to a CHILD class (DOWNCASTING)
			//           is possible anyway via "dynamic casting", as in c++ pointers.
			//           Here pAn is a cTestA pointer to a cTestB obj, so it works:
			
			//            method 1: use the DynamicCastTo<>():
		ChSharedPtr<cTestB> pBn (pAn.DynamicCastTo<cTestB>());
		if (pBn)
			GetLog() << "Test: DynamicCastTo  pAn->pBn was successfull \n";

			//            method 2: use dynamic_cast_chshared<>(), similar to dynamic_cast in C++ :
		ChSharedPtr<cTestB> pBnn = dynamic_cast_chshared<cTestB>(pAn);
		if (pBnn)
			GetLog() << "Test: dynamic_cast_chshared  pAn->pBnn was successfull \n";

			// Test 3.c : note that dynamic casting might also FAIL,
			//            in case the source object is not belonging
			//            to the same class (or children classes) of destination.
			//            Here pA points to a cTestA obj, so it fails downcasting to cTestB.
		ChSharedPtr<cTestB> pBm(pA.DynamicCastTo<cTestB>());
		if (pBm.IsNull())
			GetLog() << "Test: DynamicCastTo test pA->pBm failed, pA belongs to parent class.\n";

			// Test 4: static casting. This is like static_cast<>() for C++ pointers.
			// This fast casting happens at compile time, but unlike dynamic casting it does 
			// not ensure that downcasting is correct. Alternative: use static_cast_chshared<>()
		ChSharedPtr<cTestB> pBs = pAn.StaticCastTo<cTestB>();  // OK downcasting. Correctness is up to you.
		ChSharedPtr<cTestA> pAs = pB .StaticCastTo<cTestA>();  // OK upcasting. But superfluous. Doing ... = pB; was enough.
		//ChSharedPtr<cTestB> pBz = pC.StaticCastTo<cTestB>(); // NO! compile-time error! "Types pointed to are unrelated.."

		pA ->PrintMe(); // will print that he references a cTestA object 
		pAn->PrintMe(); // will print that he references a cTestB object 
		pBn->PrintMe(); // will print that he references a cTestB object 
		pAs->PrintMe(); // will print that he references a cTestB object
	}


	{
		// EXAMPLE 7: 
		//
		//  Same as Example 6, but using the ChSmartPtr, i.e. non-intrusive shared pointers:

		GetLog() << "\n Example: casting between ChSmartPtr pointers \n";

			// Define a hierarchy of classes (without having ChShared as super base: here we use ChSmartPtr)
			// Note the 'virtual'! (otherwise dynamic casting won't work)
		class cTestA
		{
		public:  
			cTestA () {} 
			virtual ~cTestA () {}
			virtual void PrintMe () 
				{	GetLog() <<"     Hallo! I am a cTestA object!\n"; }
		};

			// This is inherited from cTestA
		class cTestB : public cTestA
		{
		public:  
			cTestB () {} 
			virtual ~cTestB () {}
			virtual void PrintMe () 
				{	GetLog() <<"     Hallo! I am a cTestB object!\n"; }
		};

			// This is NOT inherited from cTestA, it is completely unrelated
		class cTestC : public ChShared
		{
		public:  
			cTestC () {} 
			virtual ~cTestC () {}
			virtual void PrintMe () 
				{	GetLog() <<"     Hallo! I am a cTestC object!\n"; }
		};

		ChSmartPtr<cTestA> pA(new cTestA);
		ChSmartPtr<cTestB> pB(new cTestB);
		ChSmartPtr<cTestC> pC(new cTestC);

			// Test 1: convert a shared pointer between two different UNRELATED classes:
		//ChSmartPtr<cTestA> pAn(pC);  // NO! compile-time error! "Types pointed to are unrelated.."
		//ChSmartPtr<cTestA> pAn = pC; // NO! compile-time error! "Types pointed to are unrelated.."

			// Test 2: convert a shared pointer from a CHILD class to a PARENT class (UPCASTING):
		ChSmartPtr<cTestA> pAn(pB);    // OK! because cTestA is base class of cTestB, upcasting is automatic
		ChSmartPtr<cTestA> qAn = pB;   // OK! another way of doing the same...

			// Test 3: convert a shared pointer from a PARENT class to a CHILD class (DOWNCASTING):
		//ChSmartPtr<cTestB> pBnk(pA);  // NO! compile-time error! "Cast from base to derived requires dynamic_cast.."
		//ChSmartPtr<cTestB> qBn = pA;  // NO! compile-time error! "Cast from base to derived requires dynamic_cast.."

		ChSmartPtr<cTestB> pBn (pAn.DynamicCastTo<cTestB>());
		if (pBn)
			GetLog() << "Test: DynamicCastTo  pAn->pBn was successfull \n";

		ChSmartPtr<cTestB> pBnn = dynamic_cast_chshared<cTestB>(pAn);
		if (pBnn)
			GetLog() << "Test: dynamic_cast_chshared  pAn->pBnn was successfull \n";

		ChSmartPtr<cTestB> pBm(pA.DynamicCastTo<cTestB>());
		if (pBm.IsNull())
			GetLog() << "Test: DynamicCastTo test pA->pBm failed, pA belongs to parent class.\n";

			// Test 4: static casting. This is like static_cast<>() for C++ pointers.
			// This fast casting happens at compile time, but unlike dynamic casting it does 
			// not ensure that downcasting is correct.  Alternative: use static_cast_chshared<>()
		ChSmartPtr<cTestB> pBs = pAn.StaticCastTo<cTestB>();  // OK downcasting. Correctness is up to you.
		ChSmartPtr<cTestA> pAs = pB .StaticCastTo<cTestA>();  // OK upcasting. But superfluous.
		//ChSmartPtr<cTestB> pBz = pC.StaticCastTo<cTestB>(); // NO! compile-time error! "Types pointed to are unrelated.."
		

		pA ->PrintMe(); // will print that he references a cTestA object 
		pAn->PrintMe(); // will print that he references a cTestB object 
		pBn->PrintMe(); // will print that he references a cTestB object 
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
	//  A with a normal 'raw', weak pointer. It would be a mistake to 
	//  to point 'back' from B to A with a second shared pointer
	//  becuase it would mean circular dependancy.




	return 0;
}


