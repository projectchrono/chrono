%{

/* Includes the header in the wrapper code */
#include "physics/ChSystem.h"

using namespace chrono;

// NESTED CLASSES - trick - step 1
//
// Trick for having a SWIG-specific global class that represent
// a nested class (nested are not supported in SWIG so this is a workaround)
// The workaround is done in various 1,2,3,.. steps. 
//
// STEP 1: do aliases for the c++ compiler when compiling the .cxx wrapper 
typedef chrono::ChSystem::IteratorBodies IteratorBodies;
typedef chrono::ChSystem::IteratorLinks  IteratorLinks;
typedef chrono::ChSystem::IteratorOtherPhysicsItems IteratorOtherPhysicsItems;
typedef chrono::ChSystem::IteratorPhysicsItems IteratorPhysicsItems;
//typedef chrono::ChSystem::ChCustomCollisionPointCallback ChCustomCollisionPointCallback;
//typedef chrono::ChSystem::ChCustomComputeCollisionCallback ChCustomComputeCollisionCallback;

// for these two other nested classes it is enough to inherit stubs (not virtual) as outside classes
class ChCustomCollisionPointCallbackP : public chrono::ChSystem::ChCustomCollisionPointCallback
{
	public:	
		ChCustomCollisionPointCallbackP() {};
		virtual void ContactCallback(
							const chrono::collision::ChCollisionInfo& mcontactinfo,				
							chrono::ChMaterialCouple&  material 			  		
							) { GetLog() << "You must implement ContactCallback() ! \n"; };
};
class ChCustomComputeCollisionCallbackP  : public chrono::ChSystem::ChCustomComputeCollisionCallback
{
	public: 
		ChCustomComputeCollisionCallbackP() {};
		virtual void PerformCustomCollision(chrono::ChSystem* msys) { GetLog() << "You must implement PerformCustomCollision() ! \n"; };
};


%}



// Forward ref
%import "ChCollisionModel.i"
%import "ChCollisionInfo.i"


// Cross-inheritance between Python and c++ for callbacks that must be inherited.
// Put this 'director' feature _before_ class wrapping declaration.
%feature("director") ChCustomCollisionPointCallbackP;
%feature("director") ChCustomComputeCollisionCallbackP;


// NESTED CLASSES - trick - step 2
//
// STEP 2: Now the nested classes  MyOutClass::MyNestedClass are declared  
// as ::MyNestedClass (as non-nested), for SWIG interpreter _only_:

class IteratorBodies
	{
    public:
      bool operator==(const IteratorBodies& other);
      bool operator!=(const IteratorBodies& other);
    };
class IteratorLinks
	{
    public:
      bool operator==(const IteratorLinks& other);
      bool operator!=(const IteratorLinks& other);
    };
class IteratorOtherPhysicsItems
	{
    public:
      bool operator==(const IteratorOtherPhysicsItems& other);
      bool operator!=(const IteratorOtherPhysicsItems& other);
    };
class IteratorPhysicsItems
	{
    public:
      bool operator==(const IteratorPhysicsItems& other);
      bool operator!=(const IteratorPhysicsItems& other);
    };
class ChCustomCollisionPointCallbackP
	{
	public:	
	  virtual void ContactCallback(
							const chrono::collision::ChCollisionInfo& mcontactinfo, 
							chrono::ChMaterialCouple&  material 			  		
													);
	};
class ChCustomComputeCollisionCallbackP
{
	public: 
		virtual void PerformCustomCollision(chrono::ChSystem* msys) {};
};

// NESTED CLASSES - trick - step 3
//
// STEP 3: if needed, extend some functions of the 'un-nested' classes, here
// is the way to do it:

%extend IteratorBodies{
	  IteratorBodies Next()
	  {
			IteratorBodies mynext(*$self);
			++mynext;
			return mynext;
	  }
	  chrono::ChSharedPtr<ChBody> Ref()   // note the chrono:: namespace
	  {
			return $self->operator*();
	  }
};
%extend IteratorLinks{
	  IteratorLinks Next()
	  {
			IteratorLinks mynext(*$self);
			++mynext;
			return mynext;
	  }
	  chrono::ChSharedPtr<ChLink> Ref()   // note the chrono:: namespace
	  {
			return $self->operator*();
	  }
};
%extend IteratorOtherPhysicsItems{
	  IteratorOtherPhysicsItems Next()
	  {
			IteratorOtherPhysicsItems mynext(*$self);
			++mynext;
			return mynext;
	  }
	  chrono::ChSharedPtr<ChPhysicsItem> Ref()   // note the chrono:: namespace
	  {
			return $self->operator*();
	  }
};
%extend IteratorPhysicsItems{
	  IteratorPhysicsItems Next()
	  {
			IteratorPhysicsItems mynext(*$self);
			++mynext;
			return mynext;
	  }
	  chrono::ChSharedPtr<ChPhysicsItem> Ref()   // note the chrono:: namespace
	  {
			return $self->operator*();
	  }
};


// NESTED CLASSES - trick - step 4
//
// STEP 4: if needed, extend some functions of the class that hosted the nested classes,
// because for example they returned a refrence to nested chrono::ChSystem::IteratorBody and
// this will confuse the python runtime, so do override these functions so that they return 
// a ::IteratorBody type (see step 2), that will be interpreted ok in the python runtime.

%extend chrono::ChSystem
{
	::IteratorBodies IterBeginBodies()   // note the :: at the beginning, otherwise SWIG (unsuccesfully) tries to use the nested one
	  {
			return $self->IterBeginBodies();
	  }
	::IteratorBodies IterEndBodies()   // note the :: at the beginning, otherwise SWIG (unsuccesfully) tries to use the nested one
	  {
			return $self->IterEndBodies();
	  }
	::IteratorLinks IterBeginLinks()   // note the :: at the beginning, otherwise SWIG (unsuccesfully) tries to use the nested one
	  {
			return $self->IterBeginLinks();
	  }
	::IteratorLinks IterEndLinks()   // note the :: at the beginning, otherwise SWIG (unsuccesfully) tries to use the nested one
	  {
			return $self->IterEndLinks();
	  }
	::IteratorOtherPhysicsItems IterBeginOtherPhysicsItems()   // note the :: at the beginning, otherwise SWIG (unsuccesfully) tries to use the nested one
	  {
			return $self->IterBeginOtherPhysicsItems();
	  }
	::IteratorOtherPhysicsItems IterEndOtherPhysicsItems()   // note the :: at the beginning, otherwise SWIG (unsuccesfully) tries to use the nested one
	  {
			return $self->IterEndOtherPhysicsItems();
	  }
	::IteratorPhysicsItems IterBeginPhysicsItems()   // note the :: at the beginning, otherwise SWIG (unsuccesfully) tries to use the nested one
	  {
			return $self->IterBeginPhysicsItems();
	  }
	::IteratorPhysicsItems IterEndPhysicsItems()   // note the :: at the beginning, otherwise SWIG (unsuccesfully) tries to use the nested one
	  {
			return $self->IterEndPhysicsItems();
	  }
	void SetCustomCollisionPointCallback(::ChCustomCollisionPointCallbackP* mcallb)  // note the :: at the beginning
	  {
		  $self->SetCustomCollisionPointCallback(mcallb);
	  }
	void SetCustomComputeCollisionCallback(::ChCustomComputeCollisionCallbackP* mcallb)  // note the :: at the beginning
	  {
		  $self->SetCustomComputeCollisionCallback(mcallb);
	  }
};


// NESTED CLASSES - trick - step 5
//
// STEP 5: note that if you override some functions by %extend, now you must deactivate the 
// original ones in the .h file, by using %ignore. NOTE that this must happen AFTER the %extend,
// and BEFORE the %include !!!
%ignore chrono::ChSystem::IterBeginBodies();
%ignore chrono::ChSystem::IterEndBodies();
%ignore chrono::ChSystem::IterBeginLinks();
%ignore chrono::ChSystem::IterEndLinks();
%ignore chrono::ChSystem::IterBeginOtherPhysicsItems();
%ignore chrono::ChSystem::IterEndOtherPhysicsItems();
%ignore chrono::ChSystem::IterBeginPhysicsItems();
%ignore chrono::ChSystem::IterEndPhysicsItems();
%ignore chrono::ChSystem::SetCustomCollisionPointCallback();
%ignore chrono::ChSystem::SetCustomComputeCollisionCallback();



// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 

/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChSystem.h" 



//
// ADD PYTHON CODE
//
// To make iteration much easier, as
//  for abody in chrono.IterBodies(my_system):
//     print '  body pos=', abody.GetPos()

%pythoncode %{

class IterBodies():
    def __init__(self,asystem):
        self.iterbodies = asystem.IterBeginBodies()
        self.asystem = asystem
    def __iter__(self):
        return self
    def __next__(self):
        """2.6-3.x version"""
        return self.next()
    def next(self):
        if self.iterbodies == self.asystem.IterEndBodies():
            raise StopIteration
        else:
            presentiter = self.iterbodies
            self.iterbodies = self.iterbodies.Next()
            return presentiter.Ref()

class IterLinks():
    def __init__(self,asystem):
        self.iterli = asystem.IterBeginLinks()
        self.asystem = asystem
    def __iter__(self):
        return self
    def __next__(self):
        """2.6-3.x version"""
        return self.next()
    def next(self):
        if self.iterli == self.asystem.IterEndLinks():
            raise StopIteration
        else:
            presentiter = self.iterli
            self.iterli = self.iterli.Next()
            return presentiter.Ref()

class IterOtherPhysicsItems():
    def __init__(self,asystem):
        self.iterph = asystem.IterBeginOtherPhysicsItems()
        self.asystem = asystem
    def __iter__(self):
        return self
    def __next__(self):
        """2.6-3.x version"""
        return self.next()
    def next(self):
        if self.iterph == self.asystem.IterEndOtherPhysicsItems():
            raise StopIteration
        else:
            presentiter = self.iterph
            self.iterph = self.iterph.Next()
            return presentiter.Ref()

class IterPhysicsItems():
    def __init__(self,asystem):
        self.iterph = asystem.IterBeginPhysicsItems()
        self.asystem = asystem
    def __iter__(self):
        return self
    def __next__(self):
        """2.6-3.x version"""
        return self.next()
    def next(self):
        if self.iterph == self.asystem.IterEndPhysicsItems():
            raise StopIteration
        else:
            presentiter = self.iterph
            self.iterph = self.iterph.Next()
            return presentiter.Ref()

%}


// Define also the shared pointer chrono::ChShared<ChSystem> 
// (renamed as 'ChSystemShared' in python)

%DefChSharedPtr(chrono::,ChSystem)
