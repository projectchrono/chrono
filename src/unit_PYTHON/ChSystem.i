%{

/* Includes the header in the wrapper code */
#include "physics/ChSystem.h"

using namespace chrono;

%}

/* Parse the header file to generate wrappers */
%include "../physics/ChSystem.h" 

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi 


// Nested classes workaround:

class IteratorBodies
	{
    public:
      IteratorBodies(std::vector<ChBody*>::iterator p) : node_(p) {}
      IteratorBodies& operator=(const IteratorBodies& other);
      bool operator==(const IteratorBodies& other);
      bool operator!=(const IteratorBodies& other);
      IteratorBodies& operator++();
      ChSharedPtr<ChBody> operator*();
	   private:
	    std::vector<ChBody*>::iterator node_;
    };
class IteratorLinks
	{
    public:
      IteratorLinks(std::list<ChLink*>::iterator p) : node_(p) {}
      IteratorLinks& operator=(const IteratorLinks& other);
      bool operator==(const IteratorLinks& other);
      bool operator!=(const IteratorLinks& other);
      IteratorLinks& operator++();
      ChSharedPtr<ChLink> operator*();
	  IteratorLinks(){};
	   private:
	    std::list<ChLink*>::iterator node_;
    };
class IteratorOtherPhysicsItems
	{
    public:
      IteratorOtherPhysicsItems(std::list<ChPhysicsItem*>::iterator p) : node_(p) {}
      IteratorOtherPhysicsItems& operator=(const IteratorOtherPhysicsItems& other);
      bool operator==(const IteratorOtherPhysicsItems& other);
      bool operator!=(const IteratorOtherPhysicsItems& other);
      IteratorOtherPhysicsItems& operator++();
      ChSharedPtr<ChPhysicsItem> operator*();
	  IteratorOtherPhysicsItems(){};
	   private:
	    std::list<ChPhysicsItem*>::iterator node_;
    };

%{
// Trick for having a SWIG-specific global class that represent
// a nested class (nested are not supported in SWIG so this is a workaround)
typedef chrono::ChSystem::IteratorBodies IteratorBodies;
typedef chrono::ChSystem::IteratorLinks  IteratorLinks;
typedef chrono::ChSystem::IteratorOtherPhysicsItems IteratorOtherPhysicsItems;

%}

