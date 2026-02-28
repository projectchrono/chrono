#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

// MULTIPLE INHERITANCE WORKAROUND

// (B) Methods of a base class that SWIG discards that *are* overriden in ChNodeXYZ

// Ensure that these functions are not marked as 'overrides' in the generated C# code.

%csmethodmodifiers chrono::ChNodeXYZ::GetDensity "public"
%csmethodmodifiers chrono::ChNodeXYZ::GetSubBlockOffset "public"
%csmethodmodifiers chrono::ChNodeXYZ::GetNumSubBlocks "public"
%csmethodmodifiers chrono::ChNodeXYZ::IsSubBlockActive "public"

//// RADU:  Do we actually want to wrap methods of ChLoadable?
////        If not, we should probably just use %ignore

#endif             // --------------------------------------------------------------------- CSHARP

// Include the C++ header(s)
%{
#include "chrono/physics/ChNodeXYZ.h"
%}
 
%shared_ptr(chrono::ChNodeXYZ)

// Parse the header file to generate wrappers
%include "../../../chrono/physics/ChNodeXYZ.h"  
