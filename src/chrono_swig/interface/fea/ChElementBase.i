%{

/* Includes the header in the wrapper code */
#include "chrono/fea/ChElementBase.h"
#include "chrono/fea/ChElementGeneric.h"
#include "chrono/fea/ChElementBar.h"
#include "chrono/fea/ChElementSpring.h"

using namespace chrono;

%}

#ifdef SWIGCSHARP

%ignore chrono::fea::ChElementBase;
%interface_impl(chrono::fea::ChElementGeneric);

#endif

%shared_ptr(chrono::fea::ChElementBase)
%shared_ptr(chrono::fea::ChElementGeneric)
%shared_ptr(chrono::fea::ChElementSpring)
%shared_ptr(chrono::fea::ChElementBar)

/* Parse the header file to generate wrappers */
// TODO: if eigen::ref can be wrapped, unignore these
%ignore chrono::fea::ChElementBase::ComputeKRMmatricesGlobal;
%ignore chrono::fea::ChElementBase::ComputeMmatrixGlobal;
%include "../../../chrono/fea/ChElementBase.h"
%include "../../../chrono/fea/ChElementGeneric.h"
%include "../../../chrono/fea/ChElementBar.h"
%include "../../../chrono/fea/ChElementSpring.h"
