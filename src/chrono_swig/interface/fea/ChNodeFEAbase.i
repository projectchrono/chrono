%{

/* Includes the header in the wrapper code */
#include "chrono/fea/ChNodeFEAbase.h"

using namespace chrono;

%}

#ifdef SWIGCSHARP

%interface_impl(chrono::fea::ChNodeFEAbase);

#endif

%shared_ptr(chrono::fea::ChNodeFEAbase)

/* Parse the header file to generate wrappers */
// TODO: if eigen::ref can be wrapped, unignore these,
%ignore chrono::fea::ChNodeFEAbase::ComputeKRMmatricesGlobal;
%include "../../../chrono/fea/ChNodeFEAbase.h"
%include "../../../chrono/fea/ChNodeFEAxyz.h"
%include "../../../chrono/fea/ChNodeFEAxyzP.h"
%include "../../../chrono/fea/ChNodeFEAxyzD.h"
%include "../../../chrono/fea/ChNodeFEAxyzDD.h"
%include "../../../chrono/fea/ChNodeFEAxyzDDD.h"
%include "../../../chrono/fea/ChNodeFEAxyzrot.h"
