%{
#include "chrono/physics/ChObject.h"
using namespace chrono;
%}

%shared_ptr(chrono::ChObj)

%ignore chrono::ChObj::Clone;
%ignore chrono::ChObj::ArchiveContainerName;

%include "../../../chrono/physics/ChObject.h"    

