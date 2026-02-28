%{

/* Includes the header in the wrapper code */
#include "chrono/fea/ChElementBeam.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChElementBeamTaperedTimoshenko.h"
#include "chrono/fea/ChElementBeamANCF_3243.h"
#include "chrono/fea/ChElementBeamANCF_3333.h"
#include "chrono/fea/ChElementBeamIGA.h"

using namespace chrono;

%}

%shared_ptr(chrono::fea::ChElementBeam)
%shared_ptr(chrono::fea::ChElementBeamEuler)
%shared_ptr(chrono::fea::ChElementBeamANCF_3243)
%shared_ptr(chrono::fea::ChElementBeamANCF_3333)
%shared_ptr(chrono::fea::ChElementBeamIGA)
%shared_ptr(chrono::fea::ChElementBeamTaperedTimoshenko)

/* Parse the header file to generate wrappers */
%include "../../../chrono/fea/ChElementBeam.h"    
%include "../../../chrono/fea/ChElementBeamEuler.h"
%include "../../../chrono/fea/ChElementBeamTaperedTimoshenko.h"
%feature("notabstract") chrono::fea::ChElementBeamANCF_3243;
%include "../../../chrono/fea/ChElementBeamANCF_3243.h"
%include "../../../chrono/fea/ChElementBeamANCF_3333.h"
%include "../../../chrono/fea/ChElementBeamIGA.h"
