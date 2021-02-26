%{

/* Includes the header in the wrapper code */
#include "chrono/motion_functions/ChFunction_Base.h"
#include "chrono/motion_functions/ChFunction_Const.h"
#include "chrono/motion_functions/ChFunction_ConstAcc.h"
#include "chrono/motion_functions/ChFunction_Derive.h"
#include "chrono/motion_functions/ChFunction_Fillet3.h"
#include "chrono/motion_functions/ChFunction_Integrate.h"
#include "chrono/motion_functions/ChFunction_Mirror.h"
#include "chrono/motion_functions/ChFunction_Mocap.h"
#include "chrono/motion_functions/ChFunction_Noise.h"
#include "chrono/motion_functions/ChFunction_Operation.h"
#include "chrono/motion_functions/ChFunction_Oscilloscope.h"
#include "chrono/motion_functions/ChFunction_Poly.h"
#include "chrono/motion_functions/ChFunction_Poly345.h"
#include "chrono/motion_functions/ChFunction_Ramp.h"
#include "chrono/motion_functions/ChFunction_Recorder.h"
#include "chrono/motion_functions/ChFunction_Repeat.h"
#include "chrono/motion_functions/ChFunction_Sequence.h"
#include "chrono/motion_functions/ChFunction_Sigma.h"
#include "chrono/motion_functions/ChFunction_Sine.h"
#include "chrono/motion_functions/ChFunction_Setpoint.h"

#include "chrono/motion_functions/ChFunctionRotation.h"
#include "chrono/motion_functions/ChFunctionRotation_axis.h"
#include "chrono/motion_functions/ChFunctionRotation_ABCfunctions.h"
#include "chrono/motion_functions/ChFunctionRotation_setpoint.h"
#include "chrono/motion_functions/ChFunctionRotation_spline.h"
#include "chrono/motion_functions/ChFunctionRotation_SQUAD.h"
#include "chrono/motion_functions/ChFunctionPosition.h"
#include "chrono/motion_functions/ChFunctionPosition_line.h"
#include "chrono/motion_functions/ChFunctionPosition_setpoint.h"
#include "chrono/motion_functions/ChFunctionPosition_XYZfunctions.h"

%}


// Cross-inheritance between Python and c++ for callbacks that must be inherited.
// Put this 'director' feature _before_ class wrapping declaration.
%feature("director") chrono::ChFunction;
%ignore chrono::ChFunction::Clone;

/* Parse the header file to generate wrappers */
%include "../../chrono/motion_functions/ChFunction_Base.h"  
%include "../../chrono/motion_functions/ChFunction_Const.h"
%include "../../chrono/motion_functions/ChFunction_ConstAcc.h"
%include "../../chrono/motion_functions/ChFunction_Derive.h"
%include "../../chrono/motion_functions/ChFunction_Fillet3.h"
%include "../../chrono/motion_functions/ChFunction_Integrate.h"
%include "../../chrono/motion_functions/ChFunction_Mirror.h"
%include "../../chrono/motion_functions/ChFunction_Mocap.h"
%include "../../chrono/motion_functions/ChFunction_Noise.h"
%include "../../chrono/motion_functions/ChFunction_Operation.h"
%include "../../chrono/motion_functions/ChFunction_Oscilloscope.h"
%include "../../chrono/motion_functions/ChFunction_Poly.h"
%include "../../chrono/motion_functions/ChFunction_Poly345.h"
%include "../../chrono/motion_functions/ChFunction_Ramp.h"
%include "../../chrono/motion_functions/ChFunction_Recorder.h"
%include "../../chrono/motion_functions/ChFunction_Repeat.h"
%include "../../chrono/motion_functions/ChFunction_Sequence.h"
%include "../../chrono/motion_functions/ChFunction_Sigma.h"
%include "../../chrono/motion_functions/ChFunction_Sine.h"
%include "../../chrono/motion_functions/ChFunction_Setpoint.h"

%include "../../chrono/motion_functions/ChFunctionRotation.h"
%include "../../chrono/motion_functions/ChFunctionRotation_axis.h"
%include "../../chrono/motion_functions/ChFunctionRotation_ABCfunctions.h"
%include "../../chrono/motion_functions/ChFunctionRotation_setpoint.h"
%include "../../chrono/motion_functions/ChFunctionRotation_spline.h"
%include "../../chrono/motion_functions/ChFunctionRotation_SQUAD.h"
%include "../../chrono/motion_functions/ChFunctionPosition.h"
%include "../../chrono/motion_functions/ChFunctionPosition_line.h"
%include "../../chrono/motion_functions/ChFunctionPosition_setpoint.h"
%include "../../chrono/motion_functions/ChFunctionPosition_XYZfunctions.h"
