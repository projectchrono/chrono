%{
#include "chrono/utils/ChUtilsSamplers.h"

using namespace chrono::utils;
%}

%shared_ptr(chrono::utils::ChSampler<double>)
%shared_ptr(chrono::utils::ChGridSampler<double>)

%apply std::vector<chrono::ChVector3<double>> { chrono::utils::ChSampler<double>::PointVector };

%include "../../../chrono/utils/ChUtilsSamplers.h"

%template(ChSamplerd) chrono::utils::ChSampler<double>;
%template(ChGridSamplerd) chrono::utils::ChGridSampler<double>;
