%{
#include "chrono/geometry/ChGeometry.h"
#include "chrono/utils/ChBodyGeometry.h"
%}


%shared_ptr(chrono::utils::ChBodyGeometry)

%include "../../../chrono/utils/ChBodyGeometry.h"

%template(vector_BoxShape) std::vector<chrono::utils::ChBodyGeometry::BoxShape>;
%template(vector_SphereShape) std::vector<chrono::utils::ChBodyGeometry::SphereShape>;
%template(vector_CylinderShape) std::vector<chrono::utils::ChBodyGeometry::CylinderShape>;
%template(vector_ConvexHullsShape) std::vector<chrono::utils::ChBodyGeometry::ConvexHullsShape>;
%template(vector_TrimeshShape) std::vector<chrono::utils::ChBodyGeometry::TrimeshShape>;
%template(vector_LineShape) std::vector<chrono::utils::ChBodyGeometry::LineShape>;
