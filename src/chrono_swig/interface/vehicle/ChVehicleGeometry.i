%{
#include "chrono/geometry/ChGeometry.h"
#include "chrono_vehicle/ChVehicleGeometry.h"
%}


%shared_ptr(chrono::vehicle::ChVehicleGeometry)

%include "../../../chrono/geometry/ChGeometry.h"
%include "../../../chrono_vehicle/ChVehicleGeometry.h"

#ifdef SWIGCSHARP
    // SWIG has trouble wrapping these to csharp if not explicitly templated
    %template(vector_ChContactMaterialData) std::vector<chrono::ChContactMaterialData>;
    %template(vector_BoxShape) std::vector<chrono::vehicle::ChVehicleGeometry::BoxShape>;
    %template(vector_SphereShape) std::vector<chrono::vehicle::ChVehicleGeometry::SphereShape>;
    %template(vector_CylinderShape) std::vector<chrono::vehicle::ChVehicleGeometry::CylinderShape>;
    %template(vector_ConvexHullsShape) std::vector<chrono::vehicle::ChVehicleGeometry::ConvexHullsShape>;
    %template(vector_TrimeshShape) std::vector<chrono::vehicle::ChVehicleGeometry::TrimeshShape>;
    %template(vector_LineShape) std::vector<chrono::vehicle::ChVehicleGeometry::LineShape>;
#endif