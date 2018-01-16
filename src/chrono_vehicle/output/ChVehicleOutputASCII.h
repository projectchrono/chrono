// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// ASCII text vehicle output database.
//
// =============================================================================

#ifndef CH_VEHICLE_OUTPUT_ASCII_H
#define CH_VEHICLE_OUTPUT_ASCII_H

#include <string>
#include <fstream>

#include "chrono_vehicle/ChVehicleOutput.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle
/// @{

/// ASCII text vehicle output database.
class CH_VEHICLE_API ChVehicleOutputASCII : public ChVehicleOutput {
  public:
    ChVehicleOutputASCII(const std::string& filename);
    ~ChVehicleOutputASCII();

  private:
    virtual void WriteTime(double time) override;
    virtual void WriteSection(const std::string& name) override;
    virtual void WriteBody(std::shared_ptr<ChBody> body) override;
    virtual void WriteBodyAuxRef(std::shared_ptr<ChBodyAuxRef> body) override;
    virtual void WriteMarker(std::shared_ptr<ChMarker> marker) override;
    virtual void WriteShaft(std::shared_ptr<ChShaft> shaft) override;
    virtual void WriteJoint(std::shared_ptr<ChLink> joint) override;
    virtual void WriteLinSpring(std::shared_ptr<ChLinkSpringCB> spring) override;
    virtual void WriteRotSpring(std::shared_ptr<ChLinkRotSpringCB> spring) override;

    std::ofstream m_stream;
};

template <typename T>
inline std::ostream& operator<<(std::ostream& out, const ChVector<T>& v) {
    out << v.x() << " " << v.y() << " " << v.z();
    return out;
}

template <typename T>
inline std::ostream& operator<<(std::ostream& out, const ChQuaternion<T>& q) {
    out << q.e0() << " " << q.e1() << " " << q.e2() << " " << q.e3();
    return out;
}

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
