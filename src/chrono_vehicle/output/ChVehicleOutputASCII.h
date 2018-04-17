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
    virtual void WriteTime(int frame, double time) override;
    virtual void WriteSection(const std::string& name) override;

    virtual void WriteBodies(const std::vector<std::shared_ptr<ChBody>>& bodies) override;
    virtual void WriteAuxRefBodies(const std::vector<std::shared_ptr<ChBodyAuxRef>>& bodies) override;
    virtual void WriteMarkers(const std::vector<std::shared_ptr<ChMarker>>& markers) override;
    virtual void WriteShafts(const std::vector<std::shared_ptr<ChShaft>>& shafts) override;
    virtual void WriteJoints(const std::vector<std::shared_ptr<ChLink>>& joints) override;
    virtual void WriteCouples(const std::vector<std::shared_ptr<ChShaftsCouple>>& couples) override;
    virtual void WriteLinSprings(const std::vector<std::shared_ptr<ChLinkSpringCB>>& springs) override;
    virtual void WriteRotSprings(const std::vector<std::shared_ptr<ChLinkRotSpringCB>>& springs) override;
    virtual void WriteBodyLoads(const std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) override;

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
