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

#ifndef CH_VEHICLE_OUTPUT_HDF5_H
#define CH_VEHICLE_OUTPUT_HDF5_H

#include <string>
#include <fstream>

#include "chrono_vehicle/ChVehicleOutput.h"

#include "H5Cpp.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle
/// @{

/// HDF5 vehicle output database.
class CH_VEHICLE_API ChVehicleOutputHDF5 : public ChVehicleOutput {
  public:
    ChVehicleOutputHDF5(const std::string& filename);
    ~ChVehicleOutputHDF5();

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

    H5::H5File* m_fileHDF5;
    H5::Group* m_frame_group;
    H5::Group* m_section_group;

    static H5::CompType* m_body_type;
    static H5::CompType* m_bodyaux_type;
    static H5::CompType* m_shaft_type;
    static H5::CompType* m_marker_type;
    static H5::CompType* m_joint_type;
    static H5::CompType* m_couple_type;
    static H5::CompType* m_linspring_type;
    static H5::CompType* m_rotspring_type;
    static H5::CompType* m_bodyload_type;

    static const H5::CompType& getBodyType();
    static const H5::CompType& getBodyAuxType();
    static const H5::CompType& getShaftType();
    static const H5::CompType& getMarkerType();
    static const H5::CompType& getJointType();
    static const H5::CompType& getCoupleType();
    static const H5::CompType& getLinSpringType();
    static const H5::CompType& getRotSpringType();
    static const H5::CompType& getBodyLoadType();
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
