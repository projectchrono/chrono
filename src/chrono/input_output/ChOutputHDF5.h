// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
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
// Definition of an HDF5 Chrono output database.
//
// =============================================================================

#ifndef CH_OUTPUT_HDF5_H
#define CH_OUTPUT_HDF5_H

#include <string>
#include <fstream>

#include <H5Cpp.h>

#include "chrono/input_output/ChOutput.h"

namespace chrono {

class ChOutputHDF5_impl;

/// @addtogroup chrono_io
/// @{

/// HDF5 Chrono output database.
class ChApi ChOutputHDF5 : public ChOutput {
  public:
    /// Create an output DB in HDF5 format and associate it with an output file.
    /// Note: the output file name will be `<out_dir>/<out_file_stem>.<mode>.h5`.
    ChOutputHDF5(const std::string& out_dir, const std::string& out_file_stem, Mode mode);

    ~ChOutputHDF5();

  private:
    // Implementation of functions for Mode::SERIES
    void WriteBuffers();

  private:
    // Implementation of virtual functions for Mode::FRAMES
    virtual void WriteTimeStamp(int frame, double time) override;
    virtual void WriteBodies(const std::vector<std::shared_ptr<ChBody>>& bodies) override;
    virtual void WriteMarkers(const std::vector<std::shared_ptr<ChMarker>>& markers) override;
    virtual void WriteShafts(const std::vector<std::shared_ptr<ChShaft>>& shafts) override;
    virtual void WriteJoints(const std::vector<std::shared_ptr<ChLink>>& joints) override;
    virtual void WriteCouples(const std::vector<std::shared_ptr<ChShaftsCouple>>& couples) override;
    virtual void WriteLinSprings(const std::vector<std::shared_ptr<ChLinkTSDA>>& springs) override;
    virtual void WriteRotSprings(const std::vector<std::shared_ptr<ChLinkRSDA>>& springs) override;
    virtual void WriteBodyBodyLoads(const std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) override;
    virtual void WriteLinMotors(const std::vector<std::shared_ptr<ChLinkMotorLinear>>& motors) override;
    virtual void WriteRotMotors(const std::vector<std::shared_ptr<ChLinkMotorRotation>>& motors) override;

    H5::H5File* m_fileHDF5;
    H5::Group* m_frame_group;

    H5::CompType* m_body_type;
    H5::CompType* m_shaft_type;
    H5::CompType* m_marker_type;
    H5::CompType* m_joint_type;
    H5::CompType* m_couple_type;
    H5::CompType* m_linspring_type;
    H5::CompType* m_rotspring_type;
    H5::CompType* m_bodyload_type;
    H5::CompType* m_linmotor_type;
    H5::CompType* m_rotmotor_type;
};

/// @} chrono_io

}  // end namespace chrono

#endif
