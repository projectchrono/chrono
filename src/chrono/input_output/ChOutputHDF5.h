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
// HDF5 Chrono output database.
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
    ChOutputHDF5(const std::string& filename, Mode mode = Mode::FRAMES);
    ~ChOutputHDF5();

  private:
    virtual void Initialize() override;
    virtual void WriteTime(int frame, double time) override;
    virtual void WriteSection(const std::string& name) override;
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
    Mode m_mode;
    std::unique_ptr<ChOutputHDF5_impl> m_impl;
    bool m_initialized;
};

/// @} chrono_io

}  // end namespace chrono

#endif
