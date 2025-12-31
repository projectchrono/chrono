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
// ASCII text Chrono output database.
//
// =============================================================================

#ifndef CH_OUTPUT_ASCII_H
#define CH_OUTPUT_ASCII_H

#include <string>
#include <fstream>

#include "chrono/input_output/ChOutput.h"

namespace chrono {

/// @addtogroup chrono_io
/// @{

/// ASCII text Chrono output database.
class ChApi ChOutputASCII : public ChOutput {
  public:
    ChOutputASCII(const std::string& filename);
    ChOutputASCII(std::ostream& stream);
    ~ChOutputASCII();

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

    std::ostream& m_stream;
    std::ofstream m_file_stream;
    bool m_initialized;
};

/// @} chrono_io

}  // end namespace chrono

#endif
