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
// ASCII text Chrono checkpoint database.
//
// =============================================================================

#ifndef CH_CHECKPOINT_ASCII_H
#define CH_CHECKPOINT_ASCII_H

#include <string>
#include <fstream>

#include "chrono/output/ChCheckpoint.h"
#include "chrono/utils/ChUtilsInputOutput.h"

namespace chrono {

/// @addtogroup chrono_output
/// @{

/// ASCII text Chrono checkpoint database.
class ChApi ChCheckpointASCII : public ChCheckpoint {
  public:
    ChCheckpointASCII(Type type);
    ~ChCheckpointASCII();

    virtual void Initialize() override;

    // Checkpoint export functions

    virtual void WriteState(ChSystem* sys) override;

    virtual void WriteBodies(const std::vector<std::shared_ptr<ChBody>>& bodies) override;
    virtual void WriteShafts(const std::vector<std::shared_ptr<ChShaft>>& shafts) override;
    virtual void WriteJoints(const std::vector<std::shared_ptr<ChLink>>& joints) override;
    virtual void WriteCouples(const std::vector<std::shared_ptr<ChShaftsCouple>>& couples) override;
    virtual void WriteLinSprings(const std::vector<std::shared_ptr<ChLinkTSDA>>& springs) override;
    virtual void WriteRotSprings(const std::vector<std::shared_ptr<ChLinkRSDA>>& springs) override;
    virtual void WriteBodyBodyLoads(const std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) override;
    virtual void WriteLinMotors(const std::vector<std::shared_ptr<ChLinkMotorLinear>>& motors) override;
    virtual void WriteRotMotors(const std::vector<std::shared_ptr<ChLinkMotorRotation>>& motors) override;

    virtual void WriteFile(const std::string& filename, double time) override;

    // Checkpoint import functions

    virtual void OpenFile(const std::string& filename) override;

    virtual void ReadState(ChSystem* sys) override;

  private:
    utils::ChWriterCSV m_csv;
    std::ifstream m_ifile;

    size_t m_np;  ///< number of position-level states
    size_t m_nv;  ///< number of velocity-level states
};

/// @} chrono_output

}  // end namespace chrono

#endif
