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

#include "chrono/input_output/ChCheckpoint.h"
#include "chrono/input_output/ChWriterCSV.h"

namespace chrono {

/// @addtogroup chrono_io
/// @{

/// ASCII text Chrono checkpoint database.
class ChApi ChCheckpointASCII : public ChCheckpoint {
  public:
    ChCheckpointASCII(Type type);
    ~ChCheckpointASCII();

    virtual void Initialize() override;

    // Checkpoint export functions

    virtual void WriteState(ChSystem* sys) override;

    virtual void WriteTime(double time) override;
    virtual void WriteBodies(const std::vector<std::shared_ptr<ChBody>>& bodies) override;
    virtual void WriteShafts(const std::vector<std::shared_ptr<ChShaft>>& shafts) override;
    virtual void WriteJoints(const std::vector<std::shared_ptr<ChLink>>& joints) override;
    virtual void WriteCouples(const std::vector<std::shared_ptr<ChShaftsCouple>>& couples) override;
    virtual void WriteLinSprings(const std::vector<std::shared_ptr<ChLinkTSDA>>& springs) override;
    virtual void WriteRotSprings(const std::vector<std::shared_ptr<ChLinkRSDA>>& springs) override;
    virtual void WriteBodyBodyLoads(const std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) override;
    virtual void WriteLinMotors(const std::vector<std::shared_ptr<ChLinkMotorLinear>>& motors) override;
    virtual void WriteRotMotors(const std::vector<std::shared_ptr<ChLinkMotorRotation>>& motors) override;

    virtual void WriteFile(const std::string& filename) override;

    // Checkpoint import functions

    virtual void OpenFile(const std::string& filename) override;

    virtual void ReadState(ChSystem* sys) override;

    virtual void ReadTime(double& time) override;
    virtual void ReadBodies(std::vector<std::shared_ptr<ChBody>>& bodies) override;
    virtual void ReadShafts(std::vector<std::shared_ptr<ChShaft>>& shafts) override;
    virtual void ReadJoints(std::vector<std::shared_ptr<ChLink>>& joints) override;
    virtual void ReadCouples(std::vector<std::shared_ptr<ChShaftsCouple>>& couples) override;
    virtual void ReadLinSprings(std::vector<std::shared_ptr<ChLinkTSDA>>& springs) override;
    virtual void ReadRotSprings(std::vector<std::shared_ptr<ChLinkRSDA>>& springs) override;
    virtual void ReadBodyBodyLoads(std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) override;
    virtual void ReadLinMotors(std::vector<std::shared_ptr<ChLinkMotorLinear>>& motors) override;
    virtual void ReadRotMotors(std::vector<std::shared_ptr<ChLinkMotorRotation>>& motors) override;

  private:
    void CheckIfOpen() const;

    ChWriterCSV m_csv;      ///< output checkpoint CSV
    std::ifstream m_ifile;  ///< input checkpoint file

    size_t m_np;  ///< number of position-level states
    size_t m_nv;  ///< number of velocity-level states
};

/// @} chrono_io

}  // end namespace chrono

#endif
