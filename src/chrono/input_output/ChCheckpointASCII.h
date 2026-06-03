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

    virtual void ReadFile(const std::string& filename) override;
    virtual void WriteFile(const std::string& filename) override;

  private:
    // Checkpoint export functions

    virtual void SaveState(ChSystem* sys) override;

    virtual void SaveBodies(const std::vector<std::shared_ptr<ChBody>>& bodies) override;
    virtual void SaveShafts(const std::vector<std::shared_ptr<ChShaft>>& shafts) override;
    virtual void SaveJoints(const std::vector<std::shared_ptr<ChLink>>& joints) override;
    virtual void SaveCouples(const std::vector<std::shared_ptr<ChShaftsCouple>>& couples) override;
    virtual void SaveLinSprings(const std::vector<std::shared_ptr<ChLinkTSDA>>& springs) override;
    virtual void SaveRotSprings(const std::vector<std::shared_ptr<ChLinkRSDA>>& springs) override;
    virtual void SaveBodyBodyLoads(const std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) override;
    virtual void SaveLinMotors(const std::vector<std::shared_ptr<ChLinkMotorLinear>>& motors) override;
    virtual void SaveRotMotors(const std::vector<std::shared_ptr<ChLinkMotorRotation>>& motors) override;

    virtual void SaveDouble(double value) override;
    virtual void SaveInteger(int value) override;
    virtual void SaveVector(const std::vector<double>& vector) override;
    virtual void SaveChVector3(const ChVector3d& vector) override;
    virtual void SaveChQuaternion(const ChQuaterniond& quat) override;

    // Checkpoint import functions

    virtual void LoadState(ChSystem* sys) override;

    virtual void LoadBodies(std::vector<std::shared_ptr<ChBody>>& bodies) override;
    virtual void LoadShafts(std::vector<std::shared_ptr<ChShaft>>& shafts) override;
    virtual void LoadJoints(std::vector<std::shared_ptr<ChLink>>& joints) override;
    virtual void LoadCouples(std::vector<std::shared_ptr<ChShaftsCouple>>& couples) override;
    virtual void LoadLinSprings(std::vector<std::shared_ptr<ChLinkTSDA>>& springs) override;
    virtual void LoadRotSprings(std::vector<std::shared_ptr<ChLinkRSDA>>& springs) override;
    virtual void LoadBodyBodyLoads(std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) override;
    virtual void LoadLinMotors(std::vector<std::shared_ptr<ChLinkMotorLinear>>& motors) override;
    virtual void LoadRotMotors(std::vector<std::shared_ptr<ChLinkMotorRotation>>& motors) override;

    virtual void LoadDouble(double& value) override;
    virtual void LoadInteger(int& value) override;
    virtual void LoadVector(std::vector<double>& vector) override;
    virtual void LoadChVector3(ChVector3d& vector) override;
    virtual void LoadChQuaternion(ChQuaterniond& quat) override;

    void CheckIfOpen() const;

    ChWriterCSV m_csv;             ///< output checkpoint CSV
    bool m_file_read;              ///< true if the checkpoint was read from a file
    std::istringstream m_istream;  ///< checkpoint data read from file

    size_t m_np;  ///< number of position-level states
    size_t m_nv;  ///< number of velocity-level states
};

/// @} chrono_io

}  // end namespace chrono

#endif
