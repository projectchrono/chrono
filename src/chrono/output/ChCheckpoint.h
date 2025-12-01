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
// Base class for a Chrono checkpoint database.
//
// =============================================================================

#ifndef CH_CHECKPOINT_H
#define CH_CHECKPOINT_H

#include <vector>
#include <string>

#include "chrono/core/ChApiCE.h"

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChLink.h"
#include "chrono/physics/ChShaftsCouple.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChLinkRSDA.h"
#include "chrono/physics/ChLoadsBody.h"
#include "chrono/physics/ChLinkMotorLinear.h"
#include "chrono/physics/ChLinkMotorRotation.h"

namespace chrono {

/// @addtogroup chrono_output
/// @{

/// Base class for a Chrono checkpoint database.
class ChApi ChCheckpoint {
  public:
    /// Checkpoint output format. Currently supported option is ASCII.
    enum class Format {
        ASCII  ///< ASCII text
    };

    /// Checkpoint type.
    enum class Type {
        SYSTEM,    ///< system state vector
        COMPONENT  ///< component-wise states
    };

    virtual ~ChCheckpoint() {}

    virtual void Initialize() = 0;

    // Checkpoint export function

    virtual void WriteState(ChSystem* sys) = 0;

    virtual void WriteBodies(const std::vector<std::shared_ptr<ChBody>>& bodies) = 0;
    virtual void WriteShafts(const std::vector<std::shared_ptr<ChShaft>>& shafts) = 0;
    virtual void WriteJoints(const std::vector<std::shared_ptr<ChLink>>& joints) = 0;
    virtual void WriteCouples(const std::vector<std::shared_ptr<ChShaftsCouple>>& couples) = 0;
    virtual void WriteLinSprings(const std::vector<std::shared_ptr<ChLinkTSDA>>& springs) = 0;
    virtual void WriteRotSprings(const std::vector<std::shared_ptr<ChLinkRSDA>>& springs) = 0;
    virtual void WriteBodyBodyLoads(const std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) = 0;
    virtual void WriteLinMotors(const std::vector<std::shared_ptr<ChLinkMotorLinear>>& motors) = 0;
    virtual void WriteRotMotors(const std::vector<std::shared_ptr<ChLinkMotorRotation>>& motors) = 0;

    virtual void WriteFile(const std::string& filename, double time) = 0;

    // Checkpoint import functions

    virtual void OpenFile(const std::string& filename) = 0;

    virtual void ReadState(ChSystem* sys) = 0;

    // Print utilities

    static std::string GetFormatAsString(Format format) {
        switch (format) {
            case Format::ASCII:
                return "ASCII";
        }
        return "";
    }

    static std::string GetTypeAsString(Type type) {
        switch (type) {
            case Type::SYSTEM:
                return "SYYSTEM";
            case Type::COMPONENT:
                return "COMPONENT";
        }
        return "";
    }

  protected:
    ChCheckpoint(Type type) : m_type(type) {}

    /// Verify that the checkpoint is of type SYSTEM.
    void TestTypeSystem() const {
        if (m_type != Type::SYSTEM) {
            std::cerr << "Error: Invalid function call; not a SYSTEM checkpoint" << std::endl;
            throw std::runtime_error("Invalid function call; not a SYYSTEM checkpoint");
        }
    }

    /// Verify that the checkpoint is of type COMPONENT.
    void TestTypeComponent() const {
        if (m_type != Type::COMPONENT) {
            std::cerr << "Error: Invalid function call; not a COMPONENT checkpoint" << std::endl;
            throw std::runtime_error("Invalid function call; not a COMPONENT checkpoint");
        }
    }

    Type m_type;
};

/// @} chrono_output

}  // end namespace chrono

#endif
