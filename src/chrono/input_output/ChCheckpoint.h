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

/// @addtogroup chrono_io
/// @{

/// Base class for a Chrono checkpoint database.
/// A Chrono checkpoint DB can be of SYSTEM type (in which case it contains all states associated with a given system)
/// or of COMPONENT tpe (in which case it contains states for specific subsets of Chrono phsics items). The latter
/// option is useful in checkpointing and initializing from a checkpoint sub-assemblies, for example vehicle systems.
/// When importing a checkpoint to initialize a give system (SYSTEM-type checkpoint) or a subset of physics items
/// (COMPONENT-type checkpoint), it is the caller's responsibility to ensure that the target objects (system or
/// component lists) match the number and order in the system from which the checkpoint was generated.
///
/// To write a SYSTEM checkpoint file, call:
///   <pre>
///   WriteState(...);
///   WriteFile(...);
///   </pre>
/// To write a COMPONENT checkpoint file, call:
///   <pre>
///   WriteTime(...);
///   WriteBodies(...);
///   WriteShafts(...);
///   ...
///   WriteFile(...);
///   </pre>
///
/// To read a SYSTEM checkpoint file, call:
///   <pre>
///   OpenFile(...);
///   ReadState(...);
///   </pre>
/// To read a COMPONENT checkpoint file, call:
///   <pre>
///   OpenFile(...);
///   ReadTime(...);
///   ReadBodies(...);
///   ReadShafts(...);
///   ...
///   </pre>
class ChApi ChCheckpoint {
  public:
    /// Checkpoint output format.
    /// The only option currently supported is ASCII.
    enum class Format {
        ASCII  ///< ASCII text
    };

    /// Checkpoint type.
    enum class Type {
        SYSTEM,    ///< system state vector
        COMPONENT  ///< component-wise states
    };

    virtual ~ChCheckpoint() {}

    /// Initialize the checkpoint DB.
    virtual void Initialize() = 0;

    // ---------------------------
    // Checkpoint export function
    // ---------------------------

    /// Write the entire state of the provided Chrono system.
    /// Only for a SYSTEM type checkpoint DB.
    virtual void WriteState(ChSystem* sys) = 0;

    /// Write the checkpoint time.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void WriteTime(double time) = 0;

    /// Write states of the bodies in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void WriteBodies(const std::vector<std::shared_ptr<ChBody>>& bodies) = 0;

    /// Write states of the shafts in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void WriteShafts(const std::vector<std::shared_ptr<ChShaft>>& shafts) = 0;

    /// Write states of the joints in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    /// Note that this list should only include kinematic joints (i.e., no motors, springs, bushings).
    virtual void WriteJoints(const std::vector<std::shared_ptr<ChLink>>& joints) = 0;

    /// Write states of the shaft couples in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    /// Note that this list should only include kinematic shaft couples (i.e., no motors).
    virtual void WriteCouples(const std::vector<std::shared_ptr<ChShaftsCouple>>& couples) = 0;

    /// Write states of the linear springs (TSDAs) in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void WriteLinSprings(const std::vector<std::shared_ptr<ChLinkTSDA>>& springs) = 0;

    /// Write states of the rotational springs (RSDAs) in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void WriteRotSprings(const std::vector<std::shared_ptr<ChLinkRSDA>>& springs) = 0;

    /// Write states of the body-body loads in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void WriteBodyBodyLoads(const std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) = 0;

    /// Write states of the linear motors in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void WriteLinMotors(const std::vector<std::shared_ptr<ChLinkMotorLinear>>& motors) = 0;

    /// Write states of the rotational motors in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void WriteRotMotors(const std::vector<std::shared_ptr<ChLinkMotorRotation>>& motors) = 0;

    /// Write the checkpoint database to a disk file with the specified name.
    virtual void WriteFile(const std::string& filename) = 0;

    // ---------------------------
    // Checkpoint import functions
    // ---------------------------

    /// Open the checkpoint database from the disk file with the specified name.
    virtual void OpenFile(const std::string& filename) = 0;

    /// Read the system-level state in the input checkpoint database and initialize the given system with that state.
    /// Only for a SYSTEM type checkpoint DB.
    virtual void ReadState(ChSystem* sys) = 0;

    /// Read the checkpoint time from the input checkpoint database.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void ReadTime(double& time) = 0;

    /// Read body states from the input checkpoint database and set them to the bodies in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void ReadBodies(std::vector<std::shared_ptr<ChBody>>& bodies) = 0;

    /// Read shaft states from the input checkpoint database and set them to the shafts in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void ReadShafts(std::vector<std::shared_ptr<ChShaft>>& shafts) = 0;

    /// Read joint states from the input checkpoint database and set them to the joints in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    /// Note that this list should only include kinematic joints (i.e., no motors, springs, bushings).
    virtual void ReadJoints(std::vector<std::shared_ptr<ChLink>>& joints) = 0;

    /// Read couple states from the input checkpoint database and set them to the shaft couples in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    /// Note that this list should only include kinematic shaft couples (i.e., no motors).
    virtual void ReadCouples(std::vector<std::shared_ptr<ChShaftsCouple>>& couples) = 0;

    /// Read spring states from the input checkpoint database and set them to the TSDAs in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void ReadLinSprings(std::vector<std::shared_ptr<ChLinkTSDA>>& springs) = 0;

    /// Read spring states from the input checkpoint database and set them to the RSDAs in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void ReadRotSprings(std::vector<std::shared_ptr<ChLinkRSDA>>& springs) = 0;

    /// Read body-body load states from the input checkpoint database and set them to the loads in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void ReadBodyBodyLoads(std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) = 0;

    /// Read motor states from the input checkpoint database and set them to the linear motors in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void ReadLinMotors(std::vector<std::shared_ptr<ChLinkMotorLinear>>& motors) = 0;

    /// Read motor states from the input checkpoint database and set them to the rotational motors in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void ReadRotMotors(std::vector<std::shared_ptr<ChLinkMotorRotation>>& motors) = 0;

    // ---------------
    // Print utilities
    // ---------------

    /// Return the provided checkpoint format as a string.
    static std::string GetFormatAsString(Format format) {
        switch (format) {
            case Format::ASCII:
                return "ASCII";
        }
        return "";
    }

    /// Return the provided checkpoint type as a string.
    static std::string GetTypeAsString(Type type) {
        switch (type) {
            case Type::SYSTEM:
                return "SYSTEM";
            case Type::COMPONENT:
                return "COMPONENT";
        }
        return "";
    }

  protected:
    ChCheckpoint(Type type) : m_type(type) {}

    /// Verify that the checkpoint is of type SYSTEM.
    void CheckIfSystemType() const {
        if (m_type != Type::SYSTEM) {
            std::cerr << "Error: Invalid function call; not a SYSTEM checkpoint" << std::endl;
            throw std::runtime_error("Invalid function call; not a SYSTEM checkpoint");
        }
    }

    /// Verify that the checkpoint is of type COMPONENT.
    void CheckIfComponentType() const {
        if (m_type != Type::COMPONENT) {
            std::cerr << "Error: Invalid function call; not a COMPONENT checkpoint" << std::endl;
            throw std::runtime_error("Invalid function call; not a COMPONENT checkpoint");
        }
    }

    Type m_type;  ///< checkpoint database type
};

/// @} chrono_io

}  // end namespace chrono

#endif
