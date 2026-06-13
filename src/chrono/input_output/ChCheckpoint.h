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
// Definition of base class for a Chrono checkpoint database.
//
// =============================================================================

#ifndef CH_CHECKPOINT_H
#define CH_CHECKPOINT_H

#include <vector>
#include <string>

#include "chrono/core/ChApiCE.h"
#include "chrono/utils/ChUtils.h"

#include "chrono/physics/ChSystem.h"

namespace chrono {

/// @addtogroup chrono_io
/// @{

/// Base class for a Chrono checkpoint database.
/// A Chrono checkpoint DB can be of SYSTEM type (in which case it contains all states associated with a given system)
/// or of COMPONENT tpe (in which case it contains states for specific subsets of Chrono physics items). The latter
/// option is useful in checkpointing and initializing from a checkpoint sub-assemblies, for example vehicle systems.
/// When importing a checkpoint to initialize a give system (SYSTEM-type checkpoint) or a subset of physics items
/// (COMPONENT-type checkpoint), it is the caller's responsibility to ensure that the target objects (system or
/// component lists) match the number and order in the system from which the checkpoint was generated.
///
/// To extract a checkpoint and write a SYSTEM checkpoint file, call:
///   <pre>
///   cp.Save(sys);
///   cp.WriteFile(filename);
///   </pre>
/// To write a COMPONENT checkpoint file, call:
///   <pre>
///   cp.Save(time, components);
///   cp.WriteFile(filename);
///   </pre>
///
/// To read a SYSTEM checkpoint file and apply the checkpoint to a system, call:
///   <pre>
///   cp.ReadFile(filename);
///   cp.Load(sys);
///   </pre>
/// To read a COMPONENT checkpoint file, call:
///   <pre>
///   cp.ReadFile(filename);
///   cp.Load(time, components);
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

    void SetTime(double time) { m_time = time; }
    double GetTime() const { return m_time; }

    void Save(ChSystem* sys);

    void Save(double time, const ChAssembly::Components& components);

    void Save(const ChAssembly::Components& components);

    void Load(ChSystem* sys);

    void Load(double& time, ChAssembly::Components& components);

    void Load(ChAssembly::Components& components);

    /// Open the checkpoint database from the disk file with the specified name and read the checkpoint.
    virtual void ReadFile(const std::string& filename) = 0;

    /// Write the checkpoint database to a disk file with the specified name.
    virtual void WriteFile(const std::string& filename) = 0;

    // ---------------
    // Print utilities
    // ---------------

    /// Return the provided checkpoint format as a string.
    static std::string GetFormatAsString(Format format);

    /// Return the provided checkpoint type as a string.
    static std::string GetTypeAsString(Type type);

  protected:
    ChCheckpoint(Type type);

    /// Verify that the checkpoint is of type SYSTEM.
    void CheckIfSystemType() const;

    /// Verify that the checkpoint is of type COMPONENT.
    void CheckIfComponentType() const;

    // ---------------------------
    // Checkpoint export function
    // ---------------------------

    /// Save the entire state of the provided Chrono system.
    /// Only for a SYSTEM type checkpoint DB.
    virtual void SaveState(ChSystem* sys) = 0;

    /// Save states of the bodies in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void SaveBodies(const std::vector<std::shared_ptr<ChBody>>& bodies) = 0;

    /// Save states of the shafts in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void SaveShafts(const std::vector<std::shared_ptr<ChShaft>>& shafts) = 0;

    /// Save states of the joints in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    /// Note that this list should only include kinematic joints (i.e., no motors, springs, bushings).
    virtual void SaveJoints(const std::vector<std::shared_ptr<ChLink>>& joints) = 0;

    /// Save states of the shaft couples in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    /// Note that this list should only include kinematic shaft couples (i.e., no motors).
    virtual void SaveCouples(const std::vector<std::shared_ptr<ChShaftsCouple>>& couples) = 0;

    /// Save states of the linear springs (TSDAs) in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void SaveLinSprings(const std::vector<std::shared_ptr<ChLinkTSDA>>& springs) = 0;

    /// Save states of the rotational springs (RSDAs) in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void SaveRotSprings(const std::vector<std::shared_ptr<ChLinkRSDA>>& springs) = 0;

    /// Save states of the body-body loads in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void SaveBodyBodyLoads(const std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) = 0;

    /// Save states of the linear motors in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void SaveLinMotors(const std::vector<std::shared_ptr<ChLinkMotorLinear>>& motors) = 0;

    /// Save states of the rotational motors in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void SaveRotMotors(const std::vector<std::shared_ptr<ChLinkMotorRotation>>& motors) = 0;

    /// Save the specified double value.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void SaveDouble(double value) = 0;

    /// Save the specified integer value.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void SaveInteger(int value) = 0;

    /// Save the components of the specified vector of doubles.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void SaveVector(const std::vector<double>& vector) = 0;

    /// Save the components of the specified 3D vector (with double values).
    /// Only for a COMPONENT type checkpoint DB.
    virtual void SaveChVector3(const ChVector3d& vector) = 0;

    /// Save the components of the specified quaternion (with double values).
    /// Only for a COMPONENT type checkpoint DB.
    virtual void SaveChQuaternion(const ChQuaterniond& quat) = 0;

    // ---------------------------
    // Checkpoint import functions
    // ---------------------------

    /// Load the system-level state in the input checkpoint database and initialize the given system with that state.
    /// Only for a SYSTEM type checkpoint DB.
    virtual void LoadState(ChSystem* sys) = 0;

    /// Load body states from the input checkpoint database and set them to the bodies in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void LoadBodies(std::vector<std::shared_ptr<ChBody>>& bodies) = 0;

    /// Load shaft states from the input checkpoint database and set them to the shafts in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void LoadShafts(std::vector<std::shared_ptr<ChShaft>>& shafts) = 0;

    /// Load joint states from the input checkpoint database and set them to the joints in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    /// Note that this list should only include kinematic joints (i.e., no motors, springs, bushings).
    virtual void LoadJoints(std::vector<std::shared_ptr<ChLink>>& joints) = 0;

    /// Load couple states from the input checkpoint database and set them to the shaft couples in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    /// Note that this list should only include kinematic shaft couples (i.e., no motors).
    virtual void LoadCouples(std::vector<std::shared_ptr<ChShaftsCouple>>& couples) = 0;

    /// Load spring states from the input checkpoint database and set them to the TSDAs in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void LoadLinSprings(std::vector<std::shared_ptr<ChLinkTSDA>>& springs) = 0;

    /// Load spring states from the input checkpoint database and set them to the RSDAs in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void LoadRotSprings(std::vector<std::shared_ptr<ChLinkRSDA>>& springs) = 0;

    /// Load body-body load states from the input checkpoint database and set them to the loads in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void LoadBodyBodyLoads(std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) = 0;

    /// Load motor states from the input checkpoint database and set them to the linear motors in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void LoadLinMotors(std::vector<std::shared_ptr<ChLinkMotorLinear>>& motors) = 0;

    /// Load motor states from the input checkpoint database and set them to the rotational motors in the provided list.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void LoadRotMotors(std::vector<std::shared_ptr<ChLinkMotorRotation>>& motors) = 0;

    /// Load a double value and set it to the provided variable.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void LoadDouble(double& value) = 0;

    /// Load an integer value and set it to the provided variable.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void LoadInteger(int& value) = 0;

    /// Load a sequence of double values and load them in the provided vector.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void LoadVector(std::vector<double>& vector) = 0;

    /// Load components of a 3D vector from the input checkpoint database and set them to the provided vector.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void LoadChVector3(ChVector3d& vector) = 0;

    /// Load components of a quaternion from the input checkpoint database and set them to the provided ChQuaternion.
    /// Only for a COMPONENT type checkpoint DB.
    virtual void LoadChQuaternion(ChQuaterniond& quat) = 0;

    Type m_type;    ///< checkpoint database type
    double m_time;  ///< time stamp of the checkpoint
};

/// @} chrono_io

}  // end namespace chrono

#endif
