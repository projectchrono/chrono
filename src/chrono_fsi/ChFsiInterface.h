// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Arman Pazouki, Milad Rakhsha
// =============================================================================
//
// Base class for processing the interface between chrono and fsi modules
// =============================================================================
#ifndef CH_FSIINTERFACE_H_
#define CH_FSIINTERFACE_H_

#include "chrono/physics/ChSystem.h"
#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/ChFsiDataManager.cuh"
#include "chrono_fsi/ChFsiGeneral.cuh"

namespace chrono
{
namespace fsi
{

/// @addtogroup fsi_physics
/// @{

/// Base class for processing the interface between chrono and fsi modules
class CH_FSI_API ChFsiInterface : public ChFsiGeneral
{
  public:
    /// Constructs an interface for fsi simulation.
    ChFsiInterface(FsiBodiesDataH *other_fsiBodiesH,        ///< States of rigid bodies in fsi simulation
                   chrono::ChSystem *other_mphysicalSystem, ///< A pointer to the ChSystem handled by the fsi system

                   std::vector<std::shared_ptr<chrono::ChBody>> *other_fsiBodeisPtr, ///< Pointer to the vector of the
                   /// ChBody shared pointers handle by the FSI system.

                   thrust::device_vector<Real3> *
                       other_rigid_FSI_ForcesD, ///< A pointer to the surface-integrated forces from the fluid
                                                /// dynamics system to rigid bodies

                   thrust::device_vector<Real3> *
                       other_rigid_FSI_TorquesD ///< A pointer to the surface-integrated torques from the fluid
                                                /// dynamics system to rigid bodies
                   );

    /// Destructor of the FSI interface.
    ~ChFsiInterface();

    /// Reads the surface-integrated pressure and viscous forces form the fluid dynamics system, and add these forces as
    /// external forces to the ChSystem bodies.
    virtual void Add_Rigid_ForceTorques_To_ChSystem();

    /// Uses an external configuration to set the generalized coordinates of the ChSystem.
    virtual void Copy_External_To_ChSystem();

    /// Uses the generalized coordinates of the ChSystem to set the configuration state in the FSI system.
    virtual void Copy_ChSystem_to_External();

    /// Copies the generalized coordinates of the rigid bodies at the position, velocity,
    /// and acceleration level to the fluid dynamics system.
    virtual void Copy_fsiBodies_ChSystem_to_FluidSystem(FsiBodiesDataD *fsiBodiesD);

    /// Resizes the data structure associated with the chrono system based on the size of the chrono system.
    virtual void ResizeChronoBodiesData();

  private:
    FsiBodiesDataH *fsiBodiesH;           ///< State of the FSI rigid bodies at the position, velocity and acceleration level.
    ChronoBodiesDataH *chronoRigidBackup; ///< A backup data structure to save the state of the chrono system
    chrono::ChSystem *mphysicalSystem;    ///< Pointer to the Chrono system handled by the FSI system.
    std::vector<std::shared_ptr<chrono::ChBody>> *
        fsiBodeisPtr; ///< Pointer to the vector of the ChBody shared pointers handled by the FSI system.

    thrust::device_vector<Real3> *
        rigid_FSI_ForcesD; ///< Surface-integrated forces from the fluid dynamics system to rigid bodies.
    thrust::device_vector<Real3> *
        rigid_FSI_TorquesD; ///< Surface-integrated torques from the fluid dynamics system to rigid bodies.
};

/// @} fsi_physics

} // end namespace fsi
} // end namespace chrono

#endif
