// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// Implementation of FSI system using an SPH fluid solver.
//
// =============================================================================

#ifndef CH_FSI_SYSTEM_SPH_H
#define CH_FSI_SYSTEM_SPH_H

#include "chrono_fsi/ChFsiSystem.h"

#include "chrono_fsi/sph/ChFsiFluidSystemSPH.h"

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsisph
/// @{

/// FSI system using an SPH-based fluid solver.
class CH_FSI_API ChFsiSystemSPH : public ChFsiSystem {
  public:
    /// Construct an FSI system to couple the specified Chrono MBS system and SPH fluid system.
    /// Data exchange between the tow systems is done through an FSI interface object, owned by the FSI system.
    /// If 'use_generic_interface = true', the FSI system will use a generic FSI interface. Otherwise (default), use a
    /// custom FSI interface which works directly with the data manager of the SPH fluid solver and thus circumvents
    /// additional data movement.
    ChFsiSystemSPH(ChSystem* sysMBS, ChFsiFluidSystemSPH* sysSPH, bool use_generic_interface = false);
    ~ChFsiSystemSPH();

    /// Access the associated SPH fluid system.
    ChFsiFluidSystemSPH& GetFluidSystemSPH() const;

    // Allow using the AddRigidBody method from parent class
    using ChFsiSystem::AddRigidBody;

    /// Add a rigid body to the FSI system with given set of BCE markers.
    /// BCE marker points are assumed to be specified in the given frame (itself relative to the given body).
    std::shared_ptr<FsiBody> AddRigidBody(std::shared_ptr<ChBody> body, const std::vector<ChVector3d>& bce, const ChFrame<>& rel_frame, bool check_embedded);

    /// Set the active domain for the body with specified index (as returned by AddRigidBody).
    /// By default, this is an inverted AABB.
    /// This setting is used only for CRM problems and ignored for CFD problems.
    void SetActiveDomainBody(size_t i, const ChAABB& aabb);

    /// Set the active domain for the nodes of the 1D mesh with specified index (as returned by AddFeaMesh1D).
    /// By default, this is an inverted AABB.
    /// This setting is used only for CRM problems and ignored for CFD problems.
    void SetActiveDomainMesh1D(size_t i, const ChAABB& aabb);

    /// Set the active domain for the nodes of the 2D mesh with specified index (as returned by AddFeaMesh2D).
    /// By default, this is an inverted AABB.
    /// This setting is used only for CRM problems and ignored for CFD problems.
    void SetActiveDomainMesh2D(size_t i, const ChAABB& aabb);

    /// Set the active domain for all FSI solids (bodies and nodes) to an AABB of given dimensions, centered at the solid origin.
    /// This active AABB is used for all solids for which an active AABB was not set explicitly.
    /// This setting is used only for CRM problems and ignored for CFD problems.
    void SetActiveDomain(const ChVector3d& box_dim);

    /// Add a set of boundary BCE markers.
    /// BCE marker points are assumed to be specified in the given frame (itself relative to the global frame).
    void AddFsiBoundary(const std::vector<ChVector3d>& bce, const ChFrame<>& rel_frame);

    /// Utility function to synchronize device.
    void SynchronizeDevice();

    /// Initialize the FSI system.
    /// A call to this function marks the completion of system construction.
    virtual void Initialize() override;

  private:
    ChFsiFluidSystemSPH* m_sysSPH;  ///< cached SPH fluid solver
    bool m_generic_fsi_interface;
};

/// @} fsisph

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
