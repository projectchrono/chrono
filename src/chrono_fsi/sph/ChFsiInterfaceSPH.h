// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Milad Rakhsha, Arman Pazouki, Wei Hu, Radu Serban
// =============================================================================
//
// Class for processing the interface between Chrono and the SPH-based FSI module
// =============================================================================

#ifndef CH_FSI_INTERFACE_SPH_H
#define CH_FSI_INTERFACE_SPH_H

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/ChFsiInterface.h"

#include "chrono_fsi/sph/physics/FsiDataManager.cuh"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// Class for processing the interface between Chrono and the SPH-based FSI module.
class ChFsiInterfaceSPH : public ChFsiInterface {
  public:
    /// Constructor of the FSI interface class.
    ChFsiInterfaceSPH(sph::FsiDataManager& data_mgr, bool verbose);

    /// Destructor of the FSI interface class.
    ~ChFsiInterfaceSPH();

    /// Copy rigid body states from ChSystem to FsiSystem, then to the GPU memory.
    virtual void LoadBodyStates() override;

    /// Copy FEA mesh states from ChSystem to FsiSystem, then to the GPU memory.
    virtual void LoadMeshStates() override;

    /// Read the surface-integrated pressure and viscous forces form the fluid/granular dynamics system,
    /// and add these forces and torques as external forces to the ChSystem rigid bodies.
    virtual void ApplyBodyForces() override;

    /// Add forces and torques as external forces to the ChSystem flexible bodies.
    virtual void ApplyMeshForces() override;

  private:

    sph::FsiDataManager& m_data_mgr;  ///< FSI data manager

    friend class ChFsiSystemSPH;
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
