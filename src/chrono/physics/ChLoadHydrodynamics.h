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

#ifndef CH_LOAD_HYDRODYNAMICS_H
#define CH_LOAD_HYDRODYNAMICS_H

#include <unordered_map>

#include "chrono/core/ChApiCE.h"

#include "chrono/physics/ChPhysicsItem.h"
#include <chrono/physics/ChBody.h>
#include "chrono/solver/ChSystemDescriptor.h"

namespace chrono {

// -----------------------------------------------------------------------------

/// Added mass blocks for a set of ChBody.
/// The block associated with each body must have size 6 x 6n, where n is the number of all "hydrodynamic" bodies
/// (i.e., the size of the map). For each body in the set, its n associated 6x6 blocks must follow the same order
/// as the bodies in the set.
using ChBodyAddedMassBlocks = std::unordered_map<std::shared_ptr<ChBody>, ChMatrixDynamic<>>;

/// Added mass for hydrodynamic loads.
/// The added mass is an inertia added to the system due to accelerating bodies having to displace some volume of
/// surrounding fluid. The additional system-wide mass matrix term is a block-sparse matrix with non-zero blocks
/// corresponding to all bodies that interact with fluid. For each of these `n` bodies (not necessarily all bodies
/// included in the Chrono system), a dense 6 x 6n block of hydrodynamic coefficients must be provided (see
/// ChBodyAddedMassBlocks). The 6 x 6 blocks for each of these bodies is distributed at the appropriate location in
/// the system mass matrix.
class ChApi ChLoadHydrodynamics : public ChPhysicsItem {
  public:
    ChLoadHydrodynamics(const ChBodyAddedMassBlocks& body_blocks);
    ChLoadHydrodynamics(const ChLoadHydrodynamics& other);
    ~ChLoadHydrodynamics();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadHydrodynamics* Clone() const override { return new ChLoadHydrodynamics(*this); }

    /// Enable/disable verbose terminal output (default: false).
    void SetVerbose(bool verbose) { m_verbose = verbose; }

    /// Perform setup operations at the beginning of a step.
    virtual void Setup() override {}

    /// Perform any updates necessary at the current phase during the solution process.
    /// This function is called at least once per step to update auxiliary data, internal states, etc.
    /// If a problem size is detected, resize the underlying KRM block.
    virtual void Update(double time, bool update_assets) override;

    /// Increment the given residual vector R with the term c * M * w (for the entire system).
    virtual void IntLoadResidual_Mv(const unsigned int off,      ///< offset in R residual
                                    ChVectorDynamic<>& R,        ///< updated residual R += c*M*v
                                    const ChVectorDynamic<>& w,  ///< w vector
                                    const double c               ///< scaling factor
                                    ) override;

    /// Register with the given system descriptor any ChMBlock objects associated with this item.
    virtual void InjectKRMMatrices(ChSystemDescriptor& descriptor) override;

    /// Compute and load current mass matrices in encapsulated ChMBlock objects.
    /// The resulting M block contains Mfactor * M (Kfactor and Rfactor are ignored here).
    virtual void LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) override;

  protected:
    bool m_verbose;
    ChBodyAddedMassBlocks m_body_blocks;  ///< added mass blocks for hydrodynamic bodies
    ChMatrixDynamic<> m_added_mass;       ///< added mass matrix (system size)
    ChKRMBlock m_KRM;                     ///< scaled added mass matrix (system size)
};

}  // end namespace chrono

#endif
