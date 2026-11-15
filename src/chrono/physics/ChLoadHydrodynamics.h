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
#include "chrono/physics/ChBody.h"
#include "chrono/solver/ChSystemDescriptor.h"

namespace chrono {

// -----------------------------------------------------------------------------

/// Added mass block for a ChBody.
/// The block associated with each body must have size 6 x 6n, where n is the number of all "hydrodynamic" bodies.
/// The coefficients refer to the velocity of the origin of the body reference frame and the body angular velocity,
/// expressed in the axes given by the ChLoadHydrodynamics frame convention (see ChLoadHydrodynamics::AddedMassFrame).
struct ChBodyAddedMassBlock {
    std::shared_ptr<ChBody> body;  ///< hydrodynamic body
    ChMatrixDynamic<> block;       ///< 6 x 6n block of added mass coefficients
};

/// A collection of 'n' added mass blocks.
/// For each body in such a vector, its 'n' associated 6x6 blocks must follow the same order as the bodies in the set.
using ChBodyAddedMassBlocks = std::vector<ChBodyAddedMassBlock>;

/// Added mass for hydrodynamic loads.
/// The added mass is an inertia added to the system due to accelerating bodies having to displace some volume of
/// surrounding fluid. The additional system-wide mass matrix term is a block-sparse matrix with non-zero blocks
/// corresponding to all bodies that interact with fluid. For each of these `n` bodies (not necessarily all bodies
/// included in the Chrono system), a dense 6 x 6n block of hydrodynamic coefficients must be provided (see
/// ChBodyAddedMassBlocks). The 6 x 6 blocks for each of these bodies is distributed at the appropriate location in
/// the system mass matrix.
///
/// The coefficients of each body refer to the velocity of the origin of the body reference frame (the COM frame for a
/// ChBody, the auxiliary reference frame for a ChBodyAuxRef) and to the body angular velocity, both expressed either in
/// the inertial (absolute) axes or in the axes of the body reference frame (see AddedMassFrame). Coefficients computed
/// about a different point must be shifted to the origin of the body reference frame before they are passed to this
/// load (see ComputeShiftTransformation). Since Chrono uses the COM velocity in absolute axes and the angular velocity
/// in the axes of the COM frame as velocity coordinates, the coefficients are transformed accordingly, using the
/// current orientation of the bodies. The added mass is therefore constant in the selected axes and, in general, varies with the body
/// orientation in the Chrono velocity coordinates. The velocity-dependent terms arising from the time variation of the
/// added mass (e.g., the Munk moment) are not included, which is consistent with the small-rotation assumptions under
/// which hydrodynamic coefficients are typically computed.
///
/// Note that, with a solver that needs the inverse of the total mass matrix (APGD or Barzilai-Borwein), that inverse is
/// recomputed whenever a hydrodynamic body rotates, which requires the inversion of a dense system-size matrix.
class ChApi ChLoadHydrodynamics : public ChPhysicsItem {
  public:
    /// Axes in which the added mass coefficients are expressed (and in which they are constant).
    enum class AddedMassFrame {
        INERTIAL,  ///< inertial (absolute) axes, e.g. coefficients from a BEM code (default)
        BODY       ///< axes of the body reference frame, i.e. coefficients that rotate with the body
    };

    ChLoadHydrodynamics(const ChBodyAddedMassBlocks& body_blocks);
    ChLoadHydrodynamics(const ChLoadHydrodynamics& other);
    ~ChLoadHydrodynamics();

    /// Return true: the internal state of this item is restored from a checkpoint (see ChPhysicsItem).
    virtual bool SupportsCheckpoint() const override { return true; }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLoadHydrodynamics* Clone() const override { return new ChLoadHydrodynamics(*this); }

    /// Enable/disable verbose terminal output (default: false).
    void SetVerbose(bool verbose) { m_verbose = verbose; }

    /// Set the axes in which the added mass coefficients are expressed (default: AddedMassFrame::INERTIAL).
    void SetAddedMassFrame(AddedMassFrame frame);

    /// Get the axes in which the added mass coefficients are expressed.
    AddedMassFrame GetAddedMassFrame() const { return m_frame; }

    /// Modify the added mass blocks.
    /// In this case, each given block is assumed to have size 6 x 6n and is expressed in the current frame convention.
    /// If any block changes, the inverse of the total mass matrix (if needed by the solver) is recomputed.
    void SetBodyAddedMassBlocks(const std::vector<ChMatrixDynamic<>>& blocks);

    /// Modify the added mass block.
    /// In this case, each given block is assumed to have size 6 x 6, is expressed in the current frame convention, and
    /// overwrites the diagonal block of the added mass matrix.
    /// If any block changes, the inverse of the total mass matrix (if needed by the solver) is recomputed.
    void UpdateBodyAddedMassBlocks(const std::vector<ChMatrix66d>& blocks);

    /// Compute the transformation that shifts added mass coefficients to the origin of the body reference frame.
    /// The coefficients A are given about a point fixed on the body, with the specified location in the body reference
    /// frame, and are expressed in the axes of the given frame convention. If S_i is the transformation for body i, the
    /// coefficients about the origins of the body reference frames are S_i^T * A_ij * S_j. With the inertial
    /// convention, S depends on the body orientation: the transformation computed at the current orientation applies
    /// exactly only at that orientation.
    static ChMatrix66d ComputeShiftTransformation(const ChBody& body, const ChVector3d& point, AddedMassFrame frame);

    /// Shift added mass blocks with the given transformations (see ComputeShiftTransformation).
    /// Each block, of size 6 x 6n, is replaced in place with the blocks S_i^T * A_ij * S_j.
    static void ShiftBlocks(std::vector<ChMatrixDynamic<>>& blocks, const std::vector<ChMatrix66d>& shifts);

    /// Perform setup operations at the beginning of a step.
    virtual void Setup() override {}

    /// Perform any updates necessary at the current phase during the solution process.
    /// This function is called at least once per step to update auxiliary data, internal states, etc.
    /// If a problem size is detected, resize the underlying KRM block.
    virtual void Update(double time, UpdateFlags update_flags) override;

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
    /// Return true if the current solver needs the inverse of the total mass matrix.
    bool NeedsMassInverse() const;

    /// Recompute the mass inverse right away if the blocks were modified and a mass inverse is in use.
    void OnBlocksModified();

    /// Assemble the total mass matrix (including added mass), invert it, and pass it to the system descriptor.
    void ComputeMassInverse();

    /// Return true if any hydrodynamic body rotated since the mass inverse was last computed.
    bool BodiesRotated() const;

    /// Compute the transformation from Chrono velocity coordinates to the velocities the coefficients refer to.
    /// For body i, the velocity of the origin of its reference frame and its angular velocity (in the axes given by the
    /// frame convention) are T_i * [v; w], where v is the COM velocity in absolute axes and w the angular velocity in
    /// the axes of the COM frame.
    ChMatrix66d ComputeTransformation(const ChBodyAddedMassBlock& b) const;

    /// Compute the added mass matrix in Chrono velocity coordinates at the current body configuration.
    /// The block coupling bodies i and j is T_i^T * A_ij * T_j, where A_ij is the corresponding 6 x 6 block of
    /// coefficients.
    void ComputeAddedMassMatrix();

    bool m_verbose;
    AddedMassFrame m_frame;                          ///< axes in which the added mass coefficients are expressed
    ChBodyAddedMassBlocks m_body_blocks;             ///< added mass blocks for hydrodynamic bodies
    ChMatrixDynamic<> m_M;                           ///< added mass matrix in Chrono velocity coordinates (in block order)
    ChKRMBlock m_KRM;                                ///< scaled added mass matrix (6*num_bodies square, in block order)
    unsigned int m_system_size;                      ///< system size for which the mass inverse was last computed
    bool m_blocks_modified;                          ///< added mass blocks changed since the mass inverse was last computed
    std::vector<ChQuaterniond> m_inverse_rotations;  ///< body orientations for which the mass inverse was last computed
};

}  // end namespace chrono

#endif
