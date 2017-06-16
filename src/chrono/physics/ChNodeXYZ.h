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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHNODEXYZ_H
#define CHNODEXYZ_H

#include "chrono/physics/ChLoadable.h"
#include "chrono/physics/ChNodeBase.h"
#include "chrono/solver/ChVariablesBodyOwnMass.h"
#include "chrono/solver/ChVariablesNode.h"

namespace chrono {

/// Class for a single 'point' node, that has 3 DOF degrees of freedom and a mass.

class ChApi ChNodeXYZ : public virtual ChNodeBase, public ChLoadableUVW {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChNodeXYZ)

  public:
    ChNodeXYZ();
    ChNodeXYZ(const ChVector<>& initial_pos);
    ChNodeXYZ(const ChNodeXYZ& other);
    virtual ~ChNodeXYZ() {}

    ChNodeXYZ& operator=(const ChNodeXYZ& other);

    //
    // FUNCTIONS
    //

    // Access the xyz 'variables' of the node
    virtual ChVariablesNode& Variables() = 0;

    // Position of the node - in absolute csys.
    const ChVector<>& GetPos() const { return pos; }
    // Position of the node - in absolute csys.
    void SetPos(const ChVector<>& mpos) { pos = mpos; }

    // Velocity of the node - in absolute csys.
    const ChVector<>& GetPos_dt() const { return pos_dt; }
    // Velocity of the node - in absolute csys.
    void SetPos_dt(const ChVector<>& mposdt) { pos_dt = mposdt; }

    // Acceleration of the node - in absolute csys.
    const ChVector<>& GetPos_dtdt() const { return pos_dtdt; }
    // Acceleration of the node - in absolute csys.
    void SetPos_dtdt(const ChVector<>& mposdtdt) { pos_dtdt = mposdtdt; }

    // Get mass of the node. To be implemented in children classes
    virtual double GetMass() const = 0;
    // Set mass of the node. To be implemented in children classes
    virtual void SetMass(double mm) = 0;

    /// Get the number of degrees of freedom
    virtual int Get_ndof_x() const override { return 3; }

    //
    // INTERFACE to ChLoadable
    //

    /// Gets the number of DOFs affected by this element (position part)
    virtual int LoadableGet_ndof_x() override { return 3; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual int LoadableGet_ndof_w() override { return 3; }

    /// Gets all the DOFs packed in a single vector (position part)
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) override {
        mD.PasteVector(pos, block_offset, 0);
    }

    /// Gets all the DOFs packed in a single vector (speed part)
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) override {
        mD.PasteVector(pos_dt, block_offset, 0);
    }

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x, const unsigned int off_v, const ChStateDelta& Dv) override {
        NodeIntStateIncrement(off_x, x_new, x, off_v, Dv);
    }

    /// Number of coordinates in the interpolated field, ex=3 for a
    /// tetrahedron finite element or a cable, etc. Here is 6: xyz displ + xyz rots
    virtual int Get_field_ncoords() override { return 3; }

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() override { return 1; }

    /// Get the offset of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockOffset(int nblock) override { return NodeGetOffset_w(); }

    /// Get the size of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockSize(int nblock) override { return 3; }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override { mvars.push_back(&Variables()); };

    /// Evaluate Q=N'*F , for Q generalized lagrangian load, where N is some type of matrix
    /// evaluated at point P(U,V,W) assumed in absolute coordinates, and
    /// F is a load assumed in absolute coordinates.
    /// The det[J] is unused.
    virtual void ComputeNF(
        const double U,              ///< x coordinate of application point in absolute space
        const double V,              ///< y coordinate of application point in absolute space
        const double W,              ///< z coordinate of application point in absolute space
        ChVectorDynamic<>& Qi,       ///< Return result of N'*F  here, maybe with offset block_offset
        double& detJ,                ///< Return det[J] here
        const ChVectorDynamic<>& F,  ///< Input F vector, size is 3, it is Force x,y,z in absolute coords.
        ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
        ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
        ) override;

    /// This is not needed because not used in quadrature.
    virtual double GetDensity() override { return 1; }

    // SERIALIZATION

    virtual void ArchiveOUT(ChArchiveOut& marchive) override;
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

    //
    // DATA
    //

    ChVector<> pos;
    ChVector<> pos_dt;
    ChVector<> pos_dtdt;
};

CH_CLASS_VERSION(ChNodeXYZ,0)

}  // end namespace chrono

#endif
