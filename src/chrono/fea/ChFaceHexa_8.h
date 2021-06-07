// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================
// Face of a 8-node hexahedron brick element
// =============================================================================

#ifndef CHFACEHEXA8_H
#define CHFACEHEXA8_H

#include "chrono/fea/ChElementHexa_8.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Face of a linear ChElementHexa_8 hexahedron.
/// This is a proxy to the hexahedron. It can be used to apply pressure loads.
/// Corner nodes, obtainable with GetNodeN(), are in counterclockwise order seen from the outside.
/// <pre>
///         v
///         ^
/// 3 o-----+-----o 2
///   |     |     |
/// --+-----+-----+-> u
///   |     |     |
/// 0 o-----+-----o 1
/// </pre>
class ChApi ChFaceHexa_8 : public ChLoadableUV {
  protected:
    char face_id;
    std::shared_ptr<ChElementHexa_8> melement;

  public:
    ChFaceHexa_8(std::shared_ptr<ChElementHexa_8> mel, char mid) : face_id(mid), melement(mel) {}

    virtual ~ChFaceHexa_8() {}

    // Get the node 'i' of face , with i=0,1,2,...,5
    std::shared_ptr<ChNodeFEAxyz> GetNodeN(int i) {
        int iface0[] = {0, 3, 2, 1};
        int iface1[] = {0, 1, 5, 4};
        int iface2[] = {1, 2, 6, 5};
        int iface3[] = {2, 3, 7, 6};
        int iface4[] = {3, 0, 4, 7};
        int iface5[] = {4, 5, 6, 7};

        switch (face_id) {
            case 0:
                return std::dynamic_pointer_cast<ChNodeFEAxyz>(melement->GetNodeN(iface0[i]));
            case 1:
                return std::dynamic_pointer_cast<ChNodeFEAxyz>(melement->GetNodeN(iface1[i]));
            case 2:
                return std::dynamic_pointer_cast<ChNodeFEAxyz>(melement->GetNodeN(iface2[i]));
            case 3:
                return std::dynamic_pointer_cast<ChNodeFEAxyz>(melement->GetNodeN(iface3[i]));
            case 4:
                return std::dynamic_pointer_cast<ChNodeFEAxyz>(melement->GetNodeN(iface4[i]));
            case 5:
                return std::dynamic_pointer_cast<ChNodeFEAxyz>(melement->GetNodeN(iface5[i]));
        }
        std::shared_ptr<ChNodeFEAxyz> foo_null;
        return foo_null;
    }

    /// Fills the N shape function vector (size 4)
    void ShapeFunctions(ChVectorN<double, 4>& N, double x, double y) {
        N(0) = 0.25 * (1 - x) * (1 - y);
        N(1) = 0.25 * (1 + x) * (1 - y);
        N(2) = 0.25 * (1 + x) * (1 + y);
        N(3) = 0.25 * (1 - x) * (1 + y);
    }

    //
    // Functions for ChLoadable interface
    //

    /// Gets the number of DOFs affected by this element (position part)
    virtual int LoadableGet_ndof_x() override { return 4 * 3; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual int LoadableGet_ndof_w() override  { return 4 * 3; }

    /// Gets all the DOFs packed in a single vector (position part)
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) override  {
        mD.segment(block_offset + 0, 3) = this->GetNodeN(0)->GetPos().eigen();
        mD.segment(block_offset + 3, 3) = this->GetNodeN(1)->GetPos().eigen();
        mD.segment(block_offset + 6, 3) = this->GetNodeN(2)->GetPos().eigen();
        mD.segment(block_offset + 9, 3) = this->GetNodeN(3)->GetPos().eigen();
    }

    /// Gets all the DOFs packed in a single vector (speed part)
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) override  {
        mD.segment(block_offset + 0, 3) = this->GetNodeN(0)->GetPos_dt().eigen();
        mD.segment(block_offset + 3, 3) = this->GetNodeN(1)->GetPos_dt().eigen();
        mD.segment(block_offset + 6, 3) = this->GetNodeN(2)->GetPos_dt().eigen();
        mD.segment(block_offset + 9, 3) = this->GetNodeN(3)->GetPos_dt().eigen();
    }

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x, const unsigned int off_v, const ChStateDelta& Dv) override {
        for (int i=0; i<4; ++i) {
            GetNodeN(i)->NodeIntStateIncrement(off_x + i*3  , x_new, x, off_v  + i*3  , Dv);
        }
    }

    /// Number of coordinates in the interpolated field: here the {x,y,z} displacement
    virtual int Get_field_ncoords() override { return 3; }

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() override { return 4; }

    /// Get the offset of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockOffset(int nblock) override { return this->GetNodeN(nblock)->NodeGetOffset_w(); }

    /// Get the size of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockSize(int nblock) override { return 3; }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override {
        for (int i = 0; i < 4; ++i)
            mvars.push_back(&this->GetNodeN(i)->Variables());
    };

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U,V coordinates of the surface, each ranging in -1..+1
    /// F is a load, N'*F is the resulting generalized load
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in surface
                           const double V,              ///< parametric coordinate in surface
                           ChVectorDynamic<>& Qi,       ///< Return result of N'*F  here, maybe with offset block_offset
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is = n.field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) override  {
        ChVectorN<double, 4> N;
        ShapeFunctions(N, U, V);

        //***TODO*** exact det of jacobian at u,v
        detJ = ((this->GetNodeN(0)->GetPos() - this->GetNodeN(1)->GetPos()) -
                (this->GetNodeN(2)->GetPos() - this->GetNodeN(3)->GetPos()))
                   .Length() *
               ((this->GetNodeN(1)->GetPos() - this->GetNodeN(2)->GetPos()) -
                (this->GetNodeN(3)->GetPos() - this->GetNodeN(0)->GetPos()))
                   .Length();
        // (approximate detJ, ok only for rectangular face)

        for (int i = 0; i < 4; i++) {
            Qi.segment(3 * i, 3) = N(i) * F.segment(0, 3);
        }
    }

    /// If true, use quadrature over u,v in [0..1] range as triangle volumetric coords
    /// Regular quadrature used for this element
    virtual bool IsTriangleIntegrationNeeded() override { return false; }

    /// Gets the normal to the surface at the parametric coordinate u,v.
    /// Note: the average normal is returned if the face is twisted.
    /// Normal must be considered pointing outside in case the surface is a boundary to a volume.
    virtual ChVector<> ComputeNormal(const double U, const double V) override {
        //***TODO*** use shape derivatives and cross product
        ChVector<> p0 = GetNodeN(0)->GetPos();
        ChVector<> p1 = GetNodeN(1)->GetPos();
        ChVector<> p2 = GetNodeN(2)->GetPos();
        return Vcross(p1 - p0, p2 - p0).GetNormalized();
    }
};

/// @} fea_elements

}  // end namespace fea
}  // end namespace chrono

#endif
