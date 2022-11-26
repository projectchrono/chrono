// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
// Face of a hexahedron element
// =============================================================================

#ifndef CH_HEXAHEDRON_FACE_H
#define CH_HEXAHEDRON_FACE_H

#include "chrono/fea/ChElementHexahedron.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Face of a hexahedron-shaped element.
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
class ChApi ChHexahedronFace : public ChLoadableUV {
  public:
    /// Construct the specified face (0 <= id <= 5) on the given hexahedral element.
    ChHexahedronFace(std::shared_ptr<ChElementHexahedron> element, char id) : m_face_id(id), m_element(element) {}

    ~ChHexahedronFace() {}

    // Get the specified face node (0 <= i <= 3).
    std::shared_ptr<ChNodeFEAxyz> GetNodeN(int i) const {
        static int iface0[] = {0, 3, 2, 1};
        static int iface1[] = {0, 1, 5, 4};
        static int iface2[] = {1, 2, 6, 5};
        static int iface3[] = {2, 3, 7, 6};
        static int iface4[] = {3, 0, 4, 7};
        static int iface5[] = {4, 5, 6, 7};

        switch (m_face_id) {
            case 0:
                return m_element->GetHexahedronNode(iface0[i]);
            case 1:
                return m_element->GetHexahedronNode(iface1[i]);
            case 2:
                return m_element->GetHexahedronNode(iface2[i]);
            case 3:
                return m_element->GetHexahedronNode(iface3[i]);
            case 4:
                return m_element->GetHexahedronNode(iface4[i]);
            case 5:
                return m_element->GetHexahedronNode(iface5[i]);
        }
        return nullptr;
    }

    /// Fills the N shape function vector (size 4).
    void ShapeFunctions(ChVectorN<double, 4>& N, double x, double y) {
        N(0) = 0.25 * (1 - x) * (1 - y);
        N(1) = 0.25 * (1 + x) * (1 - y);
        N(2) = 0.25 * (1 + x) * (1 + y);
        N(3) = 0.25 * (1 - x) * (1 + y);
    }

    // Functions for ChLoadable interface

    /// Get the number of DOFs affected by this element (position part).
    virtual int LoadableGet_ndof_x() override { return 4 * 3; }

    /// Get the number of DOFs affected by this element (speed part).
    virtual int LoadableGet_ndof_w() override { return 4 * 3; }

    /// Get all the DOFs packed in a single vector (position part).
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) override {
        mD.segment(block_offset + 0, 3) = GetNodeN(0)->GetPos().eigen();
        mD.segment(block_offset + 3, 3) = GetNodeN(1)->GetPos().eigen();
        mD.segment(block_offset + 6, 3) = GetNodeN(2)->GetPos().eigen();
        mD.segment(block_offset + 9, 3) = GetNodeN(3)->GetPos().eigen();
    }

    /// Get all the DOFs packed in a single vector (speed part).
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) override {
        mD.segment(block_offset + 0, 3) = GetNodeN(0)->GetPos_dt().eigen();
        mD.segment(block_offset + 3, 3) = GetNodeN(1)->GetPos_dt().eigen();
        mD.segment(block_offset + 6, 3) = GetNodeN(2)->GetPos_dt().eigen();
        mD.segment(block_offset + 9, 3) = GetNodeN(3)->GetPos_dt().eigen();
    }

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x,
                                        ChState& x_new,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        const ChStateDelta& Dv) override {
        for (int i = 0; i < 4; ++i) {
            GetNodeN(i)->NodeIntStateIncrement(off_x + i * 3, x_new, x, off_v + i * 3, Dv);
        }
    }

    /// Number of coordinates in the interpolated field: here the {x,y,z} displacement.
    virtual int Get_field_ncoords() override { return 3; }

    /// Get the number of DOFs sub-blocks.
    virtual int GetSubBlocks() override { return 4; }

    /// Get the offset of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(int nblock) override { return GetNodeN(nblock)->NodeGetOffsetW(); }

    /// Get the size of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(int nblock) override { return 3; }

    /// Check if the specified sub-block of DOFs is active.
    virtual bool IsSubBlockActive(int nblock) const override { return !GetNodeN(nblock)->IsFixed(); }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override {
        for (int i = 0; i < 4; ++i)
            mvars.push_back(&GetNodeN(i)->Variables());
    };

    /// Evaluate N'*F , where N is some type of shape function evaluated at U,V coordinates of the surface, each ranging
    /// in -1..+1 F is a load, N'*F is the resulting generalized load. Returns also det[J] with J=[dx/du,..], which may
    /// be useful in Gauss quadrature.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in surface
                           const double V,              ///< parametric coordinate in surface
                           ChVectorDynamic<>& Qi,       ///< result of N'*F, maybe with offset block_offset
                           double& detJ,                ///< det[J]
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is = n.field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) override {
        ChVectorN<double, 4> N;
        ShapeFunctions(N, U, V);

        //***TODO*** exact det of jacobian at u,v
        detJ = ((GetNodeN(0)->GetPos() - GetNodeN(1)->GetPos()) - (GetNodeN(2)->GetPos() - GetNodeN(3)->GetPos()))
                   .Length() *
               ((GetNodeN(1)->GetPos() - GetNodeN(2)->GetPos()) - (GetNodeN(3)->GetPos() - GetNodeN(0)->GetPos()))
                   .Length();
        // (approximate detJ, ok only for rectangular face)

        for (int i = 0; i < 4; i++) {
            Qi.segment(3 * i, 3) = N(i) * F.segment(0, 3);
        }
    }

    /// If true, use quadrature over u,v in [0..1] range as triangle volumetric coords.
    virtual bool IsTriangleIntegrationNeeded() override { return false; }

    /// Get the normal to the surface at the parametric coordinate u,v.
    /// Normal must be considered pointing outside in case the surface is a boundary to a volume.
    virtual ChVector<> ComputeNormal(const double U, const double V) override {
        ChVector<> p0 = GetNodeN(0)->GetPos();
        ChVector<> p1 = GetNodeN(1)->GetPos();
        ChVector<> p2 = GetNodeN(2)->GetPos();
        return Vcross(p1 - p0, p2 - p0).GetNormalized();
    }

  private:
    char m_face_id;                                  ///< id of the face on the hexahedron
    std::shared_ptr<ChElementHexahedron> m_element;  ///< associated hexahedron element
};

/// @} fea_elements

}  // namespace fea
}  // namespace chrono

#endif
