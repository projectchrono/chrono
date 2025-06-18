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
// Authors: Radu Serban
// =============================================================================
// Face of a LDPM_ element
// =============================================================================

#ifndef CH_LDPM_FACE_H
#define CH_LDPM_FACE_H

#include "chrono_ldpm/ChElementLDPM.h"

namespace chrono {
namespace ldpm {

/// @addtogroup ldpm_elements
/// @{

/// Face of a LDPM_-shaped element.
/// The face is identified by the number of the vertex to which it is opposed: 0,1,2,3.
/// Corner nodes, obtainable with GetNodeN(), are in counterclockwise order seen from the outside.
class ChApi ChLDPMFace : public ChLoadableUV {
  public:
    /// Construct the specified face (0 <= id <= 3) on the given tetrahedral element.
    ChLDPMFace(std::shared_ptr<ChElementLDPM> element, char id) : m_face_id(id), m_element(element) {}

    ~ChLDPMFace() {}

     // Get the specified face node (0 <= i <= 2).
    std::shared_ptr<ChNodeFEAxyzrot> GetNode(unsigned int i) const {
        static int iface0[] = {2, 1, 3};
        static int iface1[] = {3, 0, 2};
        static int iface2[] = {3, 1, 0};
        static int iface3[] = {0, 1, 2};

        switch (m_face_id) {
            case 0:
                return m_element->GetTetrahedronNode(iface0[i]);
            case 1:
                return m_element->GetTetrahedronNode(iface1[i]);
            case 2:
                return m_element->GetTetrahedronNode(iface2[i]);
            case 3:
                return m_element->GetTetrahedronNode(iface3[i]);
        }
        return nullptr;
    }

    /// Fills the N shape function vector (size 3) with the values of shape functions at r,s 'area' coordinates, all
    /// ranging in [0...1].
    void ShapeFunctions(ChVectorN<double, 3>& N, double r, double s) {
        N(0) = 1.0 - r - s;
        N(1) = r;
        N(2) = s;
    };

    // Functions for ChLoadable interface

    /// Get the number of DOFs affected by this element (position part).
    virtual unsigned int GetLoadableNumCoordsPosLevel() override { return 3 * 3; }

    /// Get the number of DOFs affected by this element (speed part).
    virtual unsigned int GetLoadableNumCoordsVelLevel() override { return 3 * 3; }


     /// Get all the DOFs packed in a single vector (position part).
    virtual void LoadableGetStateBlockPosLevel(int block_offset, ChState& mD) override {
        mD.segment(block_offset + 0, 3) = GetNode(0)->GetPos().eigen();
        mD.segment(block_offset + 3, 3) = GetNode(1)->GetPos().eigen();
        mD.segment(block_offset + 6, 3) = GetNode(2)->GetPos().eigen();
    }

    /// Get all the DOFs packed in a single vector (speed part).
    virtual void LoadableGetStateBlockVelLevel(int block_offset, ChStateDelta& mD) override {
        mD.segment(block_offset + 0, 3) = GetNode(0)->GetPosDt().eigen();
        mD.segment(block_offset + 3, 3) = GetNode(1)->GetPosDt().eigen();
        mD.segment(block_offset + 6, 3) = GetNode(2)->GetPosDt().eigen();
    }

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x,
                                        ChState& x_new,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        const ChStateDelta& Dv) override {
        for (int i = 0; i < 3; ++i) {
            GetNode(i)->NodeIntStateIncrement(off_x + i * 3, x_new, x, off_v + i * 3, Dv);
        }
    }

    /// Number of coordinates in the interpolated field: here the {x,y,z} displacement.
    virtual unsigned int GetNumFieldCoords() override { return 3; }

    /// Get the number of DOFs sub-blocks.
    virtual unsigned int GetNumSubBlocks() override { return 3; }

    /// Get the offset of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(unsigned int nblock) override {
        return GetNode(nblock)->NodeGetOffsetVelLevel();
    }

    /// Get the size of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(unsigned int nblock) override { return 3; }

    /// Check if the specified sub-block of DOFs is active.
    virtual bool IsSubBlockActive(unsigned int nblock) const override { return !GetNode(nblock)->IsFixed(); }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override {
        for (int i = 0; i < 3; ++i)
            mvars.push_back(&GetNode(i)->Variables());
    };

    /// Evaluate N'*F , where N is some type of shape function evaluated at U,V coordinates of the surface, each ranging
    /// in 0..+1 F is a load, N'*F is the resulting generalized load. Returns also det[J] with J=[dx/du,..], which may
    /// be useful in Gauss quadrature.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in surface
                           const double V,              ///< parametric coordinate in surface
                           ChVectorDynamic<>& Qi,       ///< result of N'*F , maybe with offset block_offset
                           double& detJ,                ///< det[J]
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is = n.field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) override {
        // evaluate shape functions (in compressed vector), btw. not dependant on state
        // note: U,V in 0..1 range, thanks to IsTriangleIntegrationNeeded() {return true;}
        ChVectorN<double, 3> N;
        this->ShapeFunctions(N, U, V);

        // determinant of jacobian is also =2*areaoftriangle, also length of cross product of sides
        ChVector3d p0 = GetNode(0)->GetPos();
        ChVector3d p1 = GetNode(1)->GetPos();
        ChVector3d p2 = GetNode(2)->GetPos();
        detJ = (Vcross(p2 - p0, p1 - p0)).Length();

        Qi(0) = N(0) * F(0);
        Qi(1) = N(0) * F(1);
        Qi(2) = N(0) * F(2);
        Qi(3) = N(1) * F(0);
        Qi(4) = N(1) * F(1);
        Qi(5) = N(1) * F(2);
        Qi(6) = N(2) * F(0);
        Qi(7) = N(2) * F(1);
        Qi(8) = N(2) * F(2);
    }

    /// If true, use quadrature over u,v in [0..1] range as triangle volumetric coords.
    virtual bool IsTriangleIntegrationNeeded() override { return true; }

    /// Get the normal to the surface at the parametric coordinate u,v.
    /// Normal must be considered pointing outside in case the surface is a boundary to a volume.
    virtual ChVector3d ComputeNormal(const double U, const double V) override {
        ChVector3d p0 = GetNode(0)->GetPos();
        ChVector3d p1 = GetNode(1)->GetPos();
        ChVector3d p2 = GetNode(2)->GetPos();
        return Vcross(p1 - p0, p2 - p0).GetNormalized();
    }

  private:
    char m_face_id;                                   ///< id of the face on the tetrahedron
    std::shared_ptr<ChElementLDPM> m_element;  ///< associated tetrahedron element

};

/// @} fea_elements

}  // end namespace ldpm
}  // end namespace chrono

#endif
