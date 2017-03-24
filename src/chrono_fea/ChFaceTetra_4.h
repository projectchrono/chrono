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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHFACETETRA4_H
#define CHFACETETRA4_H

#include "chrono_fea/ChElementTetra_4.h"

namespace chrono {
namespace fea {

/// Face of a linear ChElementTetra_4 tetrahedron.
/// This is a proxy to the tetrahedron. It can be used to apply pressure loads.
/// Note, face_id is the number of the vertex to whom it is opposed: 0,1,2,3.
/// Corner nodes, obtainable with GetNodeN(), are in counterclockwise order seen from the outside.
class ChApiFea ChFaceTetra_4 : public ChLoadableUV {
  protected:
    char face_id;
    std::shared_ptr<ChElementTetra_4> melement;

  public:
    ChFaceTetra_4(std::shared_ptr<ChElementTetra_4> mel, char mid) : melement(mel), face_id(mid) {}

    virtual ~ChFaceTetra_4() {}

    // Get the node 'i' of face , with i=0,1,2
    std::shared_ptr<ChNodeFEAxyz> GetNodeN(int i) {
        int iface0[] = {2, 1, 3};
        int iface1[] = {3, 0, 2};
        int iface2[] = {3, 1, 0};
        int iface3[] = {0, 1, 2};
        switch (face_id) {
            case 0:
                return std::dynamic_pointer_cast<ChNodeFEAxyz>(melement->GetNodeN(iface0[i]));
            case 1:
                return std::dynamic_pointer_cast<ChNodeFEAxyz>(melement->GetNodeN(iface1[i]));
            case 2:
                return std::dynamic_pointer_cast<ChNodeFEAxyz>(melement->GetNodeN(iface2[i]));
            case 3:
                return std::dynamic_pointer_cast<ChNodeFEAxyz>(melement->GetNodeN(iface3[i]));
        }
        std::shared_ptr<ChNodeFEAxyz> foo_null;
        return foo_null;
    }

    /// Fills the N shape function matrix (1 row, 3 columns) with the
    /// values of shape functions at r,s 'area' coordinates, all ranging in [0...1].
    virtual void ShapeFunctions(ChMatrix<>& N, double r, double s) {
        N(0) = 1.0 - r - s;
        N(1) = r;
        N(2) = s;
    };

    //
    // Functions for ChLoadable interface
    //

    /// Gets the number of DOFs affected by this element (position part)
    virtual int LoadableGet_ndof_x() { return 3 * 3; }

    /// Gets the number of DOFs affected by this element (speed part)
    virtual int LoadableGet_ndof_w() { return 3 * 3; }

    /// Gets all the DOFs packed in a single vector (position part)
    virtual void LoadableGetStateBlock_x(int block_offset, ChState& mD) {
        mD.PasteVector(this->GetNodeN(0)->GetPos(), block_offset, 0);
        mD.PasteVector(this->GetNodeN(1)->GetPos(), block_offset + 3, 0);
        mD.PasteVector(this->GetNodeN(2)->GetPos(), block_offset + 6, 0);
    }

    /// Gets all the DOFs packed in a single vector (speed part)
    virtual void LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) {
        mD.PasteVector(this->GetNodeN(0)->GetPos_dt(), block_offset, 0);
        mD.PasteVector(this->GetNodeN(1)->GetPos_dt(), block_offset + 3, 0);
        mD.PasteVector(this->GetNodeN(2)->GetPos_dt(), block_offset + 6, 0);
    }

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x, const unsigned int off_v, const ChStateDelta& Dv) override {
        for (int i=0; i<3; ++i) {
            GetNodeN(i)->NodeIntStateIncrement(off_x + i*3  , x_new, x, off_v  + i*3  , Dv);
        }
    }

    /// Number of coordinates in the interpolated field: here the {x,y,z} displacement
    virtual int Get_field_ncoords() { return 3; }

    /// Tell the number of DOFs blocks (ex. =1 for a body, =4 for a tetrahedron, etc.)
    virtual int GetSubBlocks() { return 3; }

    /// Get the offset of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockOffset(int nblock) { return this->GetNodeN(nblock)->NodeGetOffset_w(); }

    /// Get the size of the i-th sub-block of DOFs in global vector
    virtual unsigned int GetSubBlockSize(int nblock) { return 3; }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) {
        for (int i = 0; i < 3; ++i)
            mvars.push_back(&this->GetNodeN(i)->Variables());
    };

    /// Evaluate N'*F , where N is some type of shape function
    /// evaluated at U,V coordinates of the surface, each ranging in 0..+1
    /// F is a load, N'*F is the resulting generalized load
    /// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in surface
                           const double V,              ///< parametric coordinate in surface
                           ChVectorDynamic<>& Qi,       ///< Return result of N'*F  here, maybe with offset block_offset
                           double& detJ,                ///< Return det[J] here
                           const ChVectorDynamic<>& F,  ///< Input F vector, size is = n.field coords.
                           ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                           ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                           ) {
        // evaluate shape functions (in compressed vector), btw. not dependant on state
        ChMatrixNM<double, 1, 3> N;
        this->ShapeFunctions(N, U,
                             V);  // note: U,V in 0..1 range, thanks to IsTriangleIntegrationNeeded() {return true;}

        // determinant of jacobian is also =2*areaoftriangle, also length of cross product of sides
        ChVector<> p0 = GetNodeN(0)->GetPos();
        ChVector<> p1 = GetNodeN(1)->GetPos();
        ChVector<> p2 = GetNodeN(2)->GetPos();
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

    /// If true, use quadrature over u,v in [0..1] range as triangle volumetric coords
    virtual bool IsTriangleIntegrationNeeded() { return true; }

    /// Gets the normal to the surface at the parametric coordinate u,v.
    /// Normal must be considered pointing outside in case the surface is a boundary to a volume.
    virtual ChVector<> ComputeNormal(const double U, const double V) {
        ChVector<> p0 = GetNodeN(0)->GetPos();
        ChVector<> p1 = GetNodeN(1)->GetPos();
        ChVector<> p2 = GetNodeN(2)->GetPos();
        return Vcross(p1 - p0, p2 - p0).GetNormalized();
    }
};

}  // end namespace fea
}  // end namespace chrono

#endif
