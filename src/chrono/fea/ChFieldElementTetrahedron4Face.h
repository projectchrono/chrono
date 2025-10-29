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
// Authors: Alessandro Tasora 
// =============================================================================

#ifndef CHFIELDELEMENTTETRAHEDRON4FACE_H
#define CHFIELDELEMENTTETRAHEDRON4FACE_H

#include "chrono/fea/ChFieldElementTetrahedron4.h"
#include "chrono/fea/ChField.h"
#include "chrono/fea/ChFieldData.h"
#include "chrono/physics/ChLoadable.h"
#include "chrono/fea/ChNodeFEAfieldXYZ.h"

namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{



/// Loadable tetrahedron face, with reference to some interpolated field. 
/// This is used to apply UV loads of ChLoaderUVdistributed type, for example ChLoaderPressure
/// if referencing the ChDomainDeformation, or Neumann boundary in general like imposed heat flow, etc.
/// Corner nodes, obtainable with GetNode(), are in counterclockwise order seen from the outside.

class ChApi ChFieldTetrahedron4Face : public ChLoadableUV , public ChFieldElementSurface {
public:
    /// Construct the specified face (0 <= id <= 3) on the given hexahedral element.
    /// The face id is: 
    /// id = 0 for face opposed to vertex 0
    /// id = 1 for face opposed to vertex 1 
    /// id = 2 for face opposed to vertex 2 
    /// id = 3 for face opposed to vertex 3
    ChFieldTetrahedron4Face(std::shared_ptr<ChFieldElementTetrahedron4> element, std::shared_ptr<ChFieldBase> field, char id) : m_face_id(id), m_element(element), m_field(field) {}

    ~ChFieldTetrahedron4Face() {}

    //
    // INTERFACE to  ChFieldElement 
    //

    virtual unsigned int GetNumNodes() override { return 3; }

    virtual std::shared_ptr<ChNodeFEAbase> GetNode(unsigned int i) override {
        static int iface0[] = { 1, 3, 2 };
        static int iface1[] = { 0, 2, 3 };
        static int iface2[] = { 0, 3, 1 };
        static int iface3[] = { 0, 1, 2 };

        switch (i) {
        case 0:
            return (m_element->GetNode(iface0[i]));
        case 1:
            return (m_element->GetNode(iface1[i]));
        case 2:
            return (m_element->GetNode(iface2[i]));
        case 3:
            return (m_element->GetNode(iface3[i]));
        }
        return nullptr;
    }

    virtual int GetSpatialDimensions() const override { return 3; }

    virtual void ComputeN(const ChVector3d eta, ChRowVectorDynamic<>& N) override {
        N(0) = 1.0 - eta.x() - eta.y();
        N(1) = eta.x();
        N(2) = eta.y();
        // Btw one could also compute the shape functions from the element  
        //  m_element->ComputeN(..) 
        // and discard the ones not used on face at U or V or W. But here simplify.
    }
    virtual void ComputedNde(const ChVector3d eta, ChMatrixDynamic<>& dNde) override { throw std::exception("Not implemented, not needed."); }
    virtual double ComputeJ(const ChVector3d eta, ChMatrix33d& J)  override { throw std::exception("Not implemented, not needed."); }
    virtual double ComputeJinv(const ChVector3d eta, ChMatrix33d& Jinv)  override { throw std::exception("Not implemented, not needed."); }
    virtual int GetNumQuadraturePointsForOrder(const int order) const  override { throw std::exception("Not implemented, not needed."); }
    virtual void GetQuadraturePointWeight(const int order, const int i, double& weight, ChVector3d& coords) const  override { throw std::exception("Not implemented, not needed."); }

    // The following needed only if element is wrapped as a component of a ChLoaderUV.
    virtual bool IsTriangleIntegrationCompatible() const override { return true; }

    // Functions for ChLoadable interface

    /// Get the number of DOFs affected by this element (position part).
    virtual unsigned int GetLoadableNumCoordsPosLevel() override { return 3 * m_field->GetNumFieldCoordsPosLevel(); }

    /// Get the number of DOFs affected by this element (speed part).
    virtual unsigned int GetLoadableNumCoordsVelLevel() override { return 3 * m_field->GetNumFieldCoordsVelLevel(); }

    /// Get all the DOFs packed in a single vector (position part).
    virtual void LoadableGetStateBlockPosLevel(int block_offset, ChState& mD) override {
        unsigned int ncoords_per_node = m_field->GetNumFieldCoordsPosLevel();
        for (int i = 0; i < 3; ++i) {
            mD.segment(block_offset, ncoords_per_node) = m_field->GetNodeDataPointer(GetNode(i))->State();
            block_offset += ncoords_per_node;
        }
    }

    /// Get all the DOFs packed in a single vector (speed part).
    virtual void LoadableGetStateBlockVelLevel(int block_offset, ChStateDelta& mD) override {
        unsigned int ncoords_per_node = m_field->GetNumFieldCoordsVelLevel();
        for (int i = 0; i < 3; ++i) {
            mD.segment(block_offset, ncoords_per_node) = m_field->GetNodeDataPointer(GetNode(i))->StateDt();
            block_offset += ncoords_per_node;
        }
    }

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x,
        ChState& x_new,
        const ChState& x,
        const unsigned int off_v,
        const ChStateDelta& Dv) override {
        for (int i = 0; i < 3; ++i) {
            m_field->GetNodeDataPointer(GetNode(i))->DataIntStateIncrement(off_x + i * m_field->GetNumFieldCoordsPosLevel(), x_new, x, off_v + i * m_field->GetNumFieldCoordsPosLevel(), Dv);
        }
    }

    /// Number of coordinates in the interpolated field
    virtual unsigned int GetNumFieldCoords() override { return m_field->GetNumFieldCoordsVelLevel(); }

    /// Get the number of DOFs sub-blocks.
    virtual unsigned int GetNumSubBlocks() override { return 3; }

    /// Get the offset of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(unsigned int nblock) override {
        return m_field->GetNodeDataPointer(GetNode(nblock))->DataGetOffsetVelLevel();
    }

    /// Get the size of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(unsigned int nblock) override { return m_field->GetNumFieldCoordsVelLevel(); }

    /// Check if the specified sub-block of DOFs is active.
    virtual bool IsSubBlockActive(unsigned int nblock) const override { return !m_field->GetNodeDataPointer(const_cast<ChFieldTetrahedron4Face*>(this)->GetNode(nblock))->IsFixed(); }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override {
        for (int i = 0; i < 3; ++i)
            mvars.push_back(&m_field->GetNodeDataPointer(GetNode(i))->GetVariable());
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
        ChRowVectorDynamic<double> N(3);
        ComputeN(ChVector3d(U, V,0), N);

        // TODO throw exception if the 3 nodes are not from ChNodeFEAfieldXYZ? the jacobian would not be computable anyway..
        ChVector3d p0 = *(std::dynamic_pointer_cast<ChNodeFEAfieldXYZ>(this->GetNode(0)));
        ChVector3d p1 = *(std::dynamic_pointer_cast<ChNodeFEAfieldXYZ>(this->GetNode(1)));
        ChVector3d p2 = *(std::dynamic_pointer_cast<ChNodeFEAfieldXYZ>(this->GetNode(2)));

        // determinant of jacobian is also =2*areaoftriangle, also length of cross product of sides
        detJ = (Vcross(p2 - p0, p1 - p0)).Length();
        
        int ncoords_node = m_field->GetNumFieldCoordsVelLevel();
        for (int i = 0; i < 3; i++) {
            Qi.segment(ncoords_node * i, ncoords_node) = N(i) * F.segment(0, ncoords_node);
        }
    }

    /// If true, use quadrature over u,v in [0..1] range as triangle volumetric coords.
    virtual bool IsTriangleIntegrationNeeded() override { return true; }

    /// Get the normal to the surface at the parametric coordinate u,v.
    /// Normal must be considered pointing outside in case the surface is a boundary to a volume.
    virtual ChVector3d ComputeNormal(const double U, const double V) override {
        ChVector3d p0 = *(std::dynamic_pointer_cast<ChNodeFEAfieldXYZ>(this->GetNode(0)));
        ChVector3d p1 = *(std::dynamic_pointer_cast<ChNodeFEAfieldXYZ>(this->GetNode(1)));
        ChVector3d p2 = *(std::dynamic_pointer_cast<ChNodeFEAfieldXYZ>(this->GetNode(2)));
        return Vcross(p1 - p0, p2 - p0).GetNormalized();
    }

private:
    char m_face_id;                                  ///< id of the face on the tetrahedron
    std::shared_ptr<ChFieldElementTetrahedron4> m_element;  ///< associated tetrahedron element
    std::shared_ptr<ChFieldBase> m_field;  ///< associated field
};


/////////////////////////////////////////////////////////////////////////////////////////////////////

/// Loadable volume of a tetrahedron, with reference to some interpolated field. 
/// This is used to apply UVW loads of ChLoaderUVWdistributed type, like gravity
/*
class ChApi ChFieldTetrahedronVolume : public ChLoadableUVW {
public:
    /// Construct a loadable volume the given tetrahedron element and a field.
    /// For example if the field is ChFieldTemperature, this can be used to apply a 
    /// volumetric heat source. If the field is ChFieldDisplacement, this can apply gravity, etc.
    ChFieldTetrahedronVolume(std::shared_ptr<ChFieldElementTetrahedron4> element, std::shared_ptr<ChFieldBase> field) : m_element(element), m_field(field) {}

    ~ChFieldTetrahedronVolume() {}

    // Functions for ChLoadable interface

    /// Get the number of DOFs affected by this element (position part).
    virtual unsigned int GetLoadableNumCoordsPosLevel() override { return 4 * m_field->GetNumFieldCoordsPosLevel(); }

    /// Get the number of DOFs affected by this element (speed part).
    virtual unsigned int GetLoadableNumCoordsVelLevel() override { return 4 * m_field->GetNumFieldCoordsVelLevel(); }

    /// Get all the DOFs packed in a single vector (position part).
    virtual void LoadableGetStateBlockPosLevel(int block_offset, ChState& mD) override {
        unsigned int ncoords_per_node = m_field->GetNumFieldCoordsPosLevel();
        for (int i = 0; i < 4; ++i) {
            mD.segment(block_offset, ncoords_per_node) = m_field->GetNodeDataPointer(m_element->GetNode(i))->State();
            block_offset += ncoords_per_node;
        }
    }

    /// Get all the DOFs packed in a single vector (speed part).
    virtual void LoadableGetStateBlockVelLevel(int block_offset, ChStateDelta& mD) override {
        unsigned int ncoords_per_node = m_field->GetNumFieldCoordsVelLevel();
        for (int i = 0; i < 4; ++i) {
            mD.segment(block_offset, ncoords_per_node) = m_field->GetNodeDataPointer(m_element->GetNode(i))->StateDt();
            block_offset += ncoords_per_node;
        }
    }

    /// Increment all DOFs using a delta.
    virtual void LoadableStateIncrement(const unsigned int off_x,
        ChState& x_new,
        const ChState& x,
        const unsigned int off_v,
        const ChStateDelta& Dv) override {
        for (int i = 0; i < 4; ++i) {
            m_field->GetNodeDataPointer(m_element->GetNode(i))->DataIntStateIncrement(off_x + i * m_field->GetNumFieldCoordsPosLevel(), x_new, x, off_v + i * m_field->GetNumFieldCoordsPosLevel(), Dv);
        }
    }

    /// Number of coordinates in the interpolated field
    virtual unsigned int GetNumFieldCoords() override { return m_field->GetNumFieldCoordsVelLevel(); }

    /// Get the number of DOFs sub-blocks.
    virtual unsigned int GetNumSubBlocks() override { return 4; }

    /// Get the offset of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockOffset(unsigned int nblock) override {
        return m_field->GetNodeDataPointer(m_element->GetNode(nblock))->DataGetOffsetVelLevel();
    }

    /// Get the size of the specified sub-block of DOFs in global vector.
    virtual unsigned int GetSubBlockSize(unsigned int nblock) override { return m_field->GetNumFieldCoordsVelLevel(); }

    /// Check if the specified sub-block of DOFs is active.
    virtual bool IsSubBlockActive(unsigned int nblock) const override { return !m_field->GetNodeDataPointer(m_element->GetNode(nblock))->IsFixed(); }

    /// Get the pointers to the contained ChVariables, appending to the mvars vector.
    virtual void LoadableGetVariables(std::vector<ChVariables*>& mvars) override {
        for (int i = 0; i < 4; ++i)
            mvars.push_back(&m_field->GetNodeDataPointer(m_element->GetNode(i))->GetVariable());
    };

    /// Evaluate N'*F , where N is some type of shape function evaluated at U,V coordinates of the surface, each ranging
    /// in -1..+1 F is a load, N'*F is the resulting generalized load. Returns also det[J] with J=[dx/du,..], which may
    /// be useful in Gauss quadrature.
    virtual void ComputeNF(const double U,              ///< parametric coordinate in surface
        const double V,              ///< parametric coordinate in surface
        const double W,              ///< parametric coordinate in surface
        ChVectorDynamic<>& Qi,       ///< result of N'*F, maybe with offset block_offset
        double& detJ,                ///< det[J]
        const ChVectorDynamic<>& F,  ///< Input F vector, size is = n.field coords.
        ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
        ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
    ) override {
        ChRowVectorDynamic<double> N(4);
        ChVector3d eta(U, V, W);
        m_element->ComputeN(eta, N);

        ChMatrixNM<double, 3, 4> Xhh;
        ChMatrix33<double> J;

        detJ = m_element->ComputeJ(eta, J);

        int ncoords_node = m_field->GetNumFieldCoordsVelLevel();
        for (int i = 0; i < 4; i++) {
            Qi.segment(ncoords_node * i, ncoords_node) = N(i) * F.segment(0, ncoords_node);
        }
    }

    /// Field has no density - it requires a ChMaterial etc.
    virtual double GetDensity() override { return 0; };

    /// If true, use quadrature over u,v in [0..1] range as tetrahedral volumetric coords.
    virtual bool IsTetrahedronIntegrationNeeded() override { return true; }

private:
    std::shared_ptr<ChFieldElementTetrahedron4> m_element;  ///< associated hexahedron element
    std::shared_ptr<ChFieldBase> m_field;  ///< associated field
};


*/



/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
