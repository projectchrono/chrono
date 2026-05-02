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

#ifndef CHFIELDELEMENTLOADABLEVOLUME_H
#define CHFIELDELEMENTLOADABLEVOLUME_H

#include "chrono/fea/ChFieldElement.h"
#include "chrono/physics/ChLoaderUVW.h"
#include "chrono/fea/ChField.h"

namespace chrono {

namespace fea {

/// @addtogroup chrono_fea
/// @{


/// Loadable volume of an element, with reference to some interpolated field. 
/// This is used to apply UVW loads of ChLoaderUVWdistributed type, like gravity

class ChApi ChFieldElementLoadableVolume : public ChLoadableUVW {
public:
    /// Construct a loadable volume the given solid finite element and a field.
    /// For example if the field is ChFieldTemperature, this can be used to receive a 
    /// volumetric heat source. If the field is ChFieldDisplacement, this can receive gravity, etc.
    ChFieldElementLoadableVolume(std::shared_ptr<ChFieldElementVolume> element, std::shared_ptr<ChFieldBase> field) : m_element(element), m_field(field) {}

    ~ChFieldElementLoadableVolume() {}

    /// Set the element to wrap as loadable
    virtual void SetElement(std::shared_ptr<ChFieldElementVolume> element) {
        m_element = element;
    }

    // Functions for ChLoadable interface

    /// Get the number of DOFs affected by this element (position part).
    virtual unsigned int GetLoadableNumCoordsPosLevel() override { return m_element->GetNumNodes() * m_field->GetNumFieldCoordsPosLevel(); }

    /// Get the number of DOFs affected by this element (speed part).
    virtual unsigned int GetLoadableNumCoordsVelLevel() override { return m_element->GetNumNodes() * m_field->GetNumFieldCoordsVelLevel(); }

    /// Get all the DOFs packed in a single vector (position part).
    virtual void LoadableGetStateBlockPosLevel(int block_offset, ChState& mD) override {
        unsigned int ncoords_per_node = m_field->GetNumFieldCoordsPosLevel();
        for (unsigned int i = 0; i < m_element->GetNumNodes(); ++i) {
            mD.segment(block_offset, ncoords_per_node) = m_field->GetNodeDataPointer(m_element->GetNode(i))->State();
            block_offset += ncoords_per_node;
        }
    }

    /// Get all the DOFs packed in a single vector (speed part).
    virtual void LoadableGetStateBlockVelLevel(int block_offset, ChStateDelta& mD) override {
        unsigned int ncoords_per_node = m_field->GetNumFieldCoordsVelLevel();
        for (unsigned int i = 0; i < m_element->GetNumNodes(); ++i) {
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
        for (unsigned int i = 0; i < m_element->GetNumNodes(); ++i) {
            m_field->GetNodeDataPointer(m_element->GetNode(i))->DataIntStateIncrement(off_x + i * m_field->GetNumFieldCoordsPosLevel(), x_new, x, off_v + i * m_field->GetNumFieldCoordsPosLevel(), Dv);
        }
    }

    /// Number of coordinates in the interpolated field
    virtual unsigned int GetNumFieldCoords() override { return m_field->GetNumFieldCoordsVelLevel(); }

    /// Get the number of DOFs sub-blocks.
    virtual unsigned int GetNumSubBlocks() override { return m_element->GetNumNodes(); }

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
        for (unsigned int i = 0; i < m_element->GetNumNodes(); ++i)
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
    ) override;

    /// This has no density - it requires a ChMaterial etc.
    virtual double GetDensity() override { return 0; };

    /// If true, use quadrature over u,v in [0..1] range as triangle volumetric coords.
    virtual bool IsTetrahedronIntegrationNeeded() override { return m_element->IsTetrahedronIntegrationCompatible(); }
    virtual bool IsTrianglePrismIntegrationNeeded() override { return m_element->IsTrianglePrismIntegrationCompatible(); }

private:
    std::shared_ptr<ChFieldElementVolume> m_element;  ///< associated element
    std::shared_ptr<ChFieldBase> m_field;  ///< associated field
};




/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
