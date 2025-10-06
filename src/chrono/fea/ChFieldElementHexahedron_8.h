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

#ifndef CHFIELDELEMENTHEXAHEDRON_8_H
#define CHFIELDELEMENTHEXAHEDRON_8_H

#include "chrono/core/ChQuadrature.h"
#include "chrono/fea/ChFieldElement.h"


namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{

// Forward:
class ChNodeFEAmultiXYZ;


/// Linear hexahedron for generic constitutive laws. Can be used for multiphysics.
/// It uses 8 nodes.

class ChApi ChFieldElementHexahedron_8 : public ChFieldElement {
public:
    ChFieldElementHexahedron_8() {}
    virtual ~ChFieldElementHexahedron_8() {}

    //
    // Tetrahedron-specific
    // 

    /// Return the specified tetrahedron node (0 <= n <= 8).
    virtual std::shared_ptr<ChNodeFEAmultiXYZ> GetTetrahedronNode(unsigned int n) { return nodes[n]; }

    /// Set the nodes used by this tetrahedron.
    virtual void SetNodes(std::array<std::shared_ptr<ChNodeFEAmultiXYZ>, 8> mynodes) {
        nodes = mynodes;
    }

    //
    // Interface
    // 

    /// Get the number of nodes used by this element.
    virtual unsigned int GetNumNodes() override { return 8; };

    /// Access the nth node.
    virtual std::shared_ptr<ChNodeFEAbase> GetNode(unsigned int n) override;

    // Return dimension of embedding space X
    virtual int GetSpatialDimensions() const override { return 3; };

    // Return dimension of represented manifold in space
    virtual int GetManifoldDimensions() const override { return 3; };

    // Compute the 8 shape functions N at eta parametric coordinates. 
    virtual void ComputeN(const ChVector3d eta, ChRowVectorDynamic<>& N) override;;

    // Compute shape function material derivatives dN/d\eta at eta parametric coordinates.
    // Write shape functions dN_j(\eta)/d\eta_i in dNde, a matrix with 4 columns, and 3 rows. 
    virtual void ComputedNde(const ChVector3d eta, ChMatrixDynamic<>& dNde) override;

    // Compute Jacobian J, and returns its determinant. J is square 3x3
    virtual double ComputeJ(const ChVector3d eta, ChMatrix33d& J) override;

    // Compute Jacobian Jinv, and returns its determinant. Jinv is square 3x3
    virtual double ComputeJinv(const ChVector3d eta, ChMatrix33d& Jinv) override;

    // Tell the minimum required quadrature order when integrating over the element
    virtual int GetMinQuadratureOrder() const override {
        return 2;
    }

    // Tell how many Gauss points are needed for integration
    virtual int GetNumQuadraturePoints(const int order) const override;

    // Get i-th Gauss point weight and parametric coordinates
    virtual void GetQuadraturePointWeight(const int order, const int i, double& weight, ChVector3d& coords)  const override;


    /// Update, called at least at each time step.
    /// If the element has to keep updated some auxiliary data, such as the rotation matrices for corotational approach,
    /// this should be implemented in this function.
    virtual void Update() {}


private:
    /// Initial setup (called once before start of simulation).
    /// This is used mostly to precompute matrices that do not change during the simulation, i.e. the local stiffness of
    /// each element, if any, the mass, etc.
    virtual void SetupInitial(ChSystem* system) {}

    std::array<std::shared_ptr<ChNodeFEAmultiXYZ>, 8> nodes;
};






/// @} chrono_fea

}  // end namespace fea

//CH_CLASS_VERSION(fea::ChFieldElementHexahedron_8, 0)


}  // end namespace chrono

#endif
