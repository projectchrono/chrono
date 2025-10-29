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

#ifndef CHFIELDELEMENTTETRAHEDRON4_H
#define CHFIELDELEMENTTETRAHEDRON4_H

#include "chrono/core/ChQuadrature.h"
#include "chrono/fea/ChFieldElement.h"


namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{

// Forward:
class ChNodeFEAfieldXYZ;


/// Linear tetrahedron for generic constitutive laws. Can be used for multiphysics.
/// Because of linear interpolation, some closed form ad-hoc expressions are used here 
/// for J, dNdX, etc., aiming at higher performance.

class ChApi ChFieldElementTetrahedron4 : public ChFieldElementVolume {
public:
    ChFieldElementTetrahedron4() {
        this->quadrature_order = 1; 
    }
    virtual ~ChFieldElementTetrahedron4() {}

    //
    // Tetrahedron-specific
    // 

    /// Return the specified tetrahedron node (0 <= n <= 3).
    virtual std::shared_ptr<ChNodeFEAfieldXYZ> GetTetrahedronNode(unsigned int n) { return nodes[n]; }

    /// Set the nodes used by this tetrahedron.
    virtual void SetNodes(std::shared_ptr<ChNodeFEAfieldXYZ> nodeA,
        std::shared_ptr<ChNodeFEAfieldXYZ> nodeB,
        std::shared_ptr<ChNodeFEAfieldXYZ> nodeC,
        std::shared_ptr<ChNodeFEAfieldXYZ> nodeD);

    //
    // Interface
    // 

    /// Get the number of nodes used by this element.
    virtual unsigned int GetNumNodes() override { return 4; };

    /// Access the nth node.
    virtual std::shared_ptr<ChNodeFEAbase> GetNode(unsigned int n) override;

    // Return dimension of embedding space X
    virtual int GetSpatialDimensions() const override { return 3; };

    // Compute the 4 shape functions N at eta parametric coordinates. 
    virtual void ComputeN(const ChVector3d eta, ChRowVectorDynamic<>& N) override;;

    // Compute shape function material derivatives dN/d\eta at eta parametric coordinates.
    // Write shape functions dN_j(\eta)/d\eta_i in dNde, a matrix with 4 columns, and 3 rows. 
    virtual void ComputedNde(const ChVector3d eta, ChMatrixDynamic<>& dNde) override;

    // Compute shape function spatial derivatives dN/dX at eta parametric coordinates.
    // Write shape functions dN_j(eta)/dX_i in dNdX, a matrix with 4 columns, and 3 rows.
    // Instead of falling back to default dNdX = J^{-T} * dNde; for lin tetrahedron we know the ad-hoc expression:
    virtual void ComputedNdX(const ChVector3d eta, ChMatrixDynamic<>& dNdX) override;

    // Compute Jacobian J, and returns its determinant. J is square 3x3
    virtual double ComputeJ(const ChVector3d eta, ChMatrix33d& J) override;

    // Compute Jacobian Jinv, and returns its determinant. Jinv is square 3x3
    virtual double ComputeJinv(const ChVector3d eta, ChMatrix33d& Jinv) override;

    // Tell how many Gauss points are needed for integration
    virtual int GetNumQuadraturePointsForOrder(const int order) const override;

    // Get i-th Gauss point weight and parametric coordinates
    virtual void GetQuadraturePointWeight(const int order, const int i, double& weight, ChVector3d& coords)  const override;


    /// Update, called at least at each time step.
    /// If the element has to keep updated some auxiliary data, such as the rotation matrices for corotational approach,
    /// this should be implemented in this function.
    virtual void Update() {}

    // The following needed only if element is wrapped as component of a ChLoaderUVW.
    virtual bool IsTetrahedronIntegrationCompatible() const { return true; }
    virtual bool IsTrianglePrismIntegrationCompatible() const { return false; }

private:
    /// Initial setup (called once before start of simulation).
    /// This is used mostly to precompute matrices that do not change during the simulation, i.e. the local stiffness of
    /// each element, if any, the mass, etc.
    virtual void SetupInitial(ChSystem* system) {}

    std::array<std::shared_ptr<ChNodeFEAfieldXYZ>, 4> nodes;
};






/// @} chrono_fea

}  // end namespace fea

//CH_CLASS_VERSION(fea::ChFieldElementTetrahedron4, 0)


}  // end namespace chrono

#endif
