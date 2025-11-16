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

#ifndef CHFIELDELEMENT_H
#define CHFIELDELEMENT_H

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChFrame.h"
#include "chrono/physics/ChLoadable.h"


namespace chrono {

// Forward declarations:
    class ChSystem;

namespace fea {

/// @addtogroup chrono_fea
/// @{

// Forward declarations:
class ChNodeFEAbase;



/// Base class for finite elements over generic fields (scalrar, vectorial, etc), 
/// to be used in the new PDE multiphysics system that uses ChField and ChDomain managers.
/// These elements are NOT of the same class of the old ones, that typically were inherited 
/// from ChElementBase and ChElementGeneric, because the element here does not provide any
/// constitutive law or material: it only provides shape function interpolation, 
/// shape function derivatives, jacobian, list of nodes and quadrature schemes, and
/// few ingredients that later one can take advantage of, say in a ChDomainDeformation 
/// or ChDomainThermal, if some proper ChField is attached to the nodes.

class ChApi ChFieldElement {
public:
    ChFieldElement() {
        quadrature_order = 1; // default order
    }
    virtual ~ChFieldElement() {}

    /// Get the number of nodes used by this element.
    virtual unsigned int GetNumNodes() = 0;

    /// Access the nth node.
    virtual std::shared_ptr<ChNodeFEAbase> GetNode(unsigned int n) = 0;


    // 
    // INTERFACE
    //
    
    // Return dimension of embedding space X: 
    // if element is in 2D space, return 2, if in 3D space return 3
    virtual int GetSpatialDimensions() const = 0;

    // Return dimension of represented manifold in space:  
    // 1 for beams, 2 for shells, 3 for solid finite element. Also size of used eta parametric coordinates.
    virtual int GetManifoldDimensions() const = 0;

    // Compute shape functions N at eta parametric coordinates. 
    // Write shape functions N_i(eta) in N, a row vector with size as GetNumNodes(). 
    // N will be resized if not of proper size.
    virtual void ComputeN(const ChVector3d eta, ChRowVectorDynamic<>& N) = 0;

    // Compute shape function parametric derivatives dN/d\eta at eta parametric coordinates.
    // Write shape functions dN_j(\eta)/d\eta_i in dNde, a matrix with GetNumNodes() columns, and n_rows = GetManifoldDimensions(). 
    // dNde will be resized if not of proper size.
    virtual void ComputedNde(const ChVector3d eta, ChMatrixDynamic<>& dNde) = 0;

    // Compute shape function spatial derivatives dN/dX at eta parametric coordinates.
    // Write shape functions dN_j(eta)/dX_i in dNdX, a matrix with GetNumNodes() columns, and n_rows = GetSpatialDimensions().
    // dNde will be resized if not of proper size.
    // FALLBACK default implementation is dNdX = J^{-T} * dNde;  but if possible implement a more efficient ad hoc computation.
    virtual void ComputedNdX(const ChVector3d eta, ChMatrixDynamic<>& dNdX) {
        ChMatrix33d temp_Jinv;
        ChMatrixDynamic<> temp_dNde;
        ComputedNde(eta, temp_dNde);
        ComputeJinv(eta, temp_Jinv);
        dNdX.resize(GetSpatialDimensions(), GetNumNodes());
        dNdX = temp_Jinv.transpose() * temp_dNde;
    }

    // Compute Jacobian J, and returns its determinant. J is square with size = GetManifoldDimensions()
    virtual double ComputeJ(const ChVector3d eta, ChMatrix33d& J) = 0;

    // Compute Jacobian Jinv, and returns its determinant. Jinv is square with size = GetManifoldDimensions()
    virtual double ComputeJinv(const ChVector3d eta, ChMatrix33d& Jinv) = 0;

    // Tell the minimum required quadrature order when integrating over the element
    virtual int GetQuadratureOrder() const {  return quadrature_order; }

    // Sets the quadrature order when integrating over the element
    virtual void SetQuadratureOrder(const int morder) {  quadrature_order = morder;  }

    // Tell actual number of quadrature points
    virtual int GetNumQuadraturePoints() const { return GetNumQuadraturePointsForOrder(this->quadrature_order); };

    // Tell how many Gauss points are needed for quadrature of some order
    virtual int GetNumQuadraturePointsForOrder(const int order) const = 0;

    // Get i-th Gauss point weight and parametric coordinates
    virtual void GetQuadraturePointWeight(const int order, const int i, double& weight, ChVector3d& coords) const = 0;

    // *************************************

 
    /// Update, called at least at each time step.
    /// If the element has to keep updated some auxiliary data, such as the rotation matrices for corotational approach,
    /// this should be implemented in this function.
    virtual void Update() {}

    /// Initial setup (called once before start of simulation).
    /// This is used mostly to precompute matrices that do not change during the simulation, i.e. the local stiffness of
    /// each element, if any, the mass, etc.
    virtual void SetupInitial(ChSystem* system) {}

protected:

    int quadrature_order = 1;
};



//-----------------------------------------------------------------------------------------

/// Class for grouping all finite elements that have a manifold dimension of 1,
/// like beams, cables, etc.

class ChApi ChFieldElementLine : public ChFieldElement {
public:
    virtual int GetManifoldDimensions() const override { return 1; }

};


//-----------------------------------------------------------------------------------------

/// Class for grouping all finite elements that have a manifold dimension of 2,
/// like surfaces (shells, etc.)

class ChApi ChFieldElementSurface : public ChFieldElement {
public:
    virtual int GetManifoldDimensions() const override { return 2; }

    // The following needed only if element is wrapped as a component of a ChLoaderUV.
    virtual bool IsTriangleIntegrationCompatible() const = 0;

    /// Gets the normal to the surface at the parametric coordinate u,v.
    /// Normal must be considered pointing outside in case the surface is a boundary to a volume.
    virtual ChVector3d ComputeNormal(const double U, const double V) = 0;
};


//-----------------------------------------------------------------------------------------

/// Class for grouping all finite elements that have a manifold dimension of 3,
/// like volumes (tetrahedrons, hexahedrons, etc.)

class ChApi ChFieldElementVolume : public ChFieldElement {
public:
    virtual int GetManifoldDimensions() const override { return 3; }

    virtual int GetNumFaces() { return 0; }
    virtual std::shared_ptr<ChFieldElementSurface> BuildFace(int i_face, std::shared_ptr<ChFieldElementVolume> shared_this) { return nullptr; }

    // The following needed only if element is wrapped as component of a ChLoaderUVW.
    virtual bool IsTetrahedronIntegrationCompatible() const = 0;
    virtual bool IsTrianglePrismIntegrationCompatible() const = 0;
};


// Forward
class ChFieldBase;

class ChLoadableUVwithField : public ChLoadableUV {
public:
    ChLoadableUVwithField(std::shared_ptr<ChFieldBase> field = nullptr) : m_field(field) {}

    void SetField(std::shared_ptr<ChFieldBase> field) { m_field = field; }

protected:
    std::shared_ptr<ChFieldBase> m_field;
};




/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
