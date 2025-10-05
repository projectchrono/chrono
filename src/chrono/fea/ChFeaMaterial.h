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

#ifndef CHFEAMATERIAL_H
#define CHFEAMATERIAL_H

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChFrame.h"
#include "chrono/core/ChTensors.h"
#include "chrono/core/ChQuadrature.h"
#include "chrono/fea/ChNodeFEAbase.h"
#include "chrono/physics/ChNodeXYZ.h"
#include "chrono/solver/ChVariablesGeneric.h"
#include "chrono/solver/ChVariablesGenericDiagonalMass.h"

namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{

 
/// Base class for nodes to be used in the new multiphysics FEA system.
/// You can inherit from this for specializing it, ex making it a xyz node.
/// Note: this type of multiphysics node do not carry any ChVariable with it.
//class ChApi ChFeaNode {
//};

/// Class for a node of a generic 3D finite element node with x,y,z position.
/// This is the typical node that can be used for tetrahedrons, etc.
/// Note: this type of multiphysics node do not carry any ChVariable with it.
class ChApi ChFeaNodeXYZ : public ChNodeFEAbase, public ChVector3d {
public:
    ChFeaNodeXYZ(ChVector3d reference_pos = VNULL) : ChNodeFEAbase(), ChVector3d(reference_pos) {};
    ChFeaNodeXYZ(const ChFeaNodeXYZ& other) {};
    virtual ~ChFeaNodeXYZ() {}

    //ChFeaNodeXYZ& operator=(const ChFeaNodeXYZ& other);

    void SetReferencePos(const ChVector3d ref_pos) { this->Set(ref_pos); }
    ChVector3d GetReferencePos() { return *this; }

    // INTERFACE to ChNodeFEAbase  
    // ***NOTE*** none of these are useful, neither supported, maybe in future
    // inherit from the empty ChFeaNode. Now do this way to reuse ChMesh stuff.

    virtual void Relax() override {}
    virtual void ForceToRest() override {}
    virtual void SetFixed(bool fixed) override {}
    virtual bool IsFixed() const override { return false; }
    virtual unsigned int GetNumCoordsPosLevel() const override { return 0; }

    // SERIALIZATION

    /// Method to allow serialization of transient data to archives.
    //virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    //virtual void ArchiveIn(ChArchiveIn& archive) override;

protected:
};

//--------------------------------------------------------------------
//--------------------------------------------------------------------



/// Base class for finite elements with generic constitutive laws

class ChApi ChFeaElement {
public:
    ChFeaElement() {}
    virtual ~ChFeaElement() {}

    /// Get the number of nodes used by this element.
    virtual unsigned int GetNumNodes() = 0;

    /// Access the nth node.
    virtual std::shared_ptr<ChNodeFEAbase> GetNode(unsigned int n) = 0;


    // *************************************
    // *** interface for new generic fea ***
    // *************************************
    
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
    virtual int GetMinQuadratureOrder() const = 0;

    // Tell how many Gauss points are needed for quadrature
    virtual int GetNumQuadraturePoints(const int order) const = 0;

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

private:

    //friend class ChMesh;
};



// -----------------------------------------------------------------------------

/// Linear tetrahedron for generic constitutive laws. Can be used for multiphysics.
/// Because of linear interpolation, some closed form ad-hoc expressions are used here 
/// for J, dNdX, etc., aiming at higher performance.

class ChApi ChFeaElementTetrahedron_4 : public ChFeaElement {
public:
    ChFeaElementTetrahedron_4() {}
    virtual ~ChFeaElementTetrahedron_4() {}

    //
    // Tetrahedron-specific
    // 

    /// Return the specified tetrahedron node (0 <= n <= 3).
    virtual std::shared_ptr<ChFeaNodeXYZ> GetTetrahedronNode(unsigned int n) { return nodes[n]; }

    /// Set the nodes used by this tetrahedron.
    virtual void SetNodes(std::shared_ptr<ChFeaNodeXYZ> nodeA,
        std::shared_ptr<ChFeaNodeXYZ> nodeB,
        std::shared_ptr<ChFeaNodeXYZ> nodeC,
        std::shared_ptr<ChFeaNodeXYZ> nodeD) {
        nodes[0] = nodeA;
        nodes[1] = nodeB;
        nodes[2] = nodeC;
        nodes[3] = nodeD;
    }

    //
    // Interface
    // 

    /// Get the number of nodes used by this element.
    virtual unsigned int GetNumNodes() override { return 4; };

    /// Access the nth node.
    virtual std::shared_ptr<ChNodeFEAbase> GetNode(unsigned int n) override {
        return nodes[n];
    }

    // Return dimension of embedding space X
    virtual int GetSpatialDimensions() const override { return 3; };

    // Return dimension of represented manifold in space
    virtual int GetManifoldDimensions() const override { return 3; };

    // Compute the 4 shape functions N at eta parametric coordinates. 
    virtual void ComputeN(const ChVector3d eta, ChRowVectorDynamic<>& N) override {
        N.resize(GetNumNodes());
        N[0] = eta[0];
        N[1] = eta[1];
        N[2] = eta[2];
        N[3] = 1.0 - eta[0] - eta[1] - eta[2];
    };

    // Compute shape function material derivatives dN/d\eta at eta parametric coordinates.
    // Write shape functions dN_j(\eta)/d\eta_i in dNde, a matrix with 4 columns, and 3 rows. 
    virtual void ComputedNde(const ChVector3d eta, ChMatrixDynamic<>& dNde) override {
        dNde.setZero(GetManifoldDimensions(),GetNumNodes());
        dNde(0, 0) = 1.0;
        dNde(1, 1) = 1.0;
        dNde(2, 2) = 1.0;
        dNde(0, 3) = -1.0;
        dNde(1, 3) = -1.0;
        dNde(2, 3) = -1.0;
    }

    // Compute shape function spatial derivatives dN/dX at eta parametric coordinates.
    // Write shape functions dN_j(eta)/dX_i in dNdX, a matrix with 4 columns, and 3 rows.
    // Instead of falling back to default dNdX = J^{-T} * dNde; for lin tetrahedron we know the ad-hoc expression:
    virtual void ComputedNdX(const ChVector3d eta, ChMatrixDynamic<>& dNdX) override {
        dNdX.resize(3, 4);
        ChVector3d x14 = *this->nodes[0] - *this->nodes[3];
        ChVector3d x24 = *this->nodes[1] - *this->nodes[3];
        ChVector3d x34 = *this->nodes[2] - *this->nodes[3];
        double inv_det_J = 1.0 / Vdot(x14, Vcross( x24, x34 ) );
        ChVector3d vai = inv_det_J * Vcross(x24, x34);
        ChVector3d vbi = inv_det_J * Vcross(x34, x14);
        ChVector3d vci = inv_det_J * Vcross(x14, x24);
        dNdX(0, 0) = vai.x();   dNdX(0, 1) = vbi.x();   dNdX(0, 2) = vci.x();   dNdX(0, 3) = -vai.x() - vbi.x() - vci.x();
        dNdX(1, 0) = vai.y();   dNdX(1, 1) = vbi.y();   dNdX(1, 2) = vci.y();   dNdX(1, 3) = -vai.y() - vbi.y() - vci.y();
        dNdX(2, 0) = vai.z();   dNdX(2, 1) = vbi.z();   dNdX(2, 2) = vci.z();   dNdX(2, 3) = -vai.z() - vbi.z() - vci.z();
        //***TEST***
        //ChMatrixDynamic<> test_dNdX(3, 4);
        //ChFeaElement::ComputedNdX(eta, test_dNdX);
        //**** 
    }

    // Compute Jacobian J, and returns its determinant. J is square 3x3
    virtual double ComputeJ(const ChVector3d eta, ChMatrix33d& J) override {
        ChVector3d x14 = *this->nodes[0] - *this->nodes[3];
        ChVector3d x24 = *this->nodes[1] - *this->nodes[3];
        ChVector3d x34 = *this->nodes[2] - *this->nodes[3];
        J(0, 0) = x14.x();  J(0, 1) = x24.x();  J(0, 2) = x34.x();
        J(1, 0) = x14.y();  J(1, 1) = x24.y();  J(1, 2) = x34.y();
        J(2, 0) = x14.z();  J(2, 1) = x24.z();  J(2, 2) = x34.z();
        return Vdot(x14, Vcross(x24, x34));
    }

    // Compute Jacobian Jinv, and returns its determinant. Jinv is square 3x3
    virtual double ComputeJinv(const ChVector3d eta, ChMatrix33d& Jinv) override {
        ChVector3d x14 = *this->nodes[0] - *this->nodes[3];
        ChVector3d x24 = *this->nodes[1] - *this->nodes[3];
        ChVector3d x34 = *this->nodes[2] - *this->nodes[3];
        double inv_det_J = 1.0 / Vdot(x14, Vcross(x24, x34));
        Jinv(0, Eigen::indexing::all) = Vcross(x24, x34).eigen().transpose();
        Jinv(1, Eigen::indexing::all) = Vcross(x34, x14).eigen().transpose();
        Jinv(2, Eigen::indexing::all) = Vcross(x14, x24).eigen().transpose();
        return inv_det_J;
    }

    // Tell the minimum required quadrature order when integrating over the element
    virtual int GetMinQuadratureOrder() const override {
        return 1;
    }

    // Tell how many Gauss points are needed for integration
    virtual int GetNumQuadraturePoints(const int order) const override {
        if (order==1) 
            return 1; // shortcut
        ChQuadratureTablesTetrahedron* mtables = ChQuadrature::GetStaticTablesTetrahedron();
        return (int)(mtables->Weight[order - 1].size());
    }

    // Get i-th Gauss point weight and parametric coordinates
    virtual void GetQuadraturePointWeight(const int order, const int i, double& weight, ChVector3d& coords)  const override {
        if (order == 1) {
            coords.Set(0.25);
            weight = CH_1_6;
            return; // shortcut
        }
        ChQuadratureTablesTetrahedron* mtables = ChQuadrature::GetStaticTablesTetrahedron();
        coords.x() = mtables->LrootsU[order - 1][i];
        coords.y() = mtables->LrootsV[order - 1][i];
        coords.z() = mtables->LrootsW[order - 1][i];
        weight = mtables->Weight[order - 1][i] * CH_1_6; // the 1/6 factor is not in table
    }

    // *************************************


    /// Update, called at least at each time step.
    /// If the element has to keep updated some auxiliary data, such as the rotation matrices for corotational approach,
    /// this should be implemented in this function.
    virtual void Update() {}


private:
    /// Initial setup (called once before start of simulation).
    /// This is used mostly to precompute matrices that do not change during the simulation, i.e. the local stiffness of
    /// each element, if any, the mass, etc.
    virtual void SetupInitial(ChSystem* system) {}

    std::array<std::shared_ptr<ChFeaNodeXYZ>, 4> nodes;
};



// -----------------------------------------------------------------------------

/// Linear hexahedron for generic constitutive laws. Can be used for multiphysics.
/// It uses 8 nodes.

class ChApi ChFeaElementHexahedron_8 : public ChFeaElement {
public:
    ChFeaElementHexahedron_8() {}
    virtual ~ChFeaElementHexahedron_8() {}

    //
    // Tetrahedron-specific
    // 

    /// Return the specified tetrahedron node (0 <= n <= 8).
    virtual std::shared_ptr<ChFeaNodeXYZ> GetTetrahedronNode(unsigned int n) { return nodes[n]; }

    /// Set the nodes used by this tetrahedron.
    virtual void SetNodes(std::array<std::shared_ptr<ChFeaNodeXYZ>, 8> mynodes) {
        nodes = mynodes;
    }

    //
    // Interface
    // 

    /// Get the number of nodes used by this element.
    virtual unsigned int GetNumNodes() override { return 8; };

    /// Access the nth node.
    virtual std::shared_ptr<ChNodeFEAbase> GetNode(unsigned int n) override {
        return nodes[n];
    }

    // Return dimension of embedding space X
    virtual int GetSpatialDimensions() const override { return 3; };

    // Return dimension of represented manifold in space
    virtual int GetManifoldDimensions() const override { return 3; };

    // Compute the 8 shape functions N at eta parametric coordinates. 
    virtual void ComputeN(const ChVector3d eta, ChRowVectorDynamic<>& N) override {
        N.resize(GetNumNodes());
        N[0] = 0.125 * (1 - eta[0])*(1 - eta[1])*(1 - eta[2]);
        N[1] = 0.125 * (1 + eta[0])*(1 - eta[1])*(1 - eta[2]);
        N[2] = 0.125 * (1 + eta[0])*(1 + eta[1])*(1 - eta[2]);
        N[3] = 0.125 * (1 - eta[0])*(1 + eta[1])*(1 - eta[2]);
        N[4] = 0.125 * (1 - eta[0])*(1 - eta[1])*(1 + eta[2]);
        N[5] = 0.125 * (1 + eta[0])*(1 - eta[1])*(1 + eta[2]);
        N[6] = 0.125 * (1 + eta[0])*(1 + eta[1])*(1 + eta[2]);
        N[7] = 0.125 * (1 - eta[0])*(1 + eta[1])*(1 + eta[2]);
    };

    // Compute shape function material derivatives dN/d\eta at eta parametric coordinates.
    // Write shape functions dN_j(\eta)/d\eta_i in dNde, a matrix with 4 columns, and 3 rows. 
    virtual void ComputedNde(const ChVector3d eta, ChMatrixDynamic<>& dNde) override {
        dNde.setZero(GetManifoldDimensions(), GetNumNodes());
        dNde(0, 0) = -  (1 - eta[1]) * (1 - eta[2]);
        dNde(0, 1) = +  (1 - eta[1]) * (1 - eta[2]);
        dNde(0, 2) = +  (1 + eta[1]) * (1 - eta[2]);
        dNde(0, 3) = -  (1 + eta[1]) * (1 - eta[2]);
        dNde(0, 4) = -  (1 - eta[1]) * (1 + eta[2]);
        dNde(0, 5) = +  (1 - eta[1]) * (1 + eta[2]);
        dNde(0, 6) = +  (1 + eta[1]) * (1 + eta[2]);
        dNde(0, 7) = -  (1 + eta[1]) * (1 + eta[2]);

        dNde(1, 0) = - (1 - eta[0]) * (1 - eta[2]);
        dNde(1, 1) = - (1 + eta[0]) * (1 - eta[2]);
        dNde(1, 2) = + (1 + eta[0]) * (1 - eta[2]);
        dNde(1, 3) = + (1 - eta[0]) * (1 - eta[2]);
        dNde(1, 4) = - (1 - eta[0]) * (1 + eta[2]);
        dNde(1, 5) = - (1 + eta[0]) * (1 + eta[2]);
        dNde(1, 6) = + (1 + eta[0]) * (1 + eta[2]);
        dNde(1, 7) = + (1 - eta[0]) * (1 + eta[2]);

        dNde(2, 0) = - (1 - eta[0]) * (1 - eta[1]);
        dNde(2, 1) = - (1 + eta[0]) * (1 - eta[1]);
        dNde(2, 2) = - (1 + eta[0]) * (1 + eta[1]);
        dNde(2, 3) = - (1 - eta[0]) * (1 + eta[1]);
        dNde(2, 4) = + (1 - eta[0]) * (1 - eta[1]);
        dNde(2, 5) = + (1 + eta[0]) * (1 - eta[1]);
        dNde(2, 6) = + (1 + eta[0]) * (1 + eta[1]);
        dNde(2, 7) = + (1 - eta[0]) * (1 + eta[1]);

        dNde *= 0.125; // the 1/8 factor
    }

    // Compute Jacobian J, and returns its determinant. J is square 3x3
    virtual double ComputeJ(const ChVector3d eta, ChMatrix33d& J) override {
        ChMatrixNM<double, 3, 8> Xhat;
        for (int i = 0; i < 8; ++i)
            Xhat.block<3, 1>(0, i) = std::static_pointer_cast<ChFeaNodeXYZ>(this->GetNode(i))->eigen();
        // J = Xhat * dNde^T
        ChMatrixDynamic<double> dNde(3,8);
        ComputedNde(eta, dNde);
        J = Xhat * dNde.transpose();
        return J.determinant();
    }

    // Compute Jacobian Jinv, and returns its determinant. Jinv is square 3x3
    virtual double ComputeJinv(const ChVector3d eta, ChMatrix33d& Jinv) override {
        ChMatrix33<double> J;
        this->ComputeJ(eta, J);
        double mdet;
        bool isinvertible;
        J.computeInverseAndDetWithCheck(Jinv, mdet, isinvertible, 1e-9);
        return mdet;
    }

    // Tell the minimum required quadrature order when integrating over the element
    virtual int GetMinQuadratureOrder() const override {
        return 2;
    }

    // Tell how many Gauss points are needed for integration
    virtual int GetNumQuadraturePoints(const int order) const override {
        if (order == 1)
            return 1; // shortcut
        ChQuadratureTables* mtables = ChQuadrature::GetStaticTables();
        int points_on_abscissa = (int)mtables->Weight[order - 1].size();
        return (points_on_abscissa * points_on_abscissa * points_on_abscissa);
    }

    // Get i-th Gauss point weight and parametric coordinates
    virtual void GetQuadraturePointWeight(const int order, const int i, double& weight, ChVector3d& coords)  const override {
        ChQuadratureTables* mtables = ChQuadrature::GetStaticTables();
        int points_on_abscissa = (int)mtables->Weight[order - 1].size();
        int j = i / (points_on_abscissa * points_on_abscissa);
        int k = (i / points_on_abscissa) % points_on_abscissa;
        int l = i % points_on_abscissa;
        coords.x() = mtables->Lroots[order - 1][j];
        coords.y() = mtables->Lroots[order - 1][k];
        coords.z() = mtables->Lroots[order - 1][l];
        weight = mtables->Weight[order - 1][j] * mtables->Weight[order - 1][k] * mtables->Weight[order - 1][l]; 
    }

    // *************************************


    /// Update, called at least at each time step.
    /// If the element has to keep updated some auxiliary data, such as the rotation matrices for corotational approach,
    /// this should be implemented in this function.
    virtual void Update() {}


private:
    /// Initial setup (called once before start of simulation).
    /// This is used mostly to precompute matrices that do not change during the simulation, i.e. the local stiffness of
    /// each element, if any, the mass, etc.
    virtual void SetupInitial(ChSystem* system) {}

    std::array<std::shared_ptr<ChFeaNodeXYZ>, 8> nodes;
};


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

/// Base class for the per-node and per-materialpoint properties of some material,
/// for example it can contain some type of ChVariable for integrable state. 
/// Used, among others, in ChFeaField.

class ChFeaFieldData {
public:
    // Access state at node
    virtual ChVectorRef State() = 0;

    // Access state time derivative at node
    virtual ChVectorRef StateDt() = 0;

    /// Access the applied load term (ex. the atomic load/source in a Poisson equation).
    virtual ChVectorRef Load() = 0;

    /// Access the contained ChVariable. 
    virtual ChVariables& GetVariable() = 0;

    /// Fix/release this state.
    /// If fixed, its state variables are not changed by the solver.
    virtual void SetFixed(bool fixed) = 0;
    virtual bool IsFixed() const = 0;

    /// Get the number of degrees of freedom of this state.
    virtual unsigned int GetNumCoordsPosLevel() { return 0; }
    virtual unsigned int GetNumCoordsVelLevel() { return 0; }
    static unsigned int StaticGetNumCoordsPosLevel() { return 0; }
    static unsigned int StaticGetNumCoordsVelLevel() { return 0; }

    unsigned int NodeGetOffsetPosLevel() { return offset_x; }
    unsigned int NodeGetOffsetVelLevel() { return offset_w; }
    void NodeSetOffsetPosLevel(const unsigned int moff) { offset_x = moff; }
    void NodeSetOffsetVelLevel(const unsigned int moff) { offset_w = moff; }

    virtual void NodeIntStateGather(const unsigned int off_x,
        ChState& x,
        const unsigned int off_v,
        ChStateDelta& v,
        double& T) {}
    virtual void NodeIntStateScatter(const unsigned int off_x,
        const ChState& x,
        const unsigned int off_v,
        const ChStateDelta& v,
        const double T) {}
    virtual void NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {}
    virtual void NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {}
    virtual void NodeIntStateIncrement(const unsigned int off_x,
        ChState& x_new,
        const ChState& x,
        const unsigned int off_v,
        const ChStateDelta& Dv) {
        for (unsigned int i = 0; i < GetNumCoordsPosLevel(); ++i) {
            x_new(off_x + i) = x(off_x + i) + Dv(off_v + i);
        }
    }
    virtual void NodeIntStateGetIncrement(const unsigned int off_x,
        const ChState& x_new,
        const ChState& x,
        const unsigned int off_v,
        ChStateDelta& Dv) {
        for (unsigned int i = 0; i < GetNumCoordsPosLevel(); ++i) {
            Dv(off_v + i) = x_new(off_x + i) - x(off_x + i);
        }
    }
    virtual void NodeIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {}
    virtual void NodeIntLoadResidual_Mv(const unsigned int off,
        ChVectorDynamic<>& R,
        const ChVectorDynamic<>& w,
        const double c) {}
    virtual void NodeIntLoadLumpedMass_Md(const unsigned int off,
        ChVectorDynamic<>& Md,
        double& error,
        const double c) {};
    virtual void NodeIntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) {}
    virtual void NodeIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) {}
    virtual void InjectVariables(ChSystemDescriptor& descriptor) {}

private:
    unsigned int offset_x;
    unsigned int offset_w;
};

template <int T_nstates>
class ChFeaFieldDataGeneric : public ChFeaFieldData {
public:
    ChFeaFieldDataGeneric() :
        mvariables(T_nstates) {
        state.setZero();
        state_dt.setZero();
        F.setZero();
    }

    virtual ChVectorRef State() override { return state; }

    virtual ChVectorRef StateDt() override { return state_dt; }

    virtual ChVectorRef Load() override { return F; }
 
    virtual ChVariables& GetVariable() override { return mvariables; }

    virtual void SetFixed(bool fixed) override {
        mvariables.SetDisabled(fixed);
    }
    virtual bool IsFixed() const override {
        return mvariables.IsDisabled();
    }

    /// Get the number of degrees of freedom of the field.
    virtual unsigned int GetNumCoordsPosLevel() override { return T_nstates; }
    virtual unsigned int GetNumCoordsVelLevel() override { return T_nstates; }
    static unsigned int StaticGetNumCoordsPosLevel() { return T_nstates; }
    static unsigned int StaticGetNumCoordsVelLevel() { return T_nstates; }

    virtual void NodeIntStateGather(const unsigned int off_x,
                                    ChState& x,
                                    const unsigned int off_v,
                                    ChStateDelta& v,
                                    double& T) override {
        x.segment(off_x, T_nstates) = state;
        v.segment(off_v, T_nstates) = state_dt;
    }

    virtual void NodeIntStateScatter(const unsigned int off_x,
                                    const ChState& x,
                                    const unsigned int off_v,
                                    const ChStateDelta& v,
                                    const double T) override {
        state = x.segment(off_x, T_nstates);
        state_dt = v.segment(off_v, T_nstates);
    }

    virtual void NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) override {}
    virtual void NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) override {}

    virtual void NodeIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override {
        R.segment(off, T_nstates) += c * F;
    }

    virtual void NodeIntLoadResidual_Mv(const unsigned int off,
                                    ChVectorDynamic<>& R,
                                    const ChVectorDynamic<>& w,
                                    const double c) override {
        //R(off) += c * mvariables.GetMass() * w(off);
    }

    virtual void NodeIntLoadLumpedMass_Md(const unsigned int off,
                                    ChVectorDynamic<>& Md,
                                    double& error,
                                    const double c) override {
        //for (int i = 0; i < T_nstates; ++i) {
        //    Md(off + i) += c * mvariables.GetMass().(i, i);
        //}
        //error += std::abs(mvariables.GetMass().sum() - mvariables.GetMass().diagonal().sum());
    }

    virtual void NodeIntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) override {
        mvariables.State() = v.segment(off_v, T_nstates);
        mvariables.Force() = R.segment(off_v, T_nstates);
    }

    virtual void NodeIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) override {
        v.segment(off_v, T_nstates) = mvariables.State();
    }

    virtual void InjectVariables(ChSystemDescriptor& descriptor) override {
        descriptor.InsertVariables(&mvariables);
    }

    
private:
    ChVariablesGeneric mvariables;
    ChVectorN<double, T_nstates> state;
    ChVectorN<double, T_nstates> state_dt;
    ChVectorN<double, T_nstates> F;
};



class ChFeaFieldDataPos3D : public ChFeaFieldData {
public:
    ChFeaFieldDataPos3D()  {
        pos.setZero();
        pos_dt.setZero();
        pos_dtdt.setZero();
        F.setZero();
    }
    
    // Custom properties, helpers etc.

    virtual ChVector3d GetPos() const { return ChVector3d(this->pos); }
    virtual void SetPos(const ChVector3d mpos) { this->pos = mpos.eigen(); }

    virtual ChVector3d GetPosDt() const { return ChVector3d(this->pos_dt); }
    virtual void SetPosDt(const ChVector3d mposdt) { this->pos_dt = mposdt.eigen(); }

    virtual ChVector3d GetPosDt2() const { return ChVector3d(this->pos_dtdt); }
    virtual void SetPosDt2(const ChVector3d mposdtdt) { this->pos_dtdt = mposdtdt.eigen(); }

    virtual ChVector3d GetLoad() const { return ChVector3d(this->F); }
    virtual void SetLoad(const ChVector3d mF) { this->F = mF.eigen(); }

    // interface

    virtual ChVectorRef State() { return pos; }

    virtual ChVectorRef StateDt() { return pos_dt; }

    virtual ChVectorRef Load() { return F; }

    virtual ChVariables& GetVariable() { return mvariables; }

    virtual void SetFixed(bool fixed) {
        mvariables.SetDisabled(fixed);
    }
    virtual bool IsFixed() const {
        return mvariables.IsDisabled();
    }

    /// Get the number of degrees of freedom of the field.
    virtual unsigned int GetNumCoordsPosLevel() override { return 3; }
    virtual unsigned int GetNumCoordsVelLevel() override { return 3; }
    static unsigned int StaticGetNumCoordsPosLevel() { return 3; }
    static unsigned int StaticGetNumCoordsVelLevel() { return 3; }

    virtual void NodeIntStateGather(const unsigned int off_x,
                                    ChState& x,
                                    const unsigned int off_v,
                                    ChStateDelta& v,
                                    double& T) {
        x.segment(off_x, 3) = pos;
        v.segment(off_v, 3) = pos_dt;
    }

    virtual void NodeIntStateScatter(const unsigned int off_x,
                                    const ChState& x,
                                    const unsigned int off_v,
                                    const ChStateDelta& v,
                                    const double T) {
        pos = x.segment(off_x, 3);
        pos_dt = v.segment(off_v, 3);
    }

    virtual void NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
        a.segment(off_a, 3) = pos_dtdt;
    }
    virtual void NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
        pos_dtdt = a.segment(off_a, 3);
    }

    virtual void NodeIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
        R.segment(off, 3) += c * F;
    }

    virtual void NodeIntLoadResidual_Mv(const unsigned int off,
                                        ChVectorDynamic<>& R,
                                        const ChVectorDynamic<>& w,
                                        const double c) {
        R.segment(off, 3) += c * mvariables.GetNodeMass() * w.segment(off, 3);
    }

    virtual void NodeIntLoadLumpedMass_Md(const unsigned int off,
                                        ChVectorDynamic<>& Md,
                                        double& error,
                                        const double c) {
        for (int i = 0; i < 3; ++i) {
            Md(off + i) += c * mvariables.GetNodeMass();
        }
    }

    virtual void NodeIntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) {
        mvariables.State() = v.segment(off_v, 3);
        mvariables.Force() = R.segment(off_v, 3);
    }

    virtual void NodeIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) {
        v.segment(off_v, 3) = mvariables.State();
    }

    virtual void InjectVariables(ChSystemDescriptor& descriptor) {
        descriptor.InsertVariables(&mvariables);
    }

    
private:
    ChVariablesNode mvariables;
    ChVectorN<double, 3> pos;
    ChVectorN<double, 3> pos_dt;
    ChVectorN<double, 3> pos_dtdt;
    ChVectorN<double, 3> F;
};



// -----------------------------------------------------------------------------

class ChFeaFieldBase : public ChPhysicsItem {
public:

    ChFeaFieldBase() {}
    virtual ~ChFeaFieldBase() {}

    virtual void AddNode(std::shared_ptr<ChNodeFEAbase> mnode) = 0;

    virtual void RemoveNode(std::shared_ptr<ChNodeFEAbase> mnode) = 0;

    virtual bool IsNodeAdded(std::shared_ptr<ChNodeFEAbase> node) = 0;

    virtual ChFeaFieldData* GetNodeDataPointer(std::shared_ptr<ChNodeFEAbase> node) = 0;

    virtual unsigned int GetNumNodes() = 0;

    /// Get the number of degrees of freedom of the field per each node.
    virtual unsigned int GetNumFieldCoordsPosLevel() const = 0;
    virtual unsigned int GetNumFieldCoordsVelLevel() const = 0;

    unsigned int n_dofs;    ///< total degrees of freedom
    unsigned int n_dofs_w;  ///< total degrees of freedom, derivative (Lie algebra)
};

// -----------------------------------------------------------------------------


template <class T_data_per_node>
class ChFeaField : public ChFeaFieldBase {
public:
    using T_nodefield = T_data_per_node;

    ChFeaField() {}
    virtual ~ChFeaField() {}
    
    virtual void AddNode(std::shared_ptr<ChNodeFEAbase> mnode) override {
        node_data[mnode]; // just key, no need to provide value as it is default constructor of T_per_node
    }

    virtual void RemoveNode(std::shared_ptr<ChNodeFEAbase> mnode) override {
        node_data.erase(mnode);
    }

    virtual bool IsNodeAdded(std::shared_ptr<ChNodeFEAbase> node) override {
        return (node_data.find(node) != node_data.end());
    }

    virtual ChFeaFieldData* GetNodeDataPointer(std::shared_ptr<ChNodeFEAbase> node) override {
        return &node_data[node];
    }

    virtual unsigned int GetNumNodes() override {
        return node_data.size();
    }

    virtual unsigned int GetNumFieldCoordsPosLevel() const { return T_data_per_node::StaticGetNumCoordsPosLevel(); }
    virtual unsigned int GetNumFieldCoordsVelLevel() const { return T_data_per_node::StaticGetNumCoordsVelLevel(); }


    T_data_per_node& NodeData(std::shared_ptr<ChNodeFEAbase> node) {
        return node_data[node];
    }

    std::unordered_map<std::shared_ptr<ChNodeFEAbase>, T_data_per_node>& GetNodeDataMap() {
        return node_data;
    };

public: 
    // INTERFACE to ChPhysicsItem

    //virtual ChAABB GetTotalAABB() const override { ***TODO** };
    //virtual ChVector3d GetCenter() const override { ***TODO** };;

    virtual void Setup() override {
        n_dofs = 0;
        n_dofs_w = 0;

        for (auto& node : this->node_data) {
            // Set node offsets in state vectors (based on the offsets of the containing mesh)
            node.second.NodeSetOffsetPosLevel(GetOffset_x() + n_dofs);
            node.second.NodeSetOffsetVelLevel(GetOffset_w() + n_dofs_w);

            // Count the actual degrees of freedom (consider only nodes that are not fixed)
            if (!node.second.IsFixed()) {
                n_dofs   += T_data_per_node::StaticGetNumCoordsPosLevel();
                n_dofs_w += T_data_per_node::StaticGetNumCoordsVelLevel();
            }
        }
    };

    virtual void Update(double time, bool update_assets) override {
        // Parent class update
        ChPhysicsItem::Update(time, update_assets);
    }

    /// Set zero speed (and zero accelerations) in state without changing the position.
    virtual void ForceToRest() override {}
    virtual unsigned int GetNumCoordsPosLevel() override { return n_dofs; }
    virtual unsigned int GetNumCoordsVelLevel() override { return n_dofs_w; }

    /// From item's state to global state vectors y={x,v} pasting the states at the specified offsets.
    virtual void IntStateGather(const unsigned int off_x,  ///< offset in x state vector
        ChState& x,                ///< state vector, position part
        const unsigned int off_v,  ///< offset in v state vector
        ChStateDelta& v,           ///< state vector, speed part
        double& T                  ///< time
    ) {
        unsigned int local_off_x = 0;
        unsigned int local_off_v = 0;
        for (auto& node : this->node_data) {
            if (!node.second.IsFixed()) {
                node.second.NodeIntStateGather(off_x + local_off_x, x, off_v + local_off_v, v, T);
                local_off_x += T_data_per_node::StaticGetNumCoordsPosLevel();
                local_off_v += T_data_per_node::StaticGetNumCoordsVelLevel();
            }
        }
        T = GetChTime();
    }

    /// From global state vectors y={x,v} to  item's state (and update) fetching the states at the specified offsets.
    virtual void IntStateScatter(const unsigned int off_x,  ///< offset in x state vector
        const ChState& x,          ///< state vector, position part
        const unsigned int off_v,  ///< offset in v state vector
        const ChStateDelta& v,     ///< state vector, speed part
        const double T,            ///< time
        bool full_update           ///< perform complete update
    ) {
        unsigned int local_off_x = 0;
        unsigned int local_off_v = 0;
        for (auto& node : this->node_data) {
            if (!node.second.IsFixed()) {
                node.second.NodeIntStateScatter(off_x + local_off_x, x, off_v + local_off_v, v, T);
                local_off_x += T_data_per_node::StaticGetNumCoordsPosLevel();
                local_off_v += T_data_per_node::StaticGetNumCoordsVelLevel();
            }
        }
        Update(T, full_update);
    }

    /// From item's state acceleration to global acceleration vector
    virtual void IntStateGatherAcceleration(const unsigned int off_a,  ///< offset in a accel. vector
        ChStateDelta& a            ///< acceleration part of state vector derivative
    ) {
        unsigned int local_off_a = 0;
        for (auto& node : this->node_data) {
            if (!node.second.IsFixed()) {
                node.second.NodeIntStateGatherAcceleration(off_a + local_off_a, a);
                local_off_a += T_data_per_node::StaticGetNumCoordsVelLevel();
            }
        }
    }

    /// From global acceleration vector to item's state acceleration
    virtual void IntStateScatterAcceleration(const unsigned int off_a,  ///< offset in a accel. vector
        const ChStateDelta& a  ///< acceleration part of state vector derivative
    ) {
        unsigned int local_off_a = 0;
        for (auto& node : this->node_data) {
            if (!node.second.IsFixed()) {
                node.second.NodeIntStateScatterAcceleration(off_a + local_off_a, a);
                local_off_a += T_data_per_node::StaticGetNumCoordsVelLevel();
            }
        }
    }

    /// Computes x_new = x + Dt , using vectors at specified offsets.
    /// By default, when DOF = DOF_w, it does just the sum, but in some cases (ex when using quaternions
    /// for rotations) it could do more complex stuff, and children classes might overload it.
    virtual void IntStateIncrement(const unsigned int off_x,  ///< offset in x state vector
        ChState& x_new,            ///< state vector, position part, incremented result
        const ChState& x,          ///< state vector, initial position part
        const unsigned int off_v,  ///< offset in v state vector
        const ChStateDelta& Dv     ///< state vector, increment
    ) {
        unsigned int local_off_x = 0;
        unsigned int local_off_v = 0;
        for (auto& node : this->node_data) {
            if (!node.second.IsFixed()) {
                node.second.NodeIntStateIncrement(off_x + local_off_x, x_new, x, off_v + local_off_v, Dv);
                local_off_x += T_data_per_node::StaticGetNumCoordsPosLevel();
                local_off_v += T_data_per_node::StaticGetNumCoordsVelLevel();
            }
        }
    }

    /// Computes Dt = x_new - x, using vectors at specified offsets.
    /// By default, when DOF = DOF_w, it does just the difference of two state vectors, but in some cases (ex when using
    /// quaternions for rotations) it could do more complex stuff, and children classes might overload it.
    virtual void IntStateGetIncrement(const unsigned int off_x,  ///< offset in x state vector
        const ChState& x_new,      ///< state vector, final position part
        const ChState& x,          ///< state vector, initial position part
        const unsigned int off_v,  ///< offset in v state vector
        ChStateDelta& Dv           ///< state vector, increment. Here gets the result
    ) {
        unsigned int local_off_x = 0;
        unsigned int local_off_v = 0;
        for (auto& node : this->node_data) {
            if (!node.second.IsFixed()) {
                node.second.NodeIntStateGetIncrement(off_x + local_off_x, x_new, x, off_v + local_off_v, Dv);
                local_off_x += T_data_per_node::StaticGetNumCoordsPosLevel();
                local_off_v += T_data_per_node::StaticGetNumCoordsVelLevel();
            }
        }
    }

    /// Takes the F force term, scale and adds to R at given offset:
    ///    R += c*F
    virtual void IntLoadResidual_F(const unsigned int off,  ///< offset in R residual
        ChVectorDynamic<>& R,    ///< result: the R residual, R += c*F
        const double c           ///< a scaling factor
    ) {
        // nodes applied forces
        unsigned int local_off_v = 0;
        for (auto& node : this->node_data) {
            if (!node.second.IsFixed()) {
                node.second.NodeIntLoadResidual_F(off + local_off_v, R, c);
                local_off_v += T_data_per_node::StaticGetNumCoordsVelLevel();
            }
        }
    }

    /// Takes the M*v  term,  multiplying mass by a vector, scale and adds to R at given offset:
    ///    R += c*M*w
    virtual void IntLoadResidual_Mv(const unsigned int off,      ///< offset in R residual
        ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
        const ChVectorDynamic<>& w,  ///< the w vector
        const double c               ///< a scaling factor
    ) {
        // nodal masses
        unsigned int local_off_v = 0;
        for (auto& node : this->node_data) {
            if (!node.second.IsFixed()) {
                node.second.NodeIntLoadResidual_Mv(off + local_off_v, R, w, c);
                local_off_v += T_data_per_node::StaticGetNumCoordsVelLevel();
            }
        }
    }

    /// Adds the lumped mass to a Md vector, representing a mass diagonal matrix. Used by lumped explicit integrators.
    /// If mass lumping is impossible or approximate, adds scalar error to "error" parameter.
    ///    Md += c*diag(M)
    virtual void IntLoadLumpedMass_Md(const unsigned int off,  ///< offset in Md vector
        ChVectorDynamic<>& Md,  ///< result: Md vector, diagonal of the lumped mass matrix
        double& err,    ///< result: not touched if lumping does not introduce errors
        const double c  ///< a scaling factor
    ) {
        // nodal masses
        unsigned int local_off_v = 0;
        for (auto& node : this->node_data) {
            if (!node.second.IsFixed()) {
                node.second.NodeIntLoadLumpedMass_Md(off + local_off_v, Md, err, c);
                local_off_v += T_data_per_node::StaticGetNumCoordsVelLevel();
            }
        }
    }

    /// Prepare variables and constraints to accommodate a solution:
    virtual void IntToDescriptor(
        const unsigned int off_v,    ///< offset for \e v and \e R
        const ChStateDelta& v,       ///< vector copied into the \e q 'unknowns' term of the variables
        const ChVectorDynamic<>& R,  ///< vector copied into the \e F 'force' term of the variables
        const unsigned int off_L,    ///< offset for \e L and \e Qc
        const ChVectorDynamic<>& L,  ///< vector copied into the \e L 'lagrangian ' term of the constraints
        const ChVectorDynamic<>& Qc  ///< vector copied into the \e Qb 'constraint' term of the constraints
    ) {
        unsigned int local_off_v = 0;
        for (auto& node : this->node_data) {
            if (!node.second.IsFixed()) {
                node.second.NodeIntToDescriptor(off_v + local_off_v, v, R);
                local_off_v += T_data_per_node::StaticGetNumCoordsVelLevel();
            }
        }
    }

    /// After a solver solution, fetch values from variables and constraints into vectors:
    virtual void IntFromDescriptor(
        const unsigned int off_v,  ///< offset for \e v
        ChStateDelta& v,           ///< vector to where the \e q 'unknowns' term of the variables will be copied
        const unsigned int off_L,  ///< offset for \e L
        ChVectorDynamic<>& L       ///< vector to where \e L 'lagrangian ' term of the constraints will be copied
    ) {
        unsigned int local_off_v = 0;
        for (auto& node : this->node_data) {
            if (!node.second.IsFixed()) {
                node.second.NodeIntFromDescriptor(off_v + local_off_v, v);
                local_off_v += T_data_per_node::StaticGetNumCoordsVelLevel();
            }
        }
    }

    virtual void InjectVariables(ChSystemDescriptor& descriptor) {
        for (auto& node : this->node_data)
            node.second.InjectVariables(descriptor);
    }

protected:
    std::unordered_map<std::shared_ptr<ChNodeFEAbase>, T_data_per_node> node_data;

    unsigned int n_dofs;    ///< total degrees of freedom
    unsigned int n_dofs_w;  ///< total degrees of freedom, derivative (Lie algebra)
};


// Some ready-to-use fields.

class ChFeaFieldDataNONE : public ChFeaFieldDataGeneric<0> {};
class ChFeaFieldDataScalar : public ChFeaFieldDataGeneric<1> {};
class ChFeaFieldDataVector : public ChFeaFieldDataGeneric<3> {};
class ChFeaFieldDataTemperature : public ChFeaFieldDataScalar { 
    double& T() { return State()[0]; } 
    double& T_dt() { return StateDt()[0]; }
};
class ChFeaFieldDataElectricPotential : public ChFeaFieldDataScalar {
    double& V() { return State()[0]; }
    double& V_dt() { return StateDt()[0]; }
};

class ChFeaFieldNONE : public ChFeaField<ChFeaFieldDataNONE> {};
class ChFeaFieldScalar : public ChFeaField<ChFeaFieldDataScalar> {};
class ChFeaFieldVector : public ChFeaField<ChFeaFieldDataVector> {};
class ChFeaFieldTemperature : public ChFeaField<ChFeaFieldDataTemperature> {};
class ChFeaFieldElectricPotential : public ChFeaFieldScalar {};
class ChFeaFieldDisplacement3D : public ChFeaField<ChFeaFieldDataPos3D> {};

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------



/// Base class for the per-element properties of some material.
/// Inherit from this if you want to attach some property such as "float price; string name;" etc.
/// Note: better inherit from ChFeaPerElementDataKRM if you want that your elements have 
/// tangent stiffness and tangent damping, that are used in implicit integration.

class ChFeaPerElementDataNONE {
public:
    ChKRMBlock* GetKRM() { return nullptr; }
};

/// Class for the per-element properties of some material.
/// Inherit from this if you want to attach some property such as "float price; string name;", 
/// as well as for storing often-used data structures that persist together with the element and
/// that could be updated at each Update() of the element in order to speedup other computations later.
/// This contains the ChKRMBlock object, for automatic handling of tangent stiffness and tangent 
/// damping matrices that are used in implicit integration.

class ChFeaPerElementDataKRM : public ChFeaPerElementDataNONE {
public:
    ChKRMBlock* GetKRM() { return &Kmatr; }
private:
    ChKRMBlock Kmatr;
};


//------------------------------------------------------------------------------

/// Base class for all material properties. 
class ChFeaMaterialProperty {
public:
};


/// Base class for density in a continuum.
class ChFea3DDensity : ChFeaMaterialProperty {
public:
    ChFea3DDensity(double density = 1000) : m_density(density) {}

    virtual ~ChFea3DDensity() {}

    /// Set the density of the material, in kg/m^2.
    void SetDensity(double density) { m_density = density; }

    /// Get the density of the material, in kg/m^2.
    double GetDensity() const { return m_density; }

    //virtual void ArchiveOut(ChArchiveOut& archive_out);
    //virtual void ArchiveIn(ChArchiveIn& archive_in);

protected:
    double m_density;
};


/// Class for the basic properties of scalar fields P in 3D FEM problems
/// that can be described by Laplace PDEs of type
///    rho dP/dt + div [C] grad P = 0
class ChApi ChFea3DContinuumPoisson : public ChFea3DDensity {
public:
    ChFea3DContinuumPoisson() { ConstitutiveMatrix.setIdentity(3, 3); }

    virtual ~ChFea3DContinuumPoisson() {}

    /// Get the constitutive matrix [C] to compute the bilinear form in the weak formulation
    ChMatrixDynamic<>& GetConstitutiveMatrix() { return ConstitutiveMatrix; }

    /// Get the rho multiplier for the 'rho dP/dt term', if any (default, none)
    virtual double Get_DtMultiplier() { return 0; }

protected:
    ChMatrixDynamic<> ConstitutiveMatrix;  // constitutive matrix
};


/// Class for thermal fields, for FEA problems involving temperature, heat, etc.
/// It contains properties for thermal problems PDEs of the type 
///    c*density dT/dt + div [k] grad T = q_source
class ChApi ChFea3DMaterialThermal : public ChFea3DContinuumPoisson {
public:
    ChFea3DMaterialThermal() : k_thermal_conductivity(1), c_mass_specific_heat_capacity(1000) {}

    virtual ~ChFea3DMaterialThermal() {}

    /// Sets the k conductivity constant of the material,
    /// expressed in watts per meter kelvin [ W/(m K) ].
    /// Ex. (approx.): water = 0.6, aluminium = 200, steel = 50, plastics=0.9-0.2
    /// Sets the conductivity matrix as isotropic (diagonal k)
    void SetThermalConductivity(double mk) {
        k_thermal_conductivity = mk;
        ConstitutiveMatrix.setZero();
        ConstitutiveMatrix.fillDiagonal(k_thermal_conductivity);
    }

    /// Gets the k conductivity constant of the material,
    /// expressed in watts per meter kelvin (W/(m K)).
    double GetThermalConductivity() const { return k_thermal_conductivity; }

    /// Sets the c mass-specific heat capacity of the material,
    /// expressed as Joule per kg Kelvin [ J / (kg K) ]
    void SetSpecificHeatCapacity(double mc) { c_mass_specific_heat_capacity = mc; }

    /// Sets the c mass-specific heat capacity of the material,
    /// expressed as Joule per kg Kelvin [ J / (kg K) ]
    double GetSpecificHeatCapacity() const { return c_mass_specific_heat_capacity; }

    /// Get the k conductivity matrix
    ChMatrixDynamic<> GetConductivityMatrix() { return ConstitutiveMatrix; }

    /// override base: (the dT/dt term has multiplier rho*c with rho=density, c=heat capacity)
    virtual double Get_DtMultiplier() override { return m_density * c_mass_specific_heat_capacity; }

private:
    double k_thermal_conductivity;
    double c_mass_specific_heat_capacity;
};


/// Class for the basic properties of materials in an elastic continuum.

class ChFea3DMaterialStress : public ChFea3DDensity {
public:
    ChFea3DMaterialStress() {}

    virtual ~ChFea3DMaterialStress() {}

    /// Compute elastic stress from elastic strain.  Assuming the 2* factor in the last 3 values of strain Voigt notation.
    /// Assuming strain is Green-Lagrange tensor E, in Voigt notation. For small strains it coincides with espilon tensor.
    /// Assuming stress if Piola-Kirchhoff tensor S, in Voigt notation. 
    virtual void ComputeElasticStress(ChStressTensor<>& P_stress, const ChStrainTensor<>& E_strain) = 0;

    /// Computes the tangent modulus for a given strain. For linear elasticity it is the constant E matrix.
    /// Assuming strain is Green-Lagrange tensor, in Voigt notation. For small strains it coincides with espilon tensor.
    virtual void ComputeTangentModulus(ChMatrixNM<double, 6, 6>& StressStrainMatrix, const ChStrainTensor<>& strain) = 0;

    //virtual void ArchiveOut(ChArchiveOut& archive_out) override;  TODO
    //virtual void ArchiveIn(ChArchiveIn& archive_in) override; 
};

/// Class for properties of the 3D elasticity from StVenant-Kirchhoff model.
/// It is the simplest type of hyperelastic material. For S PiolaKirchhoff stress 
/// and E Green Lagrange strain tensors, it is S=C:E with C constant 4th order elastic tensor.
/// For small deformations it corresponds to the classical σ=C:ε. In Voigt notation, S=[C]*E,  σ=[C]*ε

class ChFea3DMaterialStressStVenant : public ChFea3DMaterialStress {
private:
    double m_E;                            ///< Young Modulus
    double m_poisson;                      ///< Poisson ratio
    double m_lamefirst;                    ///< Lame's first parameter
    ChMatrixNM<double, 6, 6> StressStrainMatrix;  ///< Elastic matrix in σ=[C]*ε (stored precomputed as it is constant, to speedup)

    double m_rayl_damping_alpha;  ///< Rayleigh damping coeff, M proportional
    double m_rayl_damping_beta;   ///< Rayleigh damping coeff, K proportional

public:
    ChFea3DMaterialStressStVenant(double young = 10000000, double poisson = 0.4, double density = 1000) {
        m_E = young;
        SetPoissonRatio(poisson);     // sets also Lamé, precomputes E matrix
        this->m_rayl_damping_alpha = 0;
        this->m_rayl_damping_beta = 0;
    }

    virtual ~ChFea3DMaterialStressStVenant() {}

    /// Set the Young elastic modulus, in Pa (N/m^2), as the ratio of the uniaxial
    /// stress over the uniaxial strain, for hookean materials.
    void SetYoungModulus(double E) {
        m_E = E;
        m_lamefirst = (m_poisson * m_E) / ((1 + m_poisson) * (1 - 2 * m_poisson));  // Lame's constant l
        UpdateStressStrainMatrix();                                                // updates Elasticity matrix
    }

    /// Get the Young elastic modulus, in Pa (N/m^2).
    double GetYoungModulus() const { return m_E; }

    /// Set the Poisson ratio, as v=-transverse_strain/axial_strain, so
    /// takes into account the 'squeezing' effect of materials that are pulled.
    /// Note: v=0.5 means perfectly incompressible material, that could give problems with some type of solvers.
    /// Setting v also changes G.
    void SetPoissonRatio(double v) {
        m_poisson = v;
        m_lamefirst = (m_poisson * m_E) / ((1 + m_poisson) * (1 - 2 * m_poisson));  // Lame's constant l
        UpdateStressStrainMatrix();                                                 // updates Elasticity matrix
    }

    /// Get the Poisson ratio, as v=-transverse_strain/axial_strain.
    double GetPoissonRatio() const { return m_poisson; }

    /// Set the shear modulus G, in Pa (N/m^2).
    /// Setting G also changes Poisson ratio v.
    void SetShearModulus(double G) {
        m_poisson = (m_E / (2 * G)) - 1;                                            // fixed G, E, get v
        m_lamefirst = (m_poisson * m_E) / ((1 + m_poisson) * (1 - 2 * m_poisson));  // Lame's constant l
        UpdateStressStrainMatrix();                                                 // updates Elasticity matrix
    }

    /// Get the shear modulus G, in Pa (N/m^2)
    double GetShearModulus() const { return m_E / (2 * (1 + m_poisson)); }

    /// Get Lamé first parameter (the second is shear modulus, so GetShearModulus() )
    double GetLameFirstParam() const { return m_lamefirst; }

    /// Get bulk modulus (increase of pressure for decrease of volume), in Pa.
    double GetBulkModulus() const { return m_E / (3 * (1 - 2 * m_poisson)); }

    /// Get P-wave modulus (if V=speed of propagation of a P-wave, then (M/density)=V^2 )
    double GetPWaveModulus() const { return m_E * ((1 - m_poisson) / (1 + m_poisson) * (1 - 2 * m_poisson)); }


    /// Compute elastic stress from elastic strain.  Assuming the 2* factor in the last 3 values of strain Voigt notation.
    /// Assuming strain is Green-Lagrange E tensor, in Voigt notation. For small strains it coincides with espilon tensor.
    /// Assuming stress if Piola-Kirchhoff S tensor, in Voigt notation. 
    virtual void ComputeElasticStress(ChStressTensor<>& stress, const ChStrainTensor<>& strain) override {
            double G = GetShearModulus();
            stress.XX() = strain.XX() * (m_lamefirst + 2 * G) + strain.YY() * m_lamefirst + strain.ZZ() * m_lamefirst;
            stress.YY() = strain.XX() * m_lamefirst + strain.YY() * (m_lamefirst + 2 * G) + strain.ZZ() * m_lamefirst;
            stress.ZZ() = strain.XX() * m_lamefirst + strain.YY() * m_lamefirst + strain.ZZ() * (m_lamefirst + 2 * G);
            stress.XY() = strain.XY() * G;
            stress.XZ() = strain.XZ() * G;
            stress.YZ() = strain.YZ() * G;
    }

    /// Computes the tangent modulus C for a given strain. Assuming the 2* factor in the last 3 values of strain Voigt notation.
    /// Since here we are using the StVenant model S=[C]*E, this is linear elasticity and C is constant matrix. 
    /// Assuming strain is Green-Lagrange E tensor, in Voigt notation. For small strains it coincides with espilon tensor.
    virtual void ComputeTangentModulus(ChMatrixNM<double, 6, 6>& C, const ChStrainTensor<>& strain) override {
        C = this->StressStrainMatrix;
    }


    /// Set the Rayleigh mass-proportional damping factor alpha, to
    /// build damping R as R=alpha*M + beta*K
    void SetRayleighDampingAlpha(double alpha) { m_rayl_damping_alpha = alpha; }

    /// Set the Rayleigh mass-proportional damping factor alpha, in R=alpha*M + beta*K
    double GetRayleighDampingAlpha() const { return m_rayl_damping_alpha; }

    /// Set the Rayleigh stiffness-proportional damping factor beta, to
    /// build damping R as R=alpha*M + beta*K
    void SetRayleighDampingBeta(double beta) { m_rayl_damping_beta = beta; }

    /// Set the Rayleigh stiffness-proportional damping factor beta, in R=alpha*M + beta*K
    double GetRayleighDampingBeta() const { return m_rayl_damping_beta; }

    //virtual void ArchiveOut(ChArchiveOut& archive_out) override;  TODO
    //virtual void ArchiveIn(ChArchiveIn& archive_in) override; 

private:

    /// This is just for optimization. The following code should go into ComputeTangentModulus(), but
    /// since E is constant, it is computed here into this->StressStrainMatrix every time one changes shear modulus, etc. via setters like SetShearModulus() etc.
    /// Later, the ComputeTangentModulus() can just copy from this->StressStrainMatrix, achieving higher speed. 
    void UpdateStressStrainMatrix() {
        StressStrainMatrix.setZero(6, 6);
        double G = GetShearModulus();
        // Fill the upper-left 3x3 block
        for (int i = 0; i < 3; ++i) {
            StressStrainMatrix(i, i) = m_lamefirst + 2.0 * G;
            for (int j = 0; j < 3; ++j) {
                if (i != j) {
                    StressStrainMatrix(i, j) = m_lamefirst;
                }
            }
        }
        // Fill the shear components
        StressStrainMatrix(3, 3) = G;
        StressStrainMatrix(4, 4) = G;
        StressStrainMatrix(5, 5) = G;
    }
};



// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------


// Type automation.
// Turning a std::tuple<ClassA,ClassB,..> into std::tuple<std::shared_ptr<ClassA>,std::shared_ptr<ClassB>,..>
template <typename Tuple>
struct tuple_as_sharedptr;
template <typename... Ts>
struct tuple_as_sharedptr<std::tuple<Ts...>> {
    using type = std::tuple<typename std::shared_ptr<Ts>...>;
};

// From a std::tuple<std::shared_ptr<Base>,std::shared_ptr<ClassDerivedFromBase>,..> 
// to std::array<std::shared_ptr<Base>,N> 
template <typename Base, typename Tuple, std::size_t... Is>
auto make_basearray_from_tuple_IMPL( const Tuple& t, std::index_sequence<Is...>) {
    return std::array<std::shared_ptr<Base>, std::tuple_size_v<Tuple>> { std::get<Is>(t)... };
}
template <typename Base, typename... Ts>
auto make_basearray_from_tuple(const std::tuple<Ts...>& t) {
    return make_basearray_from_tuple_IMPL<Base>(t, std::index_sequence_for<Ts...>{});
}


/// Base class for domains subject to a material model. They define (sub) regions of 
/// the mesh where the material has effect.
/// A ChFeaDomain has these components:
///   - a ChFeaMaterialProperty with properties for the domain material model 
///   - set of elements subject to the model
///   - set of fields needed for the material model
///   - additional data linked to finite elements and helper structures
/// Children classes should specialize this, possibly inheriting from ChFeaDomainImpl

class ChFeaDomain : public ChPhysicsItem {
public:
    virtual void AddElement(std::shared_ptr<ChFeaElement> melement) = 0;
    virtual void RemoveElement(std::shared_ptr<ChFeaElement> melement) = 0;
    virtual bool IsElementAdded(std::shared_ptr<ChFeaElement> melement) = 0;

    // This rewires all pointers and correctly set up the element_datamap
    virtual bool InitialSetup() = 0;

    // Get the total coordinates per each node (summing the dofs per each field) 
    virtual int GetNumPerNodeCoordsPosLevel() = 0;
    // Get the total coordinates per each node (summing the dofs per each field) 
    virtual int GetNumPerNodeCoordsVelLevel() { return GetNumPerNodeCoordsPosLevel(); }
    // Get the total number of nodes affected by this domain (some could be shared with other domains)
    virtual int GetNumNodes() = 0;

    /// Fills the S vector with the current field states S_i at the nodes of the element, with proper ordering.
    /// If the S vector size is not the proper size, it will be resized.
    virtual void GetStateBlock(std::shared_ptr<ChFeaElement> melement, ChVectorDynamic<>& S) = 0;

    /// Fills the dSdt vector with the current field states dS_i/dt at the nodes of the element, with proper ordering.
    /// If the dSdt vector size is not the proper size, it will be resized.
    virtual void GetStateBlockDt(std::shared_ptr<ChFeaElement> melement, ChVectorDynamic<>& dSdt) = 0;


protected:
    unsigned int n_dofs;    ///< total degrees of freedom of element materialpoint states (ex plasticity)
    unsigned int n_dofs_w;  ///< total degrees of freedom of element materialpoint states (ex plasticity), derivative (Lie algebra)
};


/// Base class for domains subject to a material model. They define (sub) regions of 
/// the mesh where the material has effect.
/// A ChFeaDomain has these components:
///   - a ChFeaMaterialProperty with properties for the domain material model 
///   - set of elements subject to the model
///   - set of fields needed for the material model
///   - additional data linked to finite elements and helper structures
/// Children classes should inherit and specialize this.
/// The T_... types are used to carry type info about 
/// the per-node or per-element or per-integration point data to instance.

template <
    typename T_per_node = std::tuple<ChFeaFieldScalar>, 
    typename T_per_matpoint = ChFeaFieldDataNONE,
    typename T_per_element = ChFeaPerElementDataNONE
>
class ChFeaDomainImpl : public ChFeaDomain {
public:

    class DataPerElement {
    public:
        DataPerElement(int n_matpoints = 0, int n_nodes = 0) :
            matpoints_data(n_matpoints), 
            nodes_data(n_nodes)
        {}
        T_per_element element_data;
        std::vector<T_per_matpoint> matpoints_data;
        std::vector<std::array<ChFeaFieldData*, std::tuple_size_v<T_per_node> > > nodes_data;
    };

    std::unordered_map<std::shared_ptr<ChFeaElement>, DataPerElement> element_datamap;

    std::array < std::shared_ptr<ChFeaFieldBase>, std::tuple_size_v<T_per_node> > fields;

    ChFeaDomainImpl(typename tuple_as_sharedptr<T_per_node>::type mfields) { fields = make_basearray_from_tuple<ChFeaFieldBase>(mfields); }

    DataPerElement& ElementData(std::shared_ptr<ChFeaElement> melement) {
        return element_datamap[melement];
    }

    // INTERFACE to ChFeaDomain
    //
    
    virtual void AddElement(std::shared_ptr<ChFeaElement> melement) override {
        return AddElement_impl(melement);
    }
    virtual void RemoveElement(std::shared_ptr<ChFeaElement> melement) override {
        return RemoveElement_impl(melement);
    }
    virtual bool IsElementAdded(std::shared_ptr<ChFeaElement> melement) override {
        return IsElementAdded_impl(melement);
    };

    // This rewires all pointers and correctly set up the element_datamap
    virtual bool InitialSetup() override {
        return InitialSetup_impl();
    }

    virtual int GetNumPerNodeCoordsPosLevel() override { return per_node_coords_pos; }
    virtual int GetNumPerNodeCoordsVelLevel() override { return per_node_coords_vel; }
    virtual int GetNumNodes() override { return num_nodes; }

    virtual void GetStateBlock(std::shared_ptr<ChFeaElement> melement, ChVectorDynamic<>& S) override {
        auto& elementdata = this->ElementData(melement);
        S.resize(this->GetNumPerNodeCoordsPosLevel()*melement->GetNumNodes());
        int off = 0;
        for (unsigned int i_node = 0; i_node < melement->GetNumNodes(); ++i_node) {
            for (unsigned int i_field = 0; i_field < this->fields.size(); ++i_field) {
                int ifieldsize = elementdata.nodes_data[i_node][i_field]->State().size();
                S.segment(off, ifieldsize) = elementdata.nodes_data[i_node][i_field]->State();
                off += ifieldsize;
            }
        }
    }
    virtual void GetStateBlockDt(std::shared_ptr<ChFeaElement> melement, ChVectorDynamic<>& dSdt) override {
        auto& elementdata = this->ElementData(melement);
        dSdt.resize(this->GetNumPerNodeCoordsVelLevel() * melement->GetNumNodes());
        int off = 0;
        for (unsigned int i_node = 0; i_node < melement->GetNumNodes(); ++i_node) {
            for (unsigned int i_field = 0; i_field < this->fields.size(); ++i_field) {
                int ifieldsize = elementdata.nodes_data[i_node][i_field]->State().size();
                dSdt.segment(off, ifieldsize) = elementdata.nodes_data[i_node][i_field]->StateDt();
                off += ifieldsize;
            }
        }
    }

    // FINITE ELEMENT MANAGERS

    /// For a given finite element, computes the internal loads Fi and set values in the Fi vector.
    /// It operates quadrature on the element, calling PointComputeInternalLoads(...) at each quadrature point.
    virtual void ElementComputeInternalLoads(std::shared_ptr<ChFeaElement> melement, 
                                            DataPerElement& data, 
                                            ChVectorDynamic<>& Fi
    ) {
        int quadorder = melement->GetMinQuadratureOrder();
        int numpoints = melement->GetNumQuadraturePoints(quadorder);
        int numelcoords = this->GetNumPerNodeCoordsVelLevel() * melement->GetNumNodes();
        Fi.setZero(numelcoords);
        ChMatrix33<> J;
        ChVector3d eta;
        double weight;
        for (int i_point = 0; i_point < numpoints; ++i_point) {
            melement->GetQuadraturePointWeight(quadorder, i_point, weight, eta); // get eta coords and weight at this i-th point
            double det_J = melement->ComputeJ(eta, J);
            double s = weight * det_J;
            PointComputeInternalLoads(melement, data, i_point, eta, s, Fi);
        }
    }

    /// For a given finite element, computes matrix H = Mfactor*M + Rfactor*dFi/dv + Kfactor*dFi/dx, as scaled sum of the tangent matrices M,R,K,:
    /// H = Mfactor*M + Rfactor*R + Kfactor*K. 
    /// Setting Mfactor=1 and Rfactor=Kfactor=0, it can be used to get just mass matrix, etc.
    /// It operates quadrature on the element, calling PointComputeKRMmatrices(...) at each quadrature point.
    virtual void ElementComputeKRMmatrices(std::shared_ptr<ChFeaElement> melement, DataPerElement& data, ChMatrixRef H,
                                            double Kfactor,
                                            double Rfactor = 0,
                                            double Mfactor = 0
    ) {
        int quadorder = melement->GetMinQuadratureOrder();
        int numpoints = melement->GetNumQuadraturePoints(quadorder);
        H.setZero(); // should be already of proper size
        ChMatrix33<> J;
        ChVector3d eta;
        double weight;
        for (int i_point = 0; i_point < numpoints; ++i_point) {
            melement->GetQuadraturePointWeight(quadorder, i_point, weight, eta); // get eta coords and weight at this i-th point
            double det_J = melement->ComputeJ(eta, J);
            double s = weight * det_J;
            PointComputeKRMmatrices(melement, 
                data, i_point, 
                eta, H, 
                Kfactor * s, 
                Rfactor * s, 
                Mfactor * s);
        }
    }

    /// For a given finite element, computes the lumped mass matrix.
    /// This falls back to the generic "Diagonal Scaling with Mass Preservation" approach, that 
    /// takes the full consistent mass matrix and reduces it to a diagonal by scaling the total mass.
    /// It works for all elements, 2D, 3D etc. It is not very efficient because it must compute the consitent mass matrix before.
    virtual void ElementIntLoadLumpedMass_Md(std::shared_ptr<ChFeaElement> melement, DataPerElement& data, 
                                            ChVectorDynamic<>& Md_i,
                                            double& error
    ) {
        int numelcoords = this->GetNumPerNodeCoordsVelLevel() * melement->GetNumNodes();
        Md_i.setZero(numelcoords);

        // Pass through the computation of the consistent mass matrix (generic approach but inefficient)
        ChMatrixDynamic<> M_consistent(numelcoords, numelcoords);

        ElementComputeKRMmatrices(melement, data, M_consistent, 0, 0, 1);

        // Calculate total mass (sum of all elements)
        double total_mass = M_consistent.sum();
        // Extract diagonal elements
        Eigen::VectorXd diag_vals = M_consistent.diagonal();
        // Calculate sum of diagonal elements
        double diag_sum = diag_vals.sum();
        // Check for zero or negative diagonal sum
        assert(diag_sum > 0.0);
        // Scale diagonal to preserve total mass
        double scale_factor = total_mass / diag_sum;

        error += std::abs(total_mass - diag_sum);
        Md_i = diag_vals * scale_factor;
    }



    // MATERIAL CONSTITUTIVE LAWS MUST IMPLEMENT THE FOLLOWING

    /// Computes the internal loads Fi for one quadrature point, except quadrature weighting, 
    /// and *ADD* the s-scaled result to Fi vector.
    /// For example, if internal load in discretized coords is 
    ///    F   = \sum (Foo*B')*w*det(J);  
    /// here you must compute  
    ///    Fi += (Foo*B')*s
    /// If the default quadrature is not good for you, then override ElementComputeInternalLoads() directly.
    virtual void PointComputeInternalLoads(std::shared_ptr<ChFeaElement> melement, 
                                            DataPerElement& data, 
                                            const int i_point, 
                                            ChVector3d& eta, 
                                            const double s, 
                                            ChVectorDynamic<>& Fi) = 0;

    /// Computes tangent matrix H for one quadrature point, except quadrature weighting, 
    /// and *ADD* the scaled result to H matrix.
    /// For example, if in discretized coords you have 
    ///    K   = \sum (B*E*B')*w*det(J); M=\sum(rho*N*N')*w*det(J); R = ...
    /// and since we assume H = Mfactor*K + Kfactor*K + Rfactor*R, then here you must compute  
    ///    H  += Mpfactor*(rho*N*N') + Kpfactor*(B*E*B') + Rpfactor*...
    /// If the default quadrature is not good for you, then override ElementComputeKRMmatrices() directly.
    virtual void PointComputeKRMmatrices(std::shared_ptr<ChFeaElement> melement, 
                                            DataPerElement& data,
                                            const int i_point, 
                                            ChVector3d& eta, 
                                            ChMatrixRef H,
                                            double Kpfactor,
                                            double Rpfactor = 0,
                                            double Mpfactor = 0) = 0;



    // INTERFACE to ChPhysicsItem
    //

    //virtual ChAABB GetTotalAABB() const override { ***TODO** };
    //virtual ChVector3d GetCenter() const override { ***TODO** };;

    virtual void Setup() override {
        n_dofs = 0;
        n_dofs_w = 0;

        for (auto& mel : this->element_datamap) {
            for (auto& matpoint : mel.second.matpoints_data) {
                // Set node offsets in state vectors (based on the offsets of the containing mesh)
                matpoint.NodeSetOffsetPosLevel(GetOffset_x() + n_dofs);
                matpoint.NodeSetOffsetVelLevel(GetOffset_w() + n_dofs_w);

                // Count the actual degrees of freedom (consider only nodes that are not fixed)
                if (!matpoint.IsFixed()) {
                    n_dofs += matpoint.GetNumCoordsPosLevel();
                    n_dofs_w += matpoint.GetNumCoordsVelLevel();
                }
            }
        }
    };

    virtual void SetupInitial() override {
        for (auto& mel : this->element_datamap) {
            mel.first->SetupInitial(this->GetSystem());
        }
    }

    virtual void Update(double time, bool update_assets) override {
        // Parent class update
        ChPhysicsItem::Update(time, update_assets);

        for (auto& mel : this->element_datamap) {
            mel.first->Update();
        }
    }

    /// Set zero speed (and zero accelerations) in state without changing the position.
    virtual void ForceToRest() override {}
    virtual unsigned int GetNumCoordsPosLevel() override { return n_dofs; }
    virtual unsigned int GetNumCoordsVelLevel() override { return n_dofs_w; }

    /// From item's state to global state vectors y={x,v} pasting the states at the specified offsets.
    virtual void IntStateGather(const unsigned int off_x,  ///< offset in x state vector
        ChState& x,                ///< state vector, position part
        const unsigned int off_v,  ///< offset in v state vector
        ChStateDelta& v,           ///< state vector, speed part
        double& T                  ///< time
    ) override {
        unsigned int local_off_x = 0;
        unsigned int local_off_v = 0;
        for (auto& mel : this->element_datamap) {
            for (auto& matpoint : mel.second.matpoints_data) {
                if (!matpoint.IsFixed()) {
                    matpoint.NodeIntStateGather(off_x + local_off_x, x, off_v + local_off_v, v, T);
                    local_off_x += matpoint.GetNumCoordsPosLevel();
                    local_off_v += matpoint.GetNumCoordsVelLevel();
                }
            }
        }
        T = GetChTime();
    }

    /// From global state vectors y={x,v} to element states (if any)  (and update) fetching the states at the specified offsets.
    virtual void IntStateScatter(const unsigned int off_x,  ///< offset in x state vector
        const ChState& x,          ///< state vector, position part
        const unsigned int off_v,  ///< offset in v state vector
        const ChStateDelta& v,     ///< state vector, speed part
        const double T,            ///< time
        bool full_update           ///< perform complete update
    ) override {
        unsigned int local_off_x = 0;
        unsigned int local_off_v = 0;
        for (auto& mel : this->element_datamap) {
            for (auto& matpoint : mel.second.matpoints_data) {
                if (!matpoint.IsFixed()) {
                    matpoint.NodeIntStateScatter(off_x + local_off_x, x, off_v + local_off_v, v, T);
                    local_off_x += matpoint.GetNumCoordsPosLevel();
                    local_off_v += matpoint.GetNumCoordsVelLevel();
                }
            }
        }
        Update(T, full_update);
    }

    /// From element states (if any) acceleration to global acceleration vector
    virtual void IntStateGatherAcceleration(const unsigned int off_a,  ///< offset in a accel. vector
        ChStateDelta& a            ///< acceleration part of state vector derivative
    ) override {
        unsigned int local_off_a = 0;
        for (auto& mel : this->element_datamap) {
            for (auto& matpoint : mel.second.matpoints_data) {
                if (!matpoint.IsFixed()) {
                    matpoint.NodeIntStateGatherAcceleration(off_a + local_off_a, a);
                    local_off_a += matpoint.GetNumCoordsVelLevel();
                }
            }
        }
    }

    /// From global acceleration vector to element states (if any) acceleration
    virtual void IntStateScatterAcceleration(const unsigned int off_a,  ///< offset in a accel. vector
        const ChStateDelta& a  ///< acceleration part of state vector derivative
    ) override {
        unsigned int local_off_a = 0;
        for (auto& mel : this->element_datamap) {
            for (auto& matpoint : mel.second.matpoints_data) {
                if (!matpoint.IsFixed()) {
                    matpoint.NodeIntStateScatterAcceleration(off_a + local_off_a, a);
                    local_off_a += matpoint.GetNumCoordsVelLevel();
                }
            }
        }
    }

    /// Computes x_new = x + Dt , using vectors at specified offsets.
    /// By default, when DOF = DOF_w, it does just the sum, but in some cases (ex when using quaternions
    /// for rotations) it could do more complex stuff, and children classes might overload it.
    virtual void IntStateIncrement(const unsigned int off_x,  ///< offset in x state vector
        ChState& x_new,            ///< state vector, position part, incremented result
        const ChState& x,          ///< state vector, initial position part
        const unsigned int off_v,  ///< offset in v state vector
        const ChStateDelta& Dv     ///< state vector, increment
    ) override {
        unsigned int local_off_x = 0;
        unsigned int local_off_v = 0;
        for (auto& mel : this->element_datamap) {
            for (auto& matpoint : mel.second.matpoints_data) {
                if (!matpoint.IsFixed()) {
                    matpoint.NodeIntStateIncrement(off_x + local_off_x, x_new, x, off_v + local_off_v, Dv);
                    local_off_x += matpoint.GetNumCoordsPosLevel();
                    local_off_v += matpoint.GetNumCoordsVelLevel();
                }
            }
        }
    }

    /// Computes Dt = x_new - x, using vectors at specified offsets.
    /// By default, when DOF = DOF_w, it does just the difference of two state vectors, but in some cases (ex when using
    /// quaternions for rotations) it could do more complex stuff, and children classes might overload it.
    virtual void IntStateGetIncrement(const unsigned int off_x,  ///< offset in x state vector
        const ChState& x_new,      ///< state vector, final position part
        const ChState& x,          ///< state vector, initial position part
        const unsigned int off_v,  ///< offset in v state vector
        ChStateDelta& Dv           ///< state vector, increment. Here gets the result
    ) override {
        unsigned int local_off_x = 0;
        unsigned int local_off_v = 0;
        for (auto& mel : this->element_datamap) {
            for (auto& matpoint : mel.second.matpoints_data) {
                if (!matpoint.IsFixed()) {
                    matpoint.NodeIntStateGetIncrement(off_x + local_off_x, x_new, x, off_v + local_off_v, Dv);
                    local_off_x += matpoint.GetNumCoordsPosLevel();
                    local_off_v += matpoint.GetNumCoordsVelLevel();
                }
            }
        }
    }

    /// Takes the F force term, scale and adds to R at given offset:
    ///    R += c*F
    virtual void IntLoadResidual_F(const unsigned int off,  ///< offset in R residual
        ChVectorDynamic<>& R,    ///< result: the R residual, R += c*F
        const double c           ///< a scaling factor
    ) override {
        // loads on element integration points states (if any)
        unsigned int local_off_v = 0;
        for (auto& mel : this->element_datamap) {
            for (auto& matpoint : mel.second.matpoints_data) {
                if (!matpoint.IsFixed()) {
                    matpoint.NodeIntLoadResidual_F(off + local_off_v, R, c);
                    local_off_v += matpoint.GetNumCoordsVelLevel();
                }
            }
        }

        // loads on nodes connected by the elements of the domain - here come the internal force vectors!!!
        for (auto& mel : this->element_datamap) {
            ChVectorDynamic<> Fi; // will be resized and zeroed by ElementComputeInternalLoads

            //****COMPUTATIONAL OVERHEAD - compute all the F loads here
            ElementComputeInternalLoads(mel.first, mel.second, Fi);

            // Fi is contiguous, so must store sparsely in R, per each node and per each field of node
            unsigned int stride = 0;
            for (unsigned int i_node = 0; i_node < mel.first->GetNumNodes(); i_node++) {
                for (unsigned int i_field = 0; i_field < this->fields.size(); ++i_field) {
                    ChFeaFieldData* mfielddata = mel.second.nodes_data[i_node][i_field];
                    int nfield_coords = this->fields[i_field]->GetNumFieldCoordsVelLevel();
                    if (!mfielddata->IsFixed()) {
                        R.segment(mfielddata->NodeGetOffsetVelLevel(), nfield_coords) += c * Fi.segment(stride, nfield_coords);
                    }
                    stride += nfield_coords;
                }
            }

        } // end loop on elements

    }

    /// Takes the M*w  term,  multiplying mass by a vector, scale and adds to R at given offset:
    ///    R += c*M*w
    virtual void IntLoadResidual_Mv(const unsigned int off,      ///< offset in R residual
        ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
        const ChVectorDynamic<>& w,  ///< the w vector
        const double c               ///< a scaling factor
    ) override {
        // M*w   caused by element states (if any) if they have some atomic mass
        unsigned int local_off_v = 0;
        for (auto& mel : this->element_datamap) {
            for (auto& matpoint : mel.second.matpoints_data) {
                if (!matpoint.IsFixed()) {
                    matpoint.NodeIntLoadResidual_Mv(off + local_off_v, R, w, c);
                    local_off_v += matpoint.GetNumCoordsVelLevel();
                }
            }
        }

        // M*w   caused by elements, where M is the mass matrix of the element
        for (auto& mel : this->element_datamap) {

            int numelcoords = this->GetNumPerNodeCoordsVelLevel() * mel.first->GetNumNodes();

            // Possible computational inefficiency: compute the consistent M matrix 
            ChMatrixDynamic<> M_i(numelcoords, numelcoords);
            ElementComputeKRMmatrices(mel.first, mel.second, M_i, 0, 0, 1);
            
            ChVectorDynamic<> W_i(numelcoords);
            W_i.setZero();
            // sparse w to contiguous W_i
            unsigned int stride = 0;
            for (unsigned int i_node = 0; i_node < mel.first->GetNumNodes(); i_node++) {
                for (unsigned int i_field = 0; i_field < this->fields.size(); ++i_field) {
                    ChFeaFieldData* mfielddata = mel.second.nodes_data[i_node][i_field];
                    int nfield_coords = this->fields[i_field]->GetNumFieldCoordsVelLevel();
                    if (!mfielddata->IsFixed()) {
                        W_i.segment(stride, nfield_coords) = w.segment(mfielddata->NodeGetOffsetVelLevel(), nfield_coords);
                    }
                    stride += nfield_coords;
                }
            }

            // R_i = c*M_i*W_i is contiguous, so must store sparsely in R, per each node and per each field of node
            stride = 0;
            for (unsigned int i_node = 0; i_node < mel.first->GetNumNodes(); i_node++) {
                for (unsigned int i_field = 0; i_field < this->fields.size(); ++i_field) {
                    ChFeaFieldData* mfielddata = mel.second.nodes_data[i_node][i_field];
                    int nfield_coords = this->fields[i_field]->GetNumFieldCoordsVelLevel();
                    if (!mfielddata->IsFixed()) {
                        R.segment(mfielddata->NodeGetOffsetVelLevel(), nfield_coords) += c * M_i.middleRows(stride, nfield_coords) * W_i;
                    }
                    stride += nfield_coords;
                }
            }


        }
    }

    /// Adds the lumped mass to a Md vector, representing a mass diagonal matrix. Used by lumped explicit integrators.
    /// If mass lumping is impossible or approximate, adds scalar error to "error" parameter.
    ///    Md += c*diag(M)
    virtual void IntLoadLumpedMass_Md(const unsigned int off,  ///< offset in Md vector
        ChVectorDynamic<>& Md,  ///< result: Md vector, diagonal of the lumped mass matrix
        double& err,    ///< result: not touched if lumping does not introduce errors
        const double c  ///< a scaling factor
    ) override {
        // Md   caused by element states (if any) if they have some 'mass'
        unsigned int local_off_v = 0;
        for (auto& mel : this->element_datamap) {
            for (auto& matpoint : mel.second.matpoints_data) {
                if (!matpoint.IsFixed()) {
                    matpoint.NodeIntLoadLumpedMass_Md(off + local_off_v, Md, err, c);
                    local_off_v += matpoint.GetNumCoordsVelLevel();
                }
            }
        }
        // Md   caused by elements, based on mass matrix of the element
        for (auto& mel : this->element_datamap) {

            // Possible computational inefficiency: compute the consistent M matrix
            ChVectorDynamic<> Md_i;
            ElementIntLoadLumpedMass_Md(mel.first, mel.second, Md_i, err);

            // Md_i is contiguous, so must store sparsely in Md, per each node and per each field of node
            unsigned int stride = 0;
            for (unsigned int i_node = 0; i_node < mel.first->GetNumNodes(); i_node++) {
                for (unsigned int i_field = 0; i_field < this->fields.size(); ++i_field) {
                    ChFeaFieldData* mfielddata = mel.second.nodes_data[i_node][i_field];
                    int nfield_coords = this->fields[i_field]->GetNumFieldCoordsVelLevel();
                    if (!mfielddata->IsFixed()) {
                        Md.segment(mfielddata->NodeGetOffsetVelLevel(), nfield_coords) += c * Md_i.segment(stride, nfield_coords);
                    }
                    stride += nfield_coords;
                }
            }

        }
    }

    /// Prepare variables and constraints to accommodate a solution:
    virtual void IntToDescriptor(
        const unsigned int off_v,    ///< offset for \e v and \e R
        const ChStateDelta& v,       ///< vector copied into the \e q 'unknowns' term of the variables
        const ChVectorDynamic<>& R,  ///< vector copied into the \e F 'force' term of the variables
        const unsigned int off_L,    ///< offset for \e L and \e Qc
        const ChVectorDynamic<>& L,  ///< vector copied into the \e L 'lagrangian ' term of the constraints
        const ChVectorDynamic<>& Qc  ///< vector copied into the \e Qb 'constraint' term of the constraints
    ) override {
        unsigned int local_off_v = 0;
        for (auto& mel : this->element_datamap) {
            for (auto& matpoint : mel.second.matpoints_data) {
                if (!matpoint.IsFixed()) {
                    matpoint.NodeIntToDescriptor(off_v + local_off_v, v, R);
                    local_off_v += matpoint.GetNumCoordsVelLevel();
                }
            }
        }
    }

    /// After a solver solution, fetch values from variables and constraints into vectors:
    virtual void IntFromDescriptor(
        const unsigned int off_v,  ///< offset for \e v
        ChStateDelta& v,           ///< vector to where the \e q 'unknowns' term of the variables will be copied
        const unsigned int off_L,  ///< offset for \e L
        ChVectorDynamic<>& L       ///< vector to where \e L 'lagrangian ' term of the constraints will be copied
    ) override {
        unsigned int local_off_v = 0;
        for (auto& mel : this->element_datamap) {
            for (auto& matpoint : mel.second.matpoints_data) {
                if (!matpoint.IsFixed()) {
                    matpoint.NodeIntFromDescriptor(off_v + local_off_v, v);
                    local_off_v += matpoint.GetNumCoordsVelLevel();
                }
            }
        }
    }

    virtual void InjectVariables(ChSystemDescriptor& descriptor) override {
        for (auto& mel : this->element_datamap) {
            for (auto& matpoint : mel.second.matpoints_data) {
                matpoint.InjectVariables(descriptor);
            }
        }
    }


    virtual void LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) override {
        for (auto& mel : this->element_datamap) {
            if (mel.second.element_data.GetKRM()) {

                //****COMPUTATIONAL OVERHEAD - compute all the tangent matrices here
                ElementComputeKRMmatrices(mel.first,
                                        mel.second,
                                        mel.second.element_data.GetKRM()->GetMatrix(),
                                        Kfactor, Rfactor, Mfactor);
            }

        }
    }

    virtual void InjectKRMMatrices(ChSystemDescriptor& descriptor) override {
        for (auto& mel : this->element_datamap) {
            if (mel.second.element_data.GetKRM())
                descriptor.InsertKRMBlock(mel.second.element_data.GetKRM());
        }
    }

    // UTILITY FUNCTIONS




private:
    void AddElement_impl(std::shared_ptr<ChFeaElement> melement) {
        element_datamap.insert(std::make_pair(melement, DataPerElement(melement->GetMinQuadratureOrder(), melement->GetNumNodes())));
    }

    void RemoveElement_impl(std::shared_ptr<ChFeaElement> melement) {
        element_datamap.erase(melement);
    }

    bool IsElementAdded_impl(std::shared_ptr<ChFeaElement> melement) {
        return (element_datamap.find(melement) != element_datamap.end());
    }

    // This rewires all pointers and correctly set up the element_datamap
    bool InitialSetup_impl() {
        num_nodes = 0;
        per_node_coords_pos = 0;
        per_node_coords_vel = 0;

        for (int i_field = 0; i_field < this->fields.size(); ++i_field) {
            this->fields[i_field]->Setup();
            per_node_coords_pos += this->fields[i_field]->GetNumFieldCoordsPosLevel();  
            per_node_coords_vel += this->fields[i_field]->GetNumFieldCoordsVelLevel();
        }

        for (auto& mel : this->element_datamap) {

            auto KRMblock = mel.second.element_data.GetKRM();
            std::vector<ChVariables*> mvars;

            // setup array of quadrature data
            if constexpr (std::is_same_v<T_per_matpoint, ChFeaFieldDataNONE>) {
                mel.second.matpoints_data.resize(0); // optimization to avoid wasting memory if domain falls back to ChFeaFieldNONE
            }
            else {
                mel.second.matpoints_data.resize(mel.first->GetMinQuadratureOrder());
            }
             

            // setup array of pointers to node field data 
            // (this array is here only for efficiency, otherwise each element should lookup the ChField maps every time
            // calling GetNodeDataPointer(mel.first->GetNode(i))
            mel.second.nodes_data.resize(mel.first->GetNumNodes());  
            for (unsigned int i = 0; i < mel.first->GetNumNodes(); ++i) {
                for (int i_field = 0; i_field < this->fields.size(); ++i_field) {
                    mel.second.nodes_data[i][i_field] = fields[i_field]->GetNodeDataPointer(mel.first->GetNode(i));
                    mvars.push_back(&(mel.second.nodes_data[i][i_field]->GetVariable()));
                }
            }
            if (KRMblock)
                KRMblock->SetVariables(mvars);

            num_nodes += mel.first->GetNumNodes();
        }



        return true;
    };
private:
    int per_node_coords_pos = 0;
    int per_node_coords_vel = 0;
    int num_nodes = 0;
};


/// Domain for FEA thermal analysis. It is based on a scalar temperature field.
/// In case you need thermoelasticity, use ChFeaDomainThermoelastic.

class ChFeaDomainThermal : public ChFeaDomainImpl<
    std::tuple<ChFeaFieldTemperature>,
    ChFeaFieldDataNONE,
    ChFeaPerElementDataKRM> {
public:
    ChFeaDomainThermal(std::shared_ptr<ChFeaFieldTemperature> mfield) 
        : ChFeaDomainImpl(mfield)
    {
        // attach a default material to simplify user side
        material = chrono_types::make_shared<ChFea3DMaterialThermal>();
    }

    /// Thermal properties of this domain (conductivity, 
    /// heat capacity constants etc.) 
    std::shared_ptr<ChFea3DMaterialThermal> material;
    
    // INTERFACES

    /// Computes the internal loads Fi for one quadrature point, except quadrature weighting, 
    /// and *ADD* the s-scaled result to Fi vector.
    virtual void PointComputeInternalLoads( std::shared_ptr<ChFeaElement> melement, 
                                            DataPerElement& data, 
                                            const int i_point, 
                                            ChVector3d& eta, 
                                            const double s, 
                                            ChVectorDynamic<>& Fi
    ) override {
        ChVectorDynamic<> T;
        this->GetStateBlock(melement, T);
        ChMatrixDynamic<> dNdX;
        ChRowVectorDynamic<> N;
        melement->ComputedNdX(eta, dNdX);
        melement->ComputeN(eta, N);
        // B = dNdX // in the lucky case of thermal problem, no need to build B because B is simply dNdX

        // We have:  Fi = - K * T;
        // where      K = sum (dNdX' * k * dNdX * w * |J|)
        // so we compute  Fi += -(dNdX' * k * dNdX * T) * s
        Fi += -(dNdX.transpose() * this->material->GetConductivityMatrix() * dNdX) * T * s;
    }

    /// Sets matrix H = Mfactor*M + Rfactor*dFi/dv + Kfactor*dFi/dx, as scaled sum of the tangent matrices M,R,K,:
    /// H = Mfactor*M + Rfactor*R + Kfactor*K. 
    /// Setting Mfactor=1 and Rfactor=Kfactor=0, it can be used to get just mass matrix, etc.
    virtual void PointComputeKRMmatrices(std::shared_ptr<ChFeaElement> melement, 
                                            DataPerElement& data, 
                                            const int i_point, 
                                            ChVector3d& eta, 
                                            ChMatrixRef H,
                                            double Kpfactor,
                                            double Rpfactor = 0,
                                            double Mpfactor = 0
    ) override {
        ChMatrixDynamic<> dNdX;
        ChRowVectorDynamic<> N;
        melement->ComputedNdX(eta, dNdX);
        melement->ComputeN(eta, N);
        // B = dNdX // in the lucky case of thermal problem, no need to build B because B is simply dNdX

        // K  matrix (jacobian d/dT of:    c dT/dt + div [C] grad T = f )  
        // K = sum (dNdX' * k * dNdX * w * |J|)
        H += Kpfactor * (dNdX.transpose() * this->material->GetConductivityMatrix() * dNdX) ;

        // R  matrix : (jacobian d / d\dot(T) of:    c dT / dt + div[C] grad T = f)
        // R = sum (N' * c*rho * N * w * |J|)
        if (Rpfactor && this->material->GetSpecificHeatCapacity()) {
            H += (Rpfactor * this->material->GetSpecificHeatCapacity() * this->material->GetDensity()) * (N.transpose() * N);
        }
    }
};


class ChFeaDomainElastic : public ChFeaDomainImpl<
    std::tuple<ChFeaFieldDisplacement3D>, // per each node
    ChFeaFieldDataNONE,   // per each GP
    ChFeaPerElementDataKRM> { // per each element
public:
    ChFeaDomainElastic(std::shared_ptr<ChFeaFieldDisplacement3D> melasticfield)
        : ChFeaDomainImpl( melasticfield )
    {
        // attach  default materials to simplify user side
        material = chrono_types::make_shared<ChFea3DMaterialStressStVenant>();
    }

    /// Elastic properties of this domain 
    std::shared_ptr<ChFea3DMaterialStress>  material;

    // INTERFACES

    /// Computes the internal loads Fi for one quadrature point, except quadrature weighting, 
    /// and *ADD* the s-scaled result to Fi vector.
    virtual void PointComputeInternalLoads(std::shared_ptr<ChFeaElement> melement,
                                        DataPerElement& data,
                                        const int i_point,
                                        ChVector3d& eta,
                                        const double s,
                                        ChVectorDynamic<>& Fi
    ) override {
        ChVectorDynamic<> T;
        this->GetStateBlock(melement, T);
        ChMatrixDynamic<> dNdX;
        ChRowVectorDynamic<> N;
        melement->ComputedNdX(eta, dNdX);
        melement->ComputeN(eta, N);

        // F deformation tensor = J_x * J_X^{-1}
        //   J_X: already available via element->ComputeJ()
        //   J_x: compute via   [x1|x2|x3|x4..]*dNde'
        ChMatrixDynamic<> Xhat(3, melement->GetNumNodes());
        for (unsigned int i = 0; i < melement->GetNumNodes(); ++i) {
            Xhat.block(0, i, 3, 1) = ((ChFeaFieldDataPos3D*)(data.nodes_data[i][0]))->GetPos().eigen();
        }
        ChMatrixDynamic<> dNde;
        melement->ComputedNde(eta, dNde);
        ChMatrix33d J_X_inv;
        melement->ComputeJinv(eta, J_X_inv);

        ChMatrix33d F = Xhat * dNde.transpose() * J_X_inv;

        // E  Green Lagrange tensor
        // E = 1/2( F*F' - I)
        ChMatrix33d E_strain33 = 0.5 * (F * F.transpose() - ChMatrix33d(1));

        ChStrainTensor<> E_strain; // Green Lagrange in Voigt notation
        E_strain.ConvertFromMatrix(E_strain33);
        E_strain.XY() *= 2; E_strain.XZ() *= 2; E_strain.YZ() *= 2; // engineering strains \gamma_xy = 2*\eps_xy

        ChStressTensor<> S_stress; // Piola Kirchhoff
        material->ComputeElasticStress(S_stress, E_strain);

        ChMatrixDynamic<> B(6, 3 * melement->GetNumNodes());
        this->ComputeB(B, dNdX, F);
        
        // We have:             Fi = - sum (B' * S * w * |J|)
        // so here we compute  Fi += - B' * S * s
        Fi += -(B.transpose() * S_stress) * s;
    }

    /// Sets matrix H = Mfactor*M + Rfactor*dFi/dv + Kfactor*dFi/dx, as scaled sum of the tangent matrices M,R,K,:
    /// H = Mfactor*M + Rfactor*R + Kfactor*K. 
    /// Setting Mfactor=1 and Rfactor=Kfactor=0, it can be used to get just mass matrix, etc.
    virtual void PointComputeKRMmatrices(std::shared_ptr<ChFeaElement> melement,
                                        DataPerElement& data,
                                        const int i_point,
                                        ChVector3d& eta,
                                        ChMatrixRef H,
                                        double Kpfactor,
                                        double Rpfactor = 0,
                                        double Mpfactor = 0
    ) override {
        ChMatrixDynamic<> dNdX;
        ChRowVectorDynamic<> N;
        melement->ComputedNdX(eta, dNdX);
        melement->ComputeN(eta, N);

        // F deformation tensor = J_x * J_X^{-1}
        //   J_X: already available via element->ComputeJ()
        //   J_x: compute via   [x1|x2|x3|x4..]*dNde'
        ChMatrixDynamic<> Xhat(3, melement->GetNumNodes());
        for (unsigned int i = 0; i < melement->GetNumNodes(); ++i) {
            Xhat.block(0, i, 3, 1) = ((ChFeaFieldDataPos3D*)(data.nodes_data[i][0]))->GetPos().eigen();
        }

        ChMatrixDynamic<> dNde;
        melement->ComputedNde(eta, dNde);

        ChMatrix33d J_X_inv;
        melement->ComputeJinv(eta, J_X_inv);

        ChMatrix33d F = Xhat * dNde.transpose() * J_X_inv;

        ChMatrixDynamic<> B(6, 3 * melement->GetNumNodes());
        this->ComputeB(B, dNdX, F);

        ChStrainTensor<> E_strain; // Green Lagrange in Voigt notation
        // ***TODO*** compute stress here - but not needed for this constant elasticity

        ChMatrix66<double> C;
        this->material->ComputeTangentModulus(C, E_strain);

        // K  matrix 
        // K = sum (B' * k * B  * w * |J|)  
        if (Kpfactor) {
            H += Kpfactor * (B.transpose() * C * B);
            // ***TODO*** add the geometric tangent stiffness
            // ***TODO*** rayleigh damping
        }

        // M  matrix : consistent mass matrix:   
        // M = sum (N' * rho * N * w * |J|)
        if (Mpfactor) {
            // If we had the "3 rows" form of the shape function matrix, say N_ where N_=[N(1)*I, N(2)*I, ], it would be
            //   M = sum (N_' * rho * N_ * w * |J|)     that is simply:
            //   H += (Mpfactor * this->material->GetDensity()) * (N_.transpose() * N_);
            // But the N_ matrix would be very sparse, so to speedup computation we unroll it and do:
            double scalar_factor = Mpfactor * this->material->GetDensity();
            for (int i = 0; i < N.cols(); i++) {
                for (int j = 0; j < N.cols(); j++) {
                    // Compute the scalar entry for the 8x8 scalar mass matrix
                    double scalar_entry = scalar_factor * N(i) * N(j);
                    int row_start = i * 3;
                    int col_start = j * 3;
                    H(row_start, col_start)         += scalar_entry; // xx
                    H(row_start + 1, col_start + 1) += scalar_entry; // yy  
                    H(row_start + 2, col_start + 2) += scalar_entry; // zz
                }
            }

            // ***TODO*** rayleigh damping
        }
    }

private:
    /// Utility: Compute  B as in  dE = B dx  where dE is variation in Green Lagrange strain (Voigt notation)
    /// and dx is the variation in spatial node coordinates (also works as  dE = B du  with du variation in displacements)
    void ComputeB(ChMatrixRef B, ChMatrixConstRef dNdX, ChMatrixConstRef F) {
        B.resize(6, 3 * dNdX.cols());
        B.setZero();
        for (int i = 0; i < dNdX.cols(); ++i) {
            // g = ∇₀ N_i = J_X⁻¹ ∇_ξ N_i = dNdX(:, i)
            //                          g₁* [F₁₁, F₂₁, F₃₁]
            B.block<1, 3>(0, i * 3) = dNdX(0, i) * F.block<3, 1>(0, 0).transpose();
            //                          g₂* [F₁₂, F₂₂, F₃₂]
            B.block<1, 3>(1, i * 3) = dNdX(1, i) * F.block<3, 1>(0, 1).transpose();
            //                          g₃* [F₁₃, F₂₃, F₃₃]
            B.block<1, 3>(2, i * 3) = dNdX(2, i) * F.block<3, 1>(0, 2).transpose();
            //                          g₂* [F₁₃, F₂₃, F₃₃]                             + g₃ * [F₁₂, F₂₂, F₃₂]
            B.block<1, 3>(3, i * 3) = dNdX(1, i) * F.block<3, 1>(0, 2).transpose() + dNdX(2, i) * F.block<3, 1>(0, 1).transpose();
            //                          g₁* [F₁₃, F₂₃, F₃₃]                             + g₃ * [F₁₁, F₂₁, F₃₁]
            B.block<1, 3>(4, i * 3) = dNdX(0, i) * F.block<3, 1>(0, 2).transpose() + dNdX(2, i) * F.block<3, 1>(0, 0).transpose();
            //                          g₁* [F₁₂, F₂₂, F₃₂]                             + g₂ * [F₁₁, F₂₁, F₃₁]
            B.block<1, 3>(5, i * 3) = dNdX(0, i) * F.block<3, 1>(0, 1).transpose() + dNdX(1, i) * F.block<3, 1>(0, 0).transpose();
        }
    }

};



class ChFeaDomainThermalElastic : public ChFeaDomainImpl<
    std::tuple<ChFeaFieldTemperature, ChFeaFieldDisplacement3D>, 
    ChFeaFieldDataNONE,
    ChFeaPerElementDataKRM> {
public:
    ChFeaDomainThermalElastic(std::shared_ptr<ChFeaFieldTemperature> mthermalfield, std::shared_ptr<ChFeaFieldDisplacement3D> melasticfield)
        : ChFeaDomainImpl({ mthermalfield, melasticfield })
    {
        // attach  default materials to simplify user side
        material_thermal = chrono_types::make_shared<ChFea3DMaterialThermal>();
        material_elasticity = chrono_types::make_shared<ChFea3DMaterialStressStVenant>();
    }

    /// Thermal properties of this domain (conductivity, 
    /// heat capacity constants etc.) 
    std::shared_ptr<ChFea3DMaterialThermal> material_thermal;
    std::shared_ptr<ChFea3DMaterialStress>  material_elasticity;

    // INTERFACES

    /// Computes the internal loads Fi for one quadrature point, except quadrature weighting, 
    /// and *ADD* the s-scaled result to Fi vector.
    virtual void PointComputeInternalLoads(std::shared_ptr<ChFeaElement> melement,
                                            DataPerElement& data,
                                            const int i_point,
                                            ChVector3d& eta,
                                            const double s,
                                            ChVectorDynamic<>& Fi
    ) override {}

    /// Sets matrix H = Mfactor*M + Rfactor*dFi/dv + Kfactor*dFi/dx, as scaled sum of the tangent matrices M,R,K,:
    /// H = Mfactor*M + Rfactor*R + Kfactor*K. 
    /// Setting Mfactor=1 and Rfactor=Kfactor=0, it can be used to get just mass matrix, etc.
    virtual void PointComputeKRMmatrices(std::shared_ptr<ChFeaElement> melement,
        DataPerElement& data,
        const int i_point,
        ChVector3d& eta,
        ChMatrixRef H,
        double Kpfactor,
        double Rpfactor = 0,
        double Mpfactor = 0
    ) override {}

};



// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------




/// @} chrono_fea

}  // end namespace fea

//CH_CLASS_VERSION(fea::ChFeaMaterialProperty, 0)


}  // end namespace chrono

#endif
