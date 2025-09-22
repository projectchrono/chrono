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
    ChFeaNodeXYZ(ChVector3d initial_pos = VNULL);
    ChFeaNodeXYZ(const ChFeaNodeXYZ& other);
    virtual ~ChFeaNodeXYZ() {}

    ChFeaNodeXYZ& operator=(const ChFeaNodeXYZ& other);

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
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;

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
    virtual void GetN(const ChVector3d eta, ChRowVectorDynamic<>& N) = 0;

    // Compute shape function parametric derivatives dN/d\eta at eta parametric coordinates.
    // Write shape functions dN_j(\eta)/d\eta_i in dNde, a matrix with GetNumNodes() columns, and n_rows = GetManifoldDimensions(). 
    // dNde will be resized if not of proper size.
    virtual void GetdNde(const ChVector3d eta, ChMatrixDynamic<>& dNde) = 0;

    // Compute shape function spatial derivatives dN/dX at eta parametric coordinates.
    // Write shape functions dN_j(eta)/dX_i in dNdX, a matrix with GetNumNodes() columns, and n_rows = GetSpatialDimensions().
    // dNde will be resized if not of proper size.
    // FALLBACK default implementation is dNdX = J^{-T} * dNde;  but if possible implement a more efficient ad hoc computation.
    virtual void GetdNdX(const ChVector3d eta, ChMatrixDynamic<>& dNdX) {
        ChMatrix33d temp_Jinv;
        ChMatrixDynamic<> temp_dNde;
        GetdNde(eta, temp_dNde);
        GetJinv(eta, temp_Jinv);
        dNdX.resize(GetSpatialDimensions(), GetNumNodes());
        dNdX = temp_Jinv.transpose() * temp_dNde;
    }

    // Compute Jacobian J, and returns its determinant. J is square with size = GetManifoldDimensions()
    virtual double GetJ(const ChVector3d eta, ChMatrix33d& J) = 0;

    // Compute Jacobian Jinv, and returns its determinant. Jinv is square with size = GetManifoldDimensions()
    virtual double GetJinv(const ChVector3d eta, ChMatrix33d& Jinv) = 0;

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


private:
    /// Initial setup (called once before start of simulation).
    /// This is used mostly to precompute matrices that do not change during the simulation, i.e. the local stiffness of
    /// each element, if any, the mass, etc.
    virtual void SetupInitial(ChSystem* system) {}

    friend class ChMesh;
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
    virtual void GetN(const ChVector3d eta, ChRowVectorDynamic<>& N) override {
        N.resize(GetNumNodes());
        N[0] = eta[0];
        N[1] = eta[1];
        N[2] = eta[2];
        N[3] = 1.0 - eta[0] - eta[1] - eta[2];
    };

    // Compute shape function material derivatives dN/d\eta at eta parametric coordinates.
    // Write shape functions dN_j(\eta)/d\eta_i in dNde, a matrix with 4 columns, and 3 rows. 
    virtual void GetdNde(const ChVector3d eta, ChMatrixDynamic<>& dNde) override {
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
    virtual void GetdNdX(const ChVector3d eta, ChMatrixDynamic<>& dNdX) override {
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
        dNdX(3, 0) = vai.z();   dNdX(3, 1) = vbi.z();   dNdX(3, 2) = vci.z();   dNdX(3, 3) = -vai.z() - vbi.z() - vci.z();
        //***TEST***
        ChMatrixDynamic<> test_dNdX(3, 4);
        ChFeaElement::GetdNdX(eta, test_dNdX);
        //**** 
    }

    // Compute Jacobian J, and returns its determinant. J is square 3x3
    virtual double GetJ(const ChVector3d eta, ChMatrix33d& J) override {
        ChVector3d x14 = *this->nodes[0] - *this->nodes[3];
        ChVector3d x24 = *this->nodes[1] - *this->nodes[3];
        ChVector3d x34 = *this->nodes[2] - *this->nodes[3];
        J(0, 0) = x14.x();  J(0, 1) = x24.x();  J(0, 2) = x34.x();
        J(1, 0) = x14.y();  J(1, 1) = x24.y();  J(1, 2) = x34.y();
        J(2, 0) = x14.z();  J(2, 1) = x24.z();  J(2, 2) = x34.z();
        return Vdot(x14, Vcross(x24, x34));
    }

    // Compute Jacobian Jinv, and returns its determinant. Jinv is square 3x3
    virtual double GetJinv(const ChVector3d eta, ChMatrix33d& Jinv) override {
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
        return (mtables->Weight[order - 1].size());
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
// -----------------------------------------------------------------------------

/// Base class for the per-node properties of some material,
/// for example it can contain some ChVariable. Used in ChFeaField.
class ChFeaPerNodeFieldData {
public:
    // Access state at node
    virtual ChVectorRef State() = 0;

    // Access state time derivative at node
    virtual ChVectorRef StateDt() = 0;

    /// Access the applied load term (ex. the atomic load/source in a Poisson equation).
    virtual ChVectorRef Load() = 0;

    /// Fix/release this node.
    /// If fixed, its state variables are not changed by the solver.
    virtual void SetFixed(bool fixed) = 0;
    virtual bool IsFixed() const = 0;

    /// Get the number of degrees of freedom.
    virtual unsigned int GetNumCoordsPosLevel() const = 0;
    virtual unsigned int GetNumCoordsVelLevel() const { return GetNumCoordsPosLevel(); }
    virtual unsigned int GetNumCoordsPosLevelActive() const { return GetNumCoordsPosLevel(); }
    virtual unsigned int GetNumCoordsVelLevelActive() const { return GetNumCoordsVelLevel(); }
    virtual bool IsAllCoordsActive() const { return true; }

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
class ChFeaPerNodeFieldDataGeneric : public ChFeaPerNodeFieldData {
public:
    ChFeaPerNodeFieldDataGeneric() :
        mvariables(T_nstates) {
    }

    virtual ChVectorRef State()  { return state; }

    virtual ChVectorRef StateDt()  { return state_dt; }

    virtual ChVectorRef Load() { return F; }

    virtual void SetFixed(bool fixed) {
        mvariables.SetDisabled(fixed);
    }
    virtual bool IsFixed() const {
        return mvariables.IsDisabled();
    }

    virtual unsigned int GetNumCoordsPosLevel() const { return T_nstates; }

    virtual void NodeIntStateGather(const unsigned int off_x,
                                    ChState& x,
                                    const unsigned int off_v,
                                    ChStateDelta& v,
                                    double& T) {
        x.segment(off_x, T_nstates) = state;
        v.segment(off_v, T_nstates) = state_dt;
    }

    virtual void NodeIntStateScatter(const unsigned int off_x,
                                    const ChState& x,
                                    const unsigned int off_v,
                                    const ChStateDelta& v,
                                    const double T) {
        state = x.segment(off_x, T_nstates);
        state_dt = v.segment(off_v, T_nstates);
    }

    virtual void NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {}
    virtual void NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {}

    virtual void NodeIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
        R.segment(off, T_nstates) += c * F;
    }

    virtual void NodeIntLoadResidual_Mv(const unsigned int off,
                                    ChVectorDynamic<>& R,
                                    const ChVectorDynamic<>& w,
                                    const double c) {
        //R(off) += c * mvariables.GetMass() * w(off);
    }

    virtual void NodeIntLoadLumpedMass_Md(const unsigned int off,
                                    ChVectorDynamic<>& Md,
                                    double& error,
                                    const double c) {
        //for (int i = 0; i < T_nstates; ++i) {
        //    Md(off + i) += c * mvariables.GetMass().(i, i);
        //}
        //error += std::abs(mvariables.GetMass().sum() - mvariables.GetMass().diagonal().sum());
    }

    virtual void NodeIntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) {
        mvariables.State() = v.segment(off_v, T_nstates);
        mvariables.Force() = R.segment(off_v, T_nstates);
    }

    virtual void NodeIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) {
        v.segment(off_v, T_nstates) = mvariables.State();
    }

    virtual void InjectVariables(ChSystemDescriptor& descriptor) {
        descriptor.InsertVariables(&mvariables);
    }

    
private:
    ChVariablesGeneric mvariables;
    ChVectorN<double, T_nstates> state;
    ChVectorN<double, T_nstates> state_dt;
    ChVectorN<double, T_nstates> F;
};



class ChFeaPerNodeFieldDataPos3D : public ChFeaPerNodeFieldData {
public:
    ChFeaPerNodeFieldDataPos3D()  {
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

    virtual void SetFixed(bool fixed) {
        mvariables.SetDisabled(fixed);
    }
    virtual bool IsFixed() const {
        return mvariables.IsDisabled();
    }

    virtual unsigned int GetNumCoordsPosLevel() const { return 3; }

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
        R(off) += c * mvariables.GetNodeMass() * w(off);
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


template <class T_data_per_node>
class ChFeaField : public ChPhysicsItem {
public:
    using T_nodefield = T_data_per_node;

    ChFeaField() {}
    virtual ~ChFeaField() {}
    
    virtual void AddNode(std::shared_ptr<ChNodeFEAbase> mnode) {
        node_data[mnode]; // just key, no need to provide value as it is default constructor of T_per_node
    }

    virtual void RemoveNode(std::shared_ptr<ChNodeFEAbase> mnode) {
        node_data.erase(mnode);
    }

    bool IsNodeAdded(std::shared_ptr<ChNodeFEAbase> node) {
        return (node_data.find(node) != node_data.end());
    }

    T_data_per_node& GetNodeData(std::shared_ptr<ChNodeFEAbase> node) {
        return node_data[node];
    }

    std::unordered_map<std::shared_ptr<ChNodeFEAbase>, T_data_per_node> node_data;

    unsigned int n_dofs;    ///< total degrees of freedom
    unsigned int n_dofs_w;  ///< total degrees of freedom, derivative (Lie algebra)

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
                n_dofs += node.second.GetNumCoordsPosLevelActive();
                n_dofs_w += node.second.GetNumCoordsVelLevelActive();
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
                local_off_x += node.second.GetNumCoordsPosLevelActive();
                local_off_v += node.second.GetNumCoordsVelLevelActive();
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
                local_off_x += node.second.GetNumCoordsPosLevelActive();
                local_off_v += node.second.GetNumCoordsVelLevelActive();
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
                local_off_a += node.second.GetNumCoordsVelLevelActive();
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
                local_off_a += node.second.GetNumCoordsVelLevelActive();
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
                local_off_x += node.second.GetNumCoordsPosLevelActive();
                local_off_v += node.second.GetNumCoordsVelLevelActive();
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
                local_off_x += node.second.GetNumCoordsPosLevelActive();
                local_off_v += node.second.GetNumCoordsVelLevelActive();
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
                local_off_v += node.second.GetNumCoordsVelLevelActive();
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
                local_off_v += node.second.GetNumCoordsVelLevelActive();
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
                local_off_v += node.second.GetNumCoordsVelLevelActive();
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
                local_off_v += node.second.GetNumCoordsVelLevelActive();
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
                local_off_v += node.second.GetNumCoordsVelLevelActive();
            }
        }
    }

    virtual void InjectVariables(ChSystemDescriptor& descriptor) {
        for (auto& node : this->node_data)
            node.second.InjectVariables(descriptor);
    }
};


// Some ready-to-use fields.

class ChFeaFieldScalar : public ChFeaField<ChFeaPerNodeFieldDataGeneric<1>> {};
class ChFeaFieldVector : public ChFeaField<ChFeaPerNodeFieldDataGeneric<3>> {};

class ChFeaFieldTemperature : public ChFeaFieldScalar {};
class ChFeaFieldElectricPotential : public ChFeaFieldScalar {};
class ChFeaFieldDisplacement3D : public ChFeaField<ChFeaPerNodeFieldDataPos3D> {};

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------


/// Base class for the per-material-point (Gauss quadrature point) properties of some material,
/// for example it can contain some ChVariable for plastic flow etc.
class ChFeaPerMaterialpointDataNONE {
};

/// Base class for the per-element properties of some material,
/// for example it can contain the ChKRMBlock object with tangent stiffness etc.
class ChFeaPerElementDataNONE {
};


//------------------------------------------------------------------------------

/// Base class for all material properties. 
class ChFeaMaterialProperty {
public:
};


/// Base class for density in a continuum.
class ChFea3DDensity : ChFeaMaterialProperty {
protected:
    double m_density;

public:
    ChFea3DDensity(double density = 1000) : m_density(density) {}
    ChFea3DDensity(const ChFea3DDensity& other) { m_density = other.m_density; }
    virtual ~ChFea3DDensity() {}

    /// Set the density of the material, in kg/m^2.
    void SetDensity(double density) { m_density = density; }

    /// Get the density of the material, in kg/m^2.
    double GetDensity() const { return m_density; }

    virtual void ArchiveOut(ChArchiveOut& archive_out);
    virtual void ArchiveIn(ChArchiveIn& archive_in);
};


/// Class for the basic properties of scalar fields P in 3D FEM problems
/// that can be described by Laplace PDEs of type
///    rho dP/dt + div [C] grad P = 0
class ChApi ChFea3DContinuumPoisson : public ChFea3DDensity {
public:
    using T_per_node = std::tuple<ChFeaFieldScalar>;
    using T_per_matpoint = ChFeaPerMaterialpointDataNONE;
    using T_per_element = ChFeaPerElementDataNONE;

protected:
    ChMatrixDynamic<> ConstitutiveMatrix;  // constitutive matrix

public:
    ChFea3DContinuumPoisson() { ConstitutiveMatrix.setIdentity(3, 3); }
    ChFea3DContinuumPoisson(const ChFea3DContinuumPoisson& other) : ChFea3DDensity(other) {
        ConstitutiveMatrix = other.ConstitutiveMatrix;
    }
    virtual ~ChFea3DContinuumPoisson() {}

    /// Get the constitutive matrix [C] to compute the bilinear form in the weak formulation
    ChMatrixDynamic<>& GetConstitutiveMatrix() { return ConstitutiveMatrix; }

    /// Get the rho multiplier for the 'rho dP/dt term', if any (default, none)
    virtual double Get_DtMultiplier() { return 0; }
};


/// Class for thermal fields, for FEA problems involving temperature, heat, etc.
/// It contains properties for thermal problems PDEs of the type 
///    c*density dT/dt + div [k] grad T = q_source
class ChApi ChFea3DMaterialThermal : public ChFea3DContinuumPoisson {
public:
    using T_per_node = std::tuple<ChFeaFieldTemperature>;
    using T_per_matpoint = ChFeaPerMaterialpointDataNONE;
    using T_per_element = ChFeaPerElementDataNONE;

private:
    double k_thermal_conductivity;
    double c_mass_specific_heat_capacity;

public:
    ChFea3DMaterialThermal() : k_thermal_conductivity(1), c_mass_specific_heat_capacity(1000) {}
    ChFea3DMaterialThermal(const ChFea3DMaterialThermal& other) : ChFea3DContinuumPoisson(other) {
        k_thermal_conductivity = other.k_thermal_conductivity;
        c_mass_specific_heat_capacity = other.c_mass_specific_heat_capacity;
    }
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

// Type automation.
// Turning a std::tuple<ClassA,ClassB,..> into std::tuple<ClassA::T_nodefield*>,ClassA::T_nodefield*,..>
template <typename Tuple>
struct tuple_as_field_ptrs;
template <typename... Ts>
struct tuple_as_field_ptrs<std::tuple<Ts...>> {
    using type = std::tuple<typename Ts::T_nodefield*...>;
};

// Data automation.
// From a std::tuple<std::shared_ptr<ClassA>,std::shared_ptr<ClassB>,..> and the pointer
// to a node std::shared_ptr<ChNodeFEAbase>, makes the tuple 
// std::tuple<ClassA::T_nodefield*>,ClassA::T_nodefield*,..>
template <typename Tuple, std::size_t... Is>
auto make_nodefields_pointer_tuple_IMPL(std::shared_ptr<ChNodeFEAbase> mnode, const Tuple& t, std::index_sequence<Is...>) {
    return std::make_tuple(&(std::get<Is>(t)->GetNodeData(mnode)) ...);
}
template <typename... Ts>
auto make_nodefields_pointer_tuple(std::shared_ptr<ChNodeFEAbase> mnode, const std::tuple<Ts...>& t) {
    return make_nodefields_pointer_tuple_IMPL(mnode, t, std::index_sequence_for<Ts...>{});
}

/// Base class for all material implementations for FEA
class ChFeaMaterialDomain {
public:
    virtual void AddElement(std::shared_ptr<ChFeaElement> melement) = 0;
    virtual void RemoveElement(std::shared_ptr<ChFeaElement> melement) = 0;
    virtual bool IsElementAdded(std::shared_ptr<ChFeaElement> melement) = 0;

    // This rewires all pointers and correctly set up the element_datamap
    virtual bool InitialSetup() = 0;
};

/// Class for all material implementations for FEA, and for 
/// defining (sub) regions of the mesh where the material has effect.
/// Usually it contains one or more ChFeaMaterialProperty objects.
/// The T_... types are used to carry type info about 
/// the per-node or per-element or per-integration point data to instance.

template <
    typename T_per_node = std::tuple<ChFeaFieldScalar>, 
    typename T_per_matpoint = ChFeaPerMaterialpointDataNONE, 
    typename T_per_element = ChFeaPerElementDataNONE
>
class ChFeaMaterialDomainImpl : public ChFeaMaterialDomain {
public:

    class DataPerElement {
    public:
        DataPerElement(int n_matpoints = 0, int n_nodes = 0) :
            matpoints_data(n_matpoints), nodes_data(n_nodes)
        {}
        T_per_element element_data;
        std::vector<T_per_matpoint> matpoints_data;
        std::vector<typename tuple_as_field_ptrs<T_per_node>::type> nodes_data;
    };
    std::unordered_map<std::shared_ptr<ChFeaElement>, DataPerElement> element_datamap;

    typename tuple_as_sharedptr<T_per_node>::type fields;

    ChFeaMaterialDomainImpl(typename tuple_as_sharedptr<T_per_node>::type mfields) { fields = mfields; }

    DataPerElement& GetElementData(std::shared_ptr<ChFeaElement> melement) {
        return element_datamap[melement];
    }

    // INTERFACES
    
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
        for (auto& mel : this->element_datamap) {
            // setup array of quadrature data
            mel.second.matpoints_data.resize(mel.first->GetMinQuadratureOrder()); // safety - should be already sized
            // setup array of pointers to node field data (this is for efficiency, otherwise each element should lookup the Chfield maps every time)
            mel.second.nodes_data.resize(mel.first->GetNumNodes());  // safety - should be already sized
            for (unsigned int i = 0; i < mel.first->GetNumNodes(); ++i) {
                mel.second.nodes_data[i] = make_nodefields_pointer_tuple(mel.first->GetNode(i), this->fields);
            }
        }
        return true;
    };
};


/// Domain for FEA thermal analysis. It is based on a scalar temperature field.
/// In case you need thermoelasticity, use ChFeaMaterialDomainThermoelastic.

class ChFeaMaterialDomainThermal : public ChFeaMaterialDomainImpl<
    std::tuple<ChFeaFieldTemperature>,
    ChFeaPerMaterialpointDataNONE,
    ChFeaPerElementDataNONE> {
public:
    ChFeaMaterialDomainThermal(std::shared_ptr<ChFeaFieldTemperature> mfield) 
        : ChFeaMaterialDomainImpl(mfield)
    {}

    /// Thermal properties of this domain (conductivity, 
    /// heat capacity constants etc.) 
    std::shared_ptr<ChFea3DMaterialThermal> material;
    
    //***TODO***
};


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------




/// @} chrono_fea

}  // end namespace fea

//CH_CLASS_VERSION(fea::ChFeaMaterialProperty, 0)


}  // end namespace chrono

#endif
