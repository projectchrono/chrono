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

#ifndef CHFIELD_H
#define CHFIELD_H

#include "chrono/core/ChApiCE.h"
#include "chrono/fea/ChFieldElement.h"
#include "chrono/fea/ChFieldData.h"
#include "chrono/fea/ChNodeFEAfieldXYZ.h"
#include "chrono/physics/ChPhysicsItem.h"

namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{


/// Base class for fields, more precisely discretized fields, that are maps of 
/// FEA nodes and associated states. Interfaces must be implemented by children classes.

class ChFieldBase : public ChPhysicsItem {
public:

    ChFieldBase() {}
    virtual ~ChFieldBase() {}

    virtual void AddNode(std::shared_ptr<ChNodeFEAbase> mnode) = 0;

    virtual void RemoveNode(std::shared_ptr<ChNodeFEAbase> mnode) = 0;

    virtual bool IsNodeAdded(std::shared_ptr<ChNodeFEAbase> node) = 0;

    virtual ChFieldDataState* GetNodeDataPointer(std::shared_ptr<ChNodeFEAbase> node) = 0;

    virtual unsigned int GetNumNodes() = 0;

    /// Get the number of degrees of freedom of the field per each node.
    virtual unsigned int GetNumFieldCoordsPosLevel() const = 0;
    virtual unsigned int GetNumFieldCoordsVelLevel() const = 0;

    /// Iterator for iterating on ChField nodes (virtual iterator, to be preferred to GetNodeDataPointer() 
    /// because the latter will be slower if iterating on all nodes, passing through hash map)
    class IteratorOnNodes {
    public:
        virtual ~IteratorOnNodes() = default;
        virtual std::pair<std::shared_ptr<ChNodeFEAbase>, ChFieldDataState&> get() = 0;
        virtual void next() = 0;
        virtual bool is_end() const = 0;
    };
    virtual std::unique_ptr<IteratorOnNodes> CreateIteratorOnNodes() = 0;

    unsigned int n_dofs;    ///< total degrees of freedom
    unsigned int n_dofs_w;  ///< total degrees of freedom, derivative (Lie algebra)
};


// -----------------------------------------------------------------------------


/// Templated class for implementing fields, more precisely discretized fields, that are maps of 
/// FEA nodes and associated states. See children classes for ready-to-use fields.

template <class T_data_per_node>
class ChField : public ChFieldBase {
public:
    using T_nodefield = T_data_per_node;

    ChField() {}
    virtual ~ChField() {}
    
    virtual void AddNode(std::shared_ptr<ChNodeFEAbase> mnode) override {
        node_data[mnode]; // just key, no need to provide value as it is default constructor of T_per_node
    }

    virtual void RemoveNode(std::shared_ptr<ChNodeFEAbase> mnode) override {
        node_data.erase(mnode);
    }

    virtual bool IsNodeAdded(std::shared_ptr<ChNodeFEAbase> node) override {
        return (node_data.find(node) != node_data.end());
    }

    virtual ChFieldDataState* GetNodeDataPointer(std::shared_ptr<ChNodeFEAbase> node) override {
        return &node_data[node];
    }

    virtual unsigned int GetNumNodes() override {
        return (unsigned int)node_data.size();
    }

    virtual unsigned int GetNumFieldCoordsPosLevel() const { return T_data_per_node::StaticGetNumCoordsPosLevel(); }
    virtual unsigned int GetNumFieldCoordsVelLevel() const { return T_data_per_node::StaticGetNumCoordsVelLevel(); }

    /// Iterator for iterating on ChField nodes (virtual iterator, to be preferred to GetNodeDataPointer() 
    /// because the latter will be slower if iterating on all nodes, passing through hash map)
    class IteratorOnNodes : public ChFieldBase::IteratorOnNodes {
        using InternalIterator = typename std::unordered_map<std::shared_ptr<ChNodeFEAbase>, T_data_per_node>::iterator;
        InternalIterator it_;
        InternalIterator end_;
    public:
        IteratorOnNodes(InternalIterator begin, InternalIterator end)
            : it_(begin), end_(end) {}

        std::pair<std::shared_ptr<ChNodeFEAbase>, ChFieldDataState&> get() override {
            ChFieldDataState& mdata = it_->second;
            std::shared_ptr<ChNodeFEAbase> mnode = it_->first;
            return std::pair<std::shared_ptr<ChNodeFEAbase>, ChFieldDataState&>(mnode, mdata);
        }
        void next() override {
            if (it_ != end_) ++it_;
        }
        bool is_end() const override {
            return it_ == end_;
        }
    };
    std::unique_ptr<ChFieldBase::IteratorOnNodes> CreateIteratorOnNodes() override {
        return std::make_unique<IteratorOnNodes>(node_data.begin(), node_data.end());
    }


    // Fast, but only if type of ChField<...> is known

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

            // Attention. Only ChFieldData without sub data can be used in fields, for performance reasons.
            assert(node.second.GetNthSubData(0)==nullptr);
            
            // Set node offsets in state vectors (based on the offsets of the containing mesh)
            node.second.DataSetOffsetPosLevel(GetOffset_x() + n_dofs);
            node.second.DataSetOffsetVelLevel(GetOffset_w() + n_dofs_w);

            // Count the actual degrees of freedom (consider only nodes that are not fixed)
            if (!node.second.IsFixed()) {
                n_dofs   += T_data_per_node::StaticGetNumCoordsPosLevel();
                n_dofs_w += T_data_per_node::StaticGetNumCoordsVelLevel();
            }
        }
    };

    virtual void Update(double time, UpdateFlags update_flags) override {
        // Parent class update
        ChPhysicsItem::Update(time, update_flags);
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
                node.second.DataIntStateGather(off_x + local_off_x, x, off_v + local_off_v, v, T);
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
        UpdateFlags update_flags   ///< perform complete update, or exclude visual assets, etc.
    ) {
        unsigned int local_off_x = 0;
        unsigned int local_off_v = 0;
        for (auto& node : this->node_data) {
            if (!node.second.IsFixed()) {
                node.second.DataIntStateScatter(off_x + local_off_x, x, off_v + local_off_v, v, T);
                local_off_x += T_data_per_node::StaticGetNumCoordsPosLevel();
                local_off_v += T_data_per_node::StaticGetNumCoordsVelLevel();
            }
        }
        Update(T, update_flags);
    }

    /// From item's state acceleration to global acceleration vector
    virtual void IntStateGatherAcceleration(const unsigned int off_a,  ///< offset in a accel. vector
        ChStateDelta& a            ///< acceleration part of state vector derivative
    ) {
        unsigned int local_off_a = 0;
        for (auto& node : this->node_data) {
            if (!node.second.IsFixed()) {
                node.second.DataIntStateGatherAcceleration(off_a + local_off_a, a);
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
                node.second.DataIntStateScatterAcceleration(off_a + local_off_a, a);
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
                node.second.DataIntStateIncrement(off_x + local_off_x, x_new, x, off_v + local_off_v, Dv);
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
                node.second.DataIntStateGetIncrement(off_x + local_off_x, x_new, x, off_v + local_off_v, Dv);
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
                node.second.DataIntLoadResidual_F(off + local_off_v, R, c);
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
                node.second.DataIntLoadResidual_Mv(off + local_off_v, R, w, c);
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
                node.second.DataIntLoadLumpedMass_Md(off + local_off_v, Md, err, c);
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
                node.second.DataIntToDescriptor(off_v + local_off_v, v, R);
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
                node.second.DataIntFromDescriptor(off_v + local_off_v, v);
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



//------------------------------------------------------------------------------------

// Some ready-to-use fields.


/// Generic scalar field. A single scalar state S_i , as ChFieldDataScalar, 
/// is attached to each node.

class ChFieldScalar : public ChField<ChFieldDataScalar> {};


/// Generic scalar field. A xyz vector state v_i, as ChFieldDataVector, 
/// is attached to each node.

class ChFieldVector : public ChField<ChFieldDataVector> {};


/// Temperature field. A scalar temperature T_i, as ChFieldDataTemperature, 
///is attached to each node. 

class ChFieldTemperature : public ChField<ChFieldDataTemperature> {};


/// Electric potential field. An electric potential V_i, as ChFieldDataElectricPotential, 
/// is attached to each node, as a scalar. 

class ChFieldElectricPotential : public ChField<ChFieldDataElectricPotential> {};


/// Field of spatial positions of moving material. This is used to represent the x_i
/// positions of nodes in deformed spatial configuration when doing large strain analysis.
/// (Note: for efficiency, it does not contain the displacement u=x-X, with X the
/// node reference position, but rather it contains x.)

class ChFieldDisplacement3D : public ChField<ChFieldDataPos3D> {
public:
    /// Add a node to the displacement 3D field (a field of spatial positions x of the deformed nodes).
    /// Note: respect to the base implementation, this also does an initialization:
    /// it sets the initial value x  to the current reference position X of the node. 
  
    virtual void AddNode(std::shared_ptr<ChNodeFEAbase> mnode) override {
        ChFieldDataPos3D& data = node_data[mnode]; // just key, no need to provide value as it is default constructor of T_per_node

        // initialize default x value
        if (auto posnode = std::dynamic_pointer_cast<ChNodeFEAfieldXYZ>(mnode)) {
            ChVector3d X = *posnode;
            data.SetPos(X); // x=X
        }
    }
};





/// @} chrono_fea

}  // end namespace fea


}  // end namespace chrono

#endif
