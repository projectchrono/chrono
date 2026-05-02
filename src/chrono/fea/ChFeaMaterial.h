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
#include "chrono/fea/ChFieldElement.h"
#include "chrono/fea/ChFieldData.h"
#include "chrono/fea/ChField.h"
#include "chrono/physics/ChNodeXYZ.h"
#include "chrono/solver/ChVariablesGeneric.h"
#include "chrono/solver/ChVariablesGenericDiagonalMass.h"

namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{



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
///   - a ChFieldMaterial with properties for the domain material model 
///   - set of elements subject to the model
///   - set of fields needed for the material model
///   - additional data linked to finite elements and helper structures
/// Children classes should specialize this, possibly inheriting from ChFeaDomainImpl

class ChFeaDomain : public ChPhysicsItem {
public:
    virtual void AddElement(std::shared_ptr<ChFieldElement> melement) = 0;
    virtual void RemoveElement(std::shared_ptr<ChFieldElement> melement) = 0;
    virtual bool IsElementAdded(std::shared_ptr<ChFieldElement> melement) = 0;

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
    virtual void GetStateBlock(std::shared_ptr<ChFieldElement> melement, ChVectorDynamic<>& S) = 0;

    /// Fills the dSdt vector with the current field states dS_i/dt at the nodes of the element, with proper ordering.
    /// If the dSdt vector size is not the proper size, it will be resized.
    virtual void GetStateBlockDt(std::shared_ptr<ChFieldElement> melement, ChVectorDynamic<>& dSdt) = 0;


protected:
    unsigned int n_dofs;    ///< total degrees of freedom of element materialpoint states (ex plasticity)
    unsigned int n_dofs_w;  ///< total degrees of freedom of element materialpoint states (ex plasticity), derivative (Lie algebra)
};


/// Base class for domains subject to a material model. They define (sub) regions of 
/// the mesh where the material has effect.
/// A ChFeaDomain has these components:
///   - a ChFieldMaterial with properties for the domain material model 
///   - set of elements subject to the model
///   - set of fields needed for the material model
///   - additional data linked to finite elements and helper structures
/// Children classes should inherit and specialize this.
/// The T_... types are used to carry type info about 
/// the per-node or per-element or per-integration point data to instance.

template <
    typename T_per_node = std::tuple<ChFieldScalar>, 
    typename T_per_matpoint = ChFieldDataNONE,
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
        std::vector<std::array<ChFieldData*, std::tuple_size_v<T_per_node> > > nodes_data;
    };

    std::unordered_map<std::shared_ptr<ChFieldElement>, DataPerElement> element_datamap;

    std::array < std::shared_ptr<ChFieldBase>, std::tuple_size_v<T_per_node> > fields;

    ChFeaDomainImpl(typename tuple_as_sharedptr<T_per_node>::type mfields) { fields = make_basearray_from_tuple<ChFieldBase>(mfields); }

    DataPerElement& ElementData(std::shared_ptr<ChFieldElement> melement) {
        return element_datamap[melement];
    }

    // INTERFACE to ChFeaDomain
    //
    
    virtual void AddElement(std::shared_ptr<ChFieldElement> melement) override {
        return AddElement_impl(melement);
    }
    virtual void RemoveElement(std::shared_ptr<ChFieldElement> melement) override {
        return RemoveElement_impl(melement);
    }
    virtual bool IsElementAdded(std::shared_ptr<ChFieldElement> melement) override {
        return IsElementAdded_impl(melement);
    };

    // This rewires all pointers and correctly set up the element_datamap
    virtual bool InitialSetup() override {
        return InitialSetup_impl();
    }

    virtual int GetNumPerNodeCoordsPosLevel() override { return per_node_coords_pos; }
    virtual int GetNumPerNodeCoordsVelLevel() override { return per_node_coords_vel; }
    virtual int GetNumNodes() override { return num_nodes; }

    virtual void GetStateBlock(std::shared_ptr<ChFieldElement> melement, ChVectorDynamic<>& S) override {
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
    virtual void GetStateBlockDt(std::shared_ptr<ChFieldElement> melement, ChVectorDynamic<>& dSdt) override {
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
    virtual void ElementComputeInternalLoads(std::shared_ptr<ChFieldElement> melement, 
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
    virtual void ElementComputeKRMmatrices(std::shared_ptr<ChFieldElement> melement, DataPerElement& data, ChMatrixRef H,
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
    virtual void ElementIntLoadLumpedMass_Md(std::shared_ptr<ChFieldElement> melement, DataPerElement& data, 
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
    virtual void PointComputeInternalLoads(std::shared_ptr<ChFieldElement> melement, 
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
    virtual void PointComputeKRMmatrices(std::shared_ptr<ChFieldElement> melement, 
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
                matpoint.DataSetOffsetPosLevel(GetOffset_x() + n_dofs);
                matpoint.DataSetOffsetVelLevel(GetOffset_w() + n_dofs_w);

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
                    matpoint.DataIntStateGather(off_x + local_off_x, x, off_v + local_off_v, v, T);
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
                    matpoint.DataIntStateScatter(off_x + local_off_x, x, off_v + local_off_v, v, T);
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
                    matpoint.DataIntStateGatherAcceleration(off_a + local_off_a, a);
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
                    matpoint.DataIntStateScatterAcceleration(off_a + local_off_a, a);
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
                    matpoint.DataIntStateIncrement(off_x + local_off_x, x_new, x, off_v + local_off_v, Dv);
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
                    matpoint.DataIntStateGetIncrement(off_x + local_off_x, x_new, x, off_v + local_off_v, Dv);
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
                    matpoint.DataIntLoadResidual_F(off + local_off_v, R, c);
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
                    ChFieldData* mfielddata = mel.second.nodes_data[i_node][i_field];
                    int nfield_coords = this->fields[i_field]->GetNumFieldCoordsVelLevel();
                    if (!mfielddata->IsFixed()) {
                        R.segment(mfielddata->DataGetOffsetVelLevel(), nfield_coords) += c * Fi.segment(stride, nfield_coords);
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
                    matpoint.DataIntLoadResidual_Mv(off + local_off_v, R, w, c);
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
                    ChFieldData* mfielddata = mel.second.nodes_data[i_node][i_field];
                    int nfield_coords = this->fields[i_field]->GetNumFieldCoordsVelLevel();
                    if (!mfielddata->IsFixed()) {
                        W_i.segment(stride, nfield_coords) = w.segment(mfielddata->DataGetOffsetVelLevel(), nfield_coords);
                    }
                    stride += nfield_coords;
                }
            }

            // R_i = c*M_i*W_i is contiguous, so must store sparsely in R, per each node and per each field of node
            stride = 0;
            for (unsigned int i_node = 0; i_node < mel.first->GetNumNodes(); i_node++) {
                for (unsigned int i_field = 0; i_field < this->fields.size(); ++i_field) {
                    ChFieldData* mfielddata = mel.second.nodes_data[i_node][i_field];
                    int nfield_coords = this->fields[i_field]->GetNumFieldCoordsVelLevel();
                    if (!mfielddata->IsFixed()) {
                        R.segment(mfielddata->DataGetOffsetVelLevel(), nfield_coords) += c * M_i.middleRows(stride, nfield_coords) * W_i;
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
                    matpoint.DataIntLoadLumpedMass_Md(off + local_off_v, Md, err, c);
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
                    ChFieldData* mfielddata = mel.second.nodes_data[i_node][i_field];
                    int nfield_coords = this->fields[i_field]->GetNumFieldCoordsVelLevel();
                    if (!mfielddata->IsFixed()) {
                        Md.segment(mfielddata->DataGetOffsetVelLevel(), nfield_coords) += c * Md_i.segment(stride, nfield_coords);
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
                    matpoint.DataIntToDescriptor(off_v + local_off_v, v, R);
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
                    matpoint.DataIntFromDescriptor(off_v + local_off_v, v);
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
    void AddElement_impl(std::shared_ptr<ChFieldElement> melement) {
        element_datamap.insert(std::make_pair(melement, DataPerElement(melement->GetMinQuadratureOrder(), melement->GetNumNodes())));
    }

    void RemoveElement_impl(std::shared_ptr<ChFieldElement> melement) {
        element_datamap.erase(melement);
    }

    bool IsElementAdded_impl(std::shared_ptr<ChFieldElement> melement) {
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
            if constexpr (std::is_same_v<T_per_matpoint, ChFieldDataNONE>) {
                mel.second.matpoints_data.resize(0); // optimization to avoid wasting memory if domain falls back to ChFieldNONE
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
    std::tuple<ChFieldTemperature>,
    ChFieldDataNONE,
    ChFeaPerElementDataKRM> {
public:
    ChFeaDomainThermal(std::shared_ptr<ChFieldTemperature> mfield) 
        : ChFeaDomainImpl(mfield)
    {
        // attach a default material to simplify user side
        material = chrono_types::make_shared<ChMaterial3DThermal>();
    }

    /// Thermal properties of this domain (conductivity, 
    /// heat capacity constants etc.) 
    std::shared_ptr<ChMaterial3DThermal> material;
    
    // INTERFACES

    /// Computes the internal loads Fi for one quadrature point, except quadrature weighting, 
    /// and *ADD* the s-scaled result to Fi vector.
    virtual void PointComputeInternalLoads( std::shared_ptr<ChFieldElement> melement, 
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
    virtual void PointComputeKRMmatrices(std::shared_ptr<ChFieldElement> melement, 
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
    std::tuple<ChFieldDisplacement3D>, // per each node
    ChFieldDataNONE,   // per each GP
    ChFeaPerElementDataKRM> { // per each element
public:
    ChFeaDomainElastic(std::shared_ptr<ChFieldDisplacement3D> melasticfield)
        : ChFeaDomainImpl( melasticfield )
    {
        // attach  default materials to simplify user side
        material = chrono_types::make_shared<ChMaterial3DStressStVenant>();
    }

    /// Elastic properties of this domain 
    std::shared_ptr<ChMaterial3DStress>  material;

    // INTERFACES

    /// Computes the internal loads Fi for one quadrature point, except quadrature weighting, 
    /// and *ADD* the s-scaled result to Fi vector.
    virtual void PointComputeInternalLoads(std::shared_ptr<ChFieldElement> melement,
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
            Xhat.block(0, i, 3, 1) = ((ChFieldDataPos3D*)(data.nodes_data[i][0]))->GetPos().eigen();
        }
        ChMatrixDynamic<> dNde;
        melement->ComputedNde(eta, dNde);

        ChMatrix33d J_X_inv;
        melement->ComputeJinv(eta, J_X_inv);

        ChMatrix33d F = Xhat * dNde.transpose() * J_X_inv;  

        // Compute  2nd Piola-Kirchhoff tensor in Voigt notation using the constitutive relation of material
        ChStressTensor<> S_stress; 
        material->ComputeStress(S_stress, F);

        ChMatrixDynamic<> B(6, 3 * melement->GetNumNodes());
        this->ComputeB(B, dNdX, F);
        
        // We have:             Fi = - sum (B' * S * w * |J|)
        // so here we compute  Fi += - B' * S * s
        Fi += -(B.transpose() * S_stress) * s;
    }

    /// Sets matrix H = Mfactor*M + Rfactor*dFi/dv + Kfactor*dFi/dx, as scaled sum of the tangent matrices M,R,K,:
    /// H = Mfactor*M + Rfactor*R + Kfactor*K. 
    /// Setting Mfactor=1 and Rfactor=Kfactor=0, it can be used to get just mass matrix, etc.
    virtual void PointComputeKRMmatrices(std::shared_ptr<ChFieldElement> melement,
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
            Xhat.block(0, i, 3, 1) = ((ChFieldDataPos3D*)(data.nodes_data[i][0]))->GetPos().eigen();
        }

        ChMatrixDynamic<> dNde;
        melement->ComputedNde(eta, dNde);

        ChMatrix33d J_X_inv;
        melement->ComputeJinv(eta, J_X_inv);

        ChMatrix33d F = Xhat * dNde.transpose() * J_X_inv;

        ChMatrixDynamic<> B(6, 3 * melement->GetNumNodes());
        this->ComputeB(B, dNdX, F);

        // Compute tangent modulus (assumed: dP=[C]dE with P  2nd Piola-Kirchhoff, E Green-Lagrange)
        ChMatrix66<double> C;
        this->material->ComputeTangentModulus(C, F);

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
    std::tuple<ChFieldTemperature, ChFieldDisplacement3D>, 
    ChFieldDataNONE,
    ChFeaPerElementDataKRM> {
public:
    ChFeaDomainThermalElastic(std::shared_ptr<ChFieldTemperature> mthermalfield, std::shared_ptr<ChFieldDisplacement3D> melasticfield)
        : ChFeaDomainImpl({ mthermalfield, melasticfield })
    {
        // attach  default materials to simplify user side
        material_thermal = chrono_types::make_shared<ChMaterial3DThermal>();
        material_elasticity = chrono_types::make_shared<ChMaterial3DStressStVenant>();
    }

    /// Thermal properties of this domain (conductivity, 
    /// heat capacity constants etc.) 
    std::shared_ptr<ChMaterial3DThermal> material_thermal;
    std::shared_ptr<ChMaterial3DStress>  material_elasticity;

    // INTERFACES

    /// Computes the internal loads Fi for one quadrature point, except quadrature weighting, 
    /// and *ADD* the s-scaled result to Fi vector.
    virtual void PointComputeInternalLoads(std::shared_ptr<ChFieldElement> melement,
                                            DataPerElement& data,
                                            const int i_point,
                                            ChVector3d& eta,
                                            const double s,
                                            ChVectorDynamic<>& Fi
    ) override {}

    /// Sets matrix H = Mfactor*M + Rfactor*dFi/dv + Kfactor*dFi/dx, as scaled sum of the tangent matrices M,R,K,:
    /// H = Mfactor*M + Rfactor*R + Kfactor*K. 
    /// Setting Mfactor=1 and Rfactor=Kfactor=0, it can be used to get just mass matrix, etc.
    virtual void PointComputeKRMmatrices(std::shared_ptr<ChFieldElement> melement,
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

//CH_CLASS_VERSION(fea::ChFieldMaterial, 0)


}  // end namespace chrono

#endif
