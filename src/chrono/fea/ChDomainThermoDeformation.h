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

#ifndef CHDOMAINTHERMODEFORMATION_H
#define CHDOMAINTHERMODEFORMATION_H

#include "chrono/fea/ChDomain.h"
#include "chrono/fea/ChDomainDeformation.h"
#include "chrono/fea/ChMaterial3DThermalStress.h"
#include "chrono/fea/ChMaterial3DThermalLinear.h"
#include "chrono/fea/ChMaterial3DStressStVenant.h"
#include "chrono/fea/ChVisualDataExtractor.h"
#include "chrono/fea/ChLoaderGravity.h"
#include "chrono/fea/ChFieldElementLoadableVolume.h"

namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{



/// Auxiliary scratch data stored per each material point during the ChDomainThermoDeformation
/// computation. This can be plotted in postprocessing, etc.

class ChFieldDataAuxiliaryThermoDeformation : public ChFieldDataNONE {
public:
    ChMatrix33d F;      /// total deformation gradient tensor F = F_m * F_t 
    ChMatrix33d F_t_inv;/// thermal deformation gradient tensor, inverse
    ChMatrix33d F_m;    /// mechanical deformation gradient tensor
    ChVector3d  q_flux; /// heat flux
    
    // by the way this could have been also: 
    // ChFieldDataAuxiliaryThermal aux_thermal;      // with "q_flux"
    // ChFieldDataAuxiliaryDeformation aux_deform;   // with "F" that is F_m in our case
    // ChMatrix33d F_t_inv; 
    // ChMatrix33d F;
};

/// Domain for FEA nonlinear finite strain with thermal coupling. It is based on a vector field,
/// (ChFieldDisplacement3D, that is used to store x, the absolute spatial position of nodes, NOT
/// the displacement from the material reference position, d=x-X, as in some software) and
/// a ChFieldTemperature. It solves the transient Poisson thermal equation together with 
/// structural dynamics. 
/// Not copyable: don't do __declspec(dllexport), ie. "class ChApi ChDomainThermoDeformation...", just keep all in .h

class ChDomainThermoDeformation : public ChDomainImpl<
    std::tuple<ChFieldTemperature, ChFieldDisplacement3D>, 
    ChFieldDataAuxiliaryThermoDeformation,
    ChElementDataKRM> {
public:

    using Base = ChDomainImpl<
        std::tuple<ChFieldTemperature, ChFieldDisplacement3D>,
        ChFieldDataAuxiliaryThermoDeformation,   
        ChElementDataKRM
    >;
    using DataPerElement = typename Base::DataPerElement;

    ChDomainThermoDeformation(std::shared_ptr<ChFieldTemperature> mthermalfield, std::shared_ptr<ChFieldDisplacement3D> melasticfield)
        : Base({ mthermalfield, melasticfield })
    {
        // attach  default materials to simplify user side
        material_thermalstress = chrono_types::make_shared<ChMaterial3DThermalStress>();
        material_thermalstress->material_stress  = chrono_types::make_shared<ChMaterial3DStressStVenant>();
        material_thermalstress->material_thermal = chrono_types::make_shared<ChMaterial3DThermalLinear>();

        automatic_gravity_load = false;
        num_points_gravity = 1;
        gravity_G_acceleration.Set(0, -9.81, 0);
    }

    /// Thermal properties of this domain (conductivity, 
    /// heat capacity constants etc.) 
    std::shared_ptr<ChMaterial3DThermalStress> material_thermalstress;

    // INTERFACES

    /// Computes the internal loads Fi for one quadrature point, except quadrature weighting, 
    /// and *ADD* the s-scaled result to Fi vector. Fi contains loads ordered as {Fi_thermal; Fi_deform}
    virtual void PointComputeInternalLoads(std::shared_ptr<ChFieldElement> melement,
                                            DataPerElement& data,
                                            const int i_point,
                                            ChVector3d& eta,
                                            const double s,
                                            ChVectorDynamic<>& Fi
    ) override {
        const unsigned int i_field_temp = 0;  // temperature  field is 1st 
        const unsigned int i_field_displ = 1; // displacement field is 2nd 

        unsigned int n_ele_coords_thermal = melement->GetNumNodes();
        unsigned int n_ele_coords_deform = 3 * melement->GetNumNodes();

        // Compute shape functions N at eta, and their material derivatives
        ChMatrixDynamic<> dNdX;
        ChRowVectorDynamic<> N;
        melement->ComputedNdX(eta, dNdX);
        melement->ComputeN(eta, N);

        // Compute the vector  T_h = [T_1, T_2, .. T_n] with discrete values of temperatures at nodes
        ChMatrixDynamic<> T_hm;
        this->GetFieldPackedStateBlock(melement, data, T_hm, i_field_temp);
        ChRowVectorDynamic<> T_h(T_hm.row(0));

        // Temperature at point  T = T_h * N'
        double T = T_h * N.transpose(); 
         
        // Compute the matrix  x_hh = [x1|x2|x3|x4..] with discrete values of spatial positions at nodes
        ChMatrixDynamic<> x_hh(3, melement->GetNumNodes());
        this->GetFieldPackedStateBlock(melement, data, x_hh, i_field_displ);

        ChMatrixDynamic<> dNde;
        melement->ComputedNde(eta, dNde);

        // F deformation tensor = J_x * J_X^{-1}
        //   J_X: already available via element->ComputeJ()
        //   J_x: compute via   [x1|x2|x3|x4..]*dNde'

        ChMatrix33d J_X_inv;
        melement->ComputeJinv(eta, J_X_inv);

        ChMatrix33d J_x = x_hh * dNde.transpose();

        ChMatrix33d F = J_x * J_X_inv;

        // Thermal deformation: 
        // alpha_t = 1 + alpha *(T - T_0)
        double alpha_t = 1 + material_thermalstress->GetThermalExpansionCoefficient() * (T - material_thermalstress->GetRestTemperature());
        // Isotropic F_t initial deformation
        ChMatrix33d F_t_inv;
        F_t_inv.setZero(); F_t_inv.fillDiagonal(1.0/alpha_t);
        // Mechanical deformation:  F = F_m * F_t  -->  F_m = F * F_t^{-1}
        ChMatrix33d F_m = F * F_t_inv;

        ChMatrix33d l;
        ChMatrix33d* a_l = nullptr;
        if (material_thermalstress->IsSpatialVelocityGradientNeeded()) {
            ChMatrixDynamic<> dot_x_hh(3, melement->GetNumNodes());
            this->GetFieldPackedStateBlockDt(melement, data, dot_x_hh, i_field_displ); // dot_x_hh = [v1 | v2 | v3 | v4..]
            l = dot_x_hh * dNde.transpose() * J_x.inverse();
        }

        // Compute  2nd Piola-Kirchhoff tensor in Voigt notation using the constitutive relation of material
        ChStressTensor<> S_stress;
        material_thermalstress->material_stress->ComputeStress(S_stress,
            F_m,
            a_l,
            data.matpoints_data.size() ? data.matpoints_data[i_point].get() : nullptr,
            &data.element_data);

        // To the discrete coordinates (elastic part): 

        ChMatrixDynamic<> B(6, n_ele_coords_deform);
        ChDomainDeformation::ComputeB(B, dNdX, F_m);

        // We have:             Fi = - sum (B' * S * w * |J|)
        // so here we compute  Fi += - B' * S * s
        Fi.segment(n_ele_coords_thermal, n_ele_coords_deform) += -(B.transpose() * S_stress) * s;


        //--- THERMAL PROBLEM ---

        // Gradient of temperature
        ChVector3d T_grad = dNdX * T_h.transpose();  //  = \nabla_x T(x) 

        // Heat flux. 
        // (For a linearixed thermal material, this is q_flux = - [k] * T_grad; with [k] conductivity matrix.)
        ChVector3d q_flux;
        this->material_thermalstress->material_thermal->ComputeHeatFlux(q_flux,
            T_grad, T,
            data.matpoints_data.size() ? data.matpoints_data[i_point].get() : nullptr,
            &data.element_data);

        // To the discrete coordinates  (thermal part): 
        
        //   Fi += dNdX' * q_flux * s
        Fi.segment(0, n_ele_coords_thermal) += dNdX.transpose() * q_flux.eigen() * s;   // += dNdX' * q_flux * s


        // ----------------------------------
        
        // Store auxiliary data in material point data (ex. for postprocessing).  

        data.matpoints_data_aux[i_point].F = F;
        data.matpoints_data_aux[i_point].F_m = F_m;
        data.matpoints_data_aux[i_point].F_t_inv = F_t_inv;
        data.matpoints_data_aux[i_point].q_flux = q_flux;
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

        const unsigned int i_field_temp = 0;  // temperature  field is 1st 
        const unsigned int i_field_displ = 1; // displacement field is 2nd 

        ChMatrixDynamic<> dNdX;
        ChRowVectorDynamic<> N;
        melement->ComputedNdX(eta, dNdX);
        melement->ComputeN(eta, N);

        unsigned int n_ele_coords_thermal = melement->GetNumNodes();
        unsigned int n_ele_coords_deform = 3* melement->GetNumNodes();

        // NOTE: the tangent matrices of the coupled thermo-deformation problem, for a finite element,
        // and with our assumption that states are ordered as [x_1; x_2; ... T_1, T_2], has a block structure
        // like the following:
        //   [ K_therm-therm | K_therm-def   ]
        //   [ --------------|---------------]
        //   [ K_def_therm   | K_def-def     ]
        // When written in this ordering in the H matrix of the ChKRMBlock of the element, bookkepping will
        // be automatic when spreading K values to the system-level matrices.
        //  In this case,
        //  - the K_def-def  block has a size of 3*n_nodes x 3*n_nodes and
        // corresponds to the same K matrix of the ChDomainDeformation problem.
        //  - the K_therm-therm  block has a size of 1*n_nodes x 1*n_nodes and
        // corresponds to the same K matrix of the ChDomainThermal problem.
        //  - the K_def_therm is 3*n_nodes x 1*n_nodes and K_therm-def is 1*n_nodes x 3*n_nodes and
        // represent the coupling between temp and deformation in implicit iterations, but these are ignored 
        // for simplicity, and not computed here.
        // So, proceed as with a staggered solver:
        

        //--- DEFORMATION PROBLEM ---

        ChVectorDynamic<> T_h;
        this->GetFieldStateBlock(melement, T_h, i_field_temp);
        double T = N * T_h;

        // Compute the matrix  x_hh = [x1|x2|x3|x4..] with discrete values of spatial positions at nodes
        ChMatrixDynamic<> x_hh(3, melement->GetNumNodes());
        this->GetFieldPackedStateBlock(melement, data, x_hh, i_field_displ);

        ChMatrixDynamic<> dNde;
        melement->ComputedNde(eta, dNde);

        // F deformation tensor = J_x * J_X^{-1}
        //   J_X: already available via element->ComputeJ()
        //   J_x: compute via   [x1|x2|x3|x4..]*dNde'

        ChMatrix33d J_X_inv;
        melement->ComputeJinv(eta, J_X_inv);

        ChMatrix33d J_x = x_hh * dNde.transpose();

        ChMatrix33d F = J_x * J_X_inv;

        // Thermal deformation: 
        // alpha_t = 1 + alpha *(T - T_0)
        double alpha_t = 1 + material_thermalstress->GetThermalExpansionCoefficient() * (T - material_thermalstress->GetRestTemperature());
        // Isotropic F_t initial deformation
        ChMatrix33d F_t_inv;
        F_t_inv.setZero(); F_t_inv.fillDiagonal(1.0 / alpha_t);
        // Mechanical deformation:  F = F_m * F_t  -->  F_m = F * F_t^{-1}
        ChMatrix33d F_m = F * F_t_inv;

        ChMatrix33d l;
        ChMatrix33d* a_l = nullptr;
        if (material_thermalstress->material_stress->IsSpatialVelocityGradientNeeded()) {
            ChMatrixDynamic<> dot_x_hh(3, melement->GetNumNodes());
            this->GetFieldPackedStateBlockDt(melement, data, dot_x_hh, i_field_displ); // dot_x_hh = [v1 | v2 | v3 | v4..]
            l = dot_x_hh * dNde.transpose() * J_x.inverse();
        }

        // K_def-def = sum (B' * k * B  * w * |J|)  
        if (Kpfactor) {

            // Compute tangent modulus
            ChMatrix66<double> C;
            this->material_thermalstress->material_stress->ComputeTangentModulus(C,
                F_m,
                a_l,
                data.matpoints_data.size() ? data.matpoints_data[i_point].get() : nullptr,
                &data.element_data);

            ChMatrixDynamic<> B(6, n_ele_coords_deform);
            ChDomainDeformation::ComputeB(B, dNdX, F_m);

            H.block(n_ele_coords_thermal, n_ele_coords_thermal, n_ele_coords_deform, n_ele_coords_deform) += Kpfactor * (B.transpose() * C * B);

            // ***TODO*** add the geometric tangent stiffness
            // ***TODO*** rayleigh damping
        }

        // M_def-def  matrix : consistent mass matrix:   
        // M_def-def = sum (N' * rho * N * w * |J|)
        if (Mpfactor) {

            // If we had the "3 rows" form of the shape function matrix, say N_ where N_=[N(1)*I, N(2)*I, ], it would be
            //   M = sum (N_' * rho * N_ * w * |J|)     that is simply:
            //   H += (Mpfactor * this->material->GetDensity()) * (N_.transpose() * N_);
            // But the N_ matrix would be very sparse, so to speedup computation we unroll it and do:
            double scalar_factor = Mpfactor * this->material_thermalstress->material_stress->GetDensity();
            for (int i = 0; i < N.cols(); i++) {
                for (int j = 0; j < N.cols(); j++) {
                    // Compute the scalar entry for the 8x8 scalar mass matrix (offset because of thermal coords in upper block)
                    double scalar_entry = scalar_factor * N(i) * N(j);
                    int row_start = n_ele_coords_thermal + i * 3;
                    int col_start = n_ele_coords_thermal + j * 3;
                    H(row_start, col_start) += scalar_entry; // xx
                    H(row_start + 1, col_start + 1) += scalar_entry; // yy  
                    H(row_start + 2, col_start + 2) += scalar_entry; // zz
                }
            }

            // ***TODO*** rayleigh damping
        }


        //--- THERMAL PROBLEM ---
        
        // Temperature at point (might be needed by nonlinear ChMaterial3DThermal materials with dependence on T)
        

        // K_thermo-thermo = sum (dNdX' * [k] * dNdX * w * |J|)
        if (Kpfactor) {
            
            ChMatrix33d tangent_conductivity;
            this->material_thermalstress->material_thermal->ComputeTangentModulus(tangent_conductivity,
                VNULL, T,
                data.matpoints_data.size() ? data.matpoints_data[i_point].get() : nullptr,
                &data.element_data);
            
            // upper left block of K 
            H.block(0, 0, n_ele_coords_thermal, n_ele_coords_thermal) += Kpfactor * (dNdX.transpose() * tangent_conductivity * dNdX); // H += Kpfactor * (B' * [k] * B)
        }

        // R_thermo-thermo = sum ( N' * N * (c*rho) * w * |J|)
        if (Rpfactor) {
            double c_rho;
            this->material_thermalstress->material_thermal->ComputeDtMultiplier(c_rho,
                T,
                data.matpoints_data.size() ? data.matpoints_data[i_point].get() : nullptr,
                &data.element_data);

            // upper left block of K 
            H.block(0, 0, n_ele_coords_thermal, n_ele_coords_thermal) += Rpfactor * c_rho * (N.transpose() * N); // H += Rpfactor  * (N' * N) * (c*rho)
        }
    }



    /// Invoked at the end of each time step. If the material has some 
    /// custom handling of auxiliary data (states etc.) it will manage this in ComputeUpdateEndStep()
    virtual void PointUpdateEndStep(std::shared_ptr<ChFieldElement> melement,
        DataPerElement& data,
        const int i_point, 
        const double time
    ) override {
        material_thermalstress->material_stress->ComputeUpdateEndStep(
            data.matpoints_data.size() ? data.matpoints_data[i_point].get() : nullptr,
            &data.element_data,
            time);
        //material_thermalstress->material_thermal->ComputeUpdateEndStep(); // not needed as thermal materials do not have end step updates
    }


    //
    // INTERFACE to ChPhysicsItem 
    //

    /// Usually not necessary to override ChPhysicsItem because the parent ChDomainImpl does all needed.
    /// However, HERE WE OVERLOAD THE PARENT IMPLEMENTATION BECAUSE WE MAY ADD AUTOMATIC GRAVITY
    /// PER EACH ELEMENT.
    /// Takes the F force term, scale and adds to R at given offset:
    virtual void IntLoadResidual_F(const unsigned int off,  ///< offset in R residual
        ChVectorDynamic<>& R,    ///< result: the R residual, R += c*F
        const double c           ///< a scaling factor
    ) override {
        // Do usual stuff with computation of R loads
        this->Base::IntLoadResidual_F(off, R, c);

        // Do also gravity, automatic per each element:
        // loads on nodes connected by the elements of the domain - here come the internal force vectors!!!
        if (this->automatic_gravity_load) {
            this->LoadAutomaticGravity(R, c);
        }
    }

    /// Utility function that adds gravity force per each element. 
    /// This function can called from the IntLoadResidual_F() function.
    void LoadAutomaticGravity(ChVectorDynamic<>& R,  ///< result: the R residual, R += c*F
        const double c           ///< a scaling factor
    ) {
        unsigned int i_field = 1; //  ChFieldDisplacement is the second in the tuple of this domain
        int nfield_coords = this->fields[i_field]->GetNumFieldCoordsVelLevel(); // should be 3 anyway

        auto mloadable_uvw = chrono_types::make_shared<ChFieldElementLoadableVolume>(nullptr, this->fields[i_field]);

        fea::ChLoaderGravity mygravity_loader(nullptr, this->material_thermalstress->GetDensity());
        mygravity_loader.SetGravitationalAcceleration(this->gravity_G_acceleration);
        mygravity_loader.SetNumIntPoints(this->num_points_gravity);
        mygravity_loader.SetLoadable(mloadable_uvw);

        for (auto& mel : this->element_datamap) {
            if (auto melement_volume = std::dynamic_pointer_cast<ChFieldElementVolume>(mel.first)) {
                mloadable_uvw->SetElement(melement_volume);
                mygravity_loader.ComputeQ(nullptr, nullptr);
                // Q is contiguous, so must store sparsely in R, per each node 
                unsigned int stride = 0;
                for (unsigned int i_node = 0; i_node < melement_volume->GetNumNodes(); i_node++) {
                    if (ChFieldDataState* mfielddata = dynamic_cast<ChFieldDataState*>(mel.second.nodes_data[i_node][i_field])) {
                        if (!mfielddata->IsFixed()) {
                            R.segment(mfielddata->DataGetOffsetVelLevel(), nfield_coords) += c * mygravity_loader.Q.segment(stride, nfield_coords);
                        }
                        stride += nfield_coords;
                    }
                }
            }
        }

    }


    /// If true, as by default, this mesh will add automatically a gravity load
    /// to all contained elements (that support gravity) using the G value from the ChSystem.
    /// So this saves you from adding many ChLoad<fea::ChLoaderGravity> to all elements.
    void SetAutomaticGravity(bool mg, const ChVector3d G_acc = ChVector3d(0, -9.81, 0), int num_points = 1) {
        automatic_gravity_load = mg;
        num_points_gravity = num_points;
        gravity_G_acceleration = G_acc;
    }
    /// Tell if this mesh will add automatically a gravity load to all contained elements.
    bool GetAutomaticGravity() { return automatic_gravity_load; }


    //
    // EXTRACTORS for drawing stuff in postprocessors/visualization:
    //

    /// Extract the unsymmetric F deformation gradient tensor
    class ExtractDeformationGradientF : public ChVisualDataExtractorMatrix33<ExtractDeformationGradientF, ChFieldDataAuxiliaryThermoDeformation, DataAtMaterialpoint> {
        virtual ChMatrix33d ExtractImpl(const ChFieldDataAuxiliaryThermoDeformation* fdata)  const override {
            return fdata->F;
        }
    };

    /// Extract the  C=F^T*F  right Cauchy-Green deformation tensor (should be plotted on material undeformed space)
    class ExtractRightCauchyGreenC : public ChVisualDataExtractorMatrix33<ExtractRightCauchyGreenC, ChFieldDataAuxiliaryThermoDeformation, DataAtMaterialpoint> {
        virtual ChMatrix33d ExtractImpl(const ChFieldDataAuxiliaryThermoDeformation* fdata)  const override {
            return fdata->F.transpose() * fdata->F;
        }
    };

    /// Extract the  B=F*F^T left Cauchy-Green deformation tensor (should be plotted on spatial deformed space)
    class ExtractLeftCauchyGreenB : public ChVisualDataExtractorMatrix33<ExtractLeftCauchyGreenB, ChFieldDataAuxiliaryThermoDeformation, DataAtMaterialpoint> {
        virtual ChMatrix33d ExtractImpl(const ChFieldDataAuxiliaryThermoDeformation* fdata)  const override {
            return fdata->F * fdata->F.transpose();
        }
    };

    /// Extract the E=1/2(F^T*F - I)  Green-Lagrange strain tensor (should be plotted on material undeformed space)
    class ExtractGreenLagrangeStrain : public ChVisualDataExtractorMatrix33<ExtractGreenLagrangeStrain, ChFieldDataAuxiliaryThermoDeformation, DataAtMaterialpoint> {
        virtual ChMatrix33d ExtractImpl(const ChFieldDataAuxiliaryThermoDeformation* fdata)  const override {
            return 0.5 * (fdata->F.transpose() * fdata->F - ChMatrix33d(1));
        }
    };

    /// Extract the e=1/2(I- (F*F^T)^-1)  Euler-Almansi strain tensor (should be plotted on spatial deformed space)
    class ExtractEulerAlmansiStrain : public ChVisualDataExtractorMatrix33<ExtractEulerAlmansiStrain, ChFieldDataAuxiliaryThermoDeformation, DataAtMaterialpoint> {
        virtual ChMatrix33d ExtractImpl(const ChFieldDataAuxiliaryThermoDeformation* fdata)  const override {
            return 0.5 * (ChMatrix33d(1) - (fdata->F * fdata->F.transpose()).inverse());
        }
    };

    class ExtractHeatFlux : public ChVisualDataExtractorVector<ExtractHeatFlux, ChFieldDataAuxiliaryThermoDeformation, DataAtMaterialpoint > {
        virtual ChVector3d ExtractImpl(const ChFieldDataAuxiliaryThermoDeformation* fdata)  const override {
            return fdata->q_flux;
        }
    };

protected:

    /// Get the material of the domain. Called when creating auxiliary data per material point.
    virtual std::shared_ptr<ChMaterial> GetMaterial() override {
        return material_thermalstress;
    };


private:
    /// Utility: Compute 9 x (n_nodes*4) matrix B=[B_1, B_2, ...B_n] where  B_i is a 9x4 matrix as in 
    ///    dE = B_i dx_i,  where dE is variation in {Green Lagrange strain, ∇T} 
    /// and dx_i is the variation in {spatial node coordinates, T} of i-th node. 
    void ComputeB(ChMatrixRef B, ChMatrixConstRef dNdX, ChMatrixConstRef F, unsigned int n_nodes) {
        B.resize(9, 4 * n_nodes);
        B.setZero();
        for (unsigned int i = 0; i < n_nodes; ++i) {
            // stress part:
            // g = ∇₀ N_i = J_X⁻¹ ∇_ξ N_i = dNdX(:, i)
            //                          g₁* [F₁₁, F₂₁, F₃₁]
            B.block<1, 3>(0, i * 4) = dNdX(0, i) * F.block<3, 1>(0, 0).transpose();
            //                          g₂* [F₁₂, F₂₂, F₃₂]
            B.block<1, 3>(1, i * 4) = dNdX(1, i) * F.block<3, 1>(0, 1).transpose();
            //                          g₃* [F₁₃, F₂₃, F₃₃]
            B.block<1, 3>(2, i * 4) = dNdX(2, i) * F.block<3, 1>(0, 2).transpose();
            //                          g₂* [F₁₃, F₂₃, F₃₃]                             + g₃ * [F₁₂, F₂₂, F₃₂]
            B.block<1, 3>(3, i * 4) = dNdX(1, i) * F.block<3, 1>(0, 2).transpose() + dNdX(2, i) * F.block<3, 1>(0, 1).transpose();
            //                          g₁* [F₁₃, F₂₃, F₃₃]                             + g₃ * [F₁₁, F₂₁, F₃₁]
            B.block<1, 3>(4, i * 4) = dNdX(0, i) * F.block<3, 1>(0, 2).transpose() + dNdX(2, i) * F.block<3, 1>(0, 0).transpose();
            //                          g₁* [F₁₂, F₂₂, F₃₂]                             + g₂ * [F₁₁, F₂₁, F₃₁]
            B.block<1, 3>(5, i * 4) = dNdX(0, i) * F.block<3, 1>(0, 1).transpose() + dNdX(1, i) * F.block<3, 1>(0, 0).transpose();
            // temperature part:                      
            B.block<3, 1>(6, 3 + i * 4) = dNdX.block<3, 1>(0, i);
        }
    }

    bool automatic_gravity_load;
    int num_points_gravity;
    ChVector3d gravity_G_acceleration;

};




/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
