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

#ifndef CHDOMAINDEFORMATION_H
#define CHDOMAINDEFORMATION_H

#include "chrono/fea/ChDomain.h"
#include "chrono/fea/ChMaterial3DStressStVenant.h"
#include "chrono/fea/ChVisualDataExtractor.h"
#include "chrono/fea/ChLoaderGravity.h"
#include "chrono/fea/ChFieldElementLoadableVolume.h"

namespace chrono {
namespace fea {


/// @addtogroup chrono_fea
/// @{


/// Auxiliary data stored per each material point during the ChDomainDeformation
/// computation. This can be plotted in postprocessing, etc.
/// If you need to append additional data per each matpoint, do not modify this, just 
/// define your class with custom data and use it in my_material_class::T_per_materialpoint

class ChFieldDataAuxiliaryDeformation : public ChFieldDataNONE {
public:
    ChMatrix33d F;  /// deformation gradient tensor - can be used to plot Green-Lagrange etc. 
    ChVoightTensor<> S_pk2; /// stress as 2nd Piola-Kirchhoff stress tensor in Voigt notation
};


/// Domain for FEA large deformations (nonlinear finite strain theory). It is based on a vector field,
/// ChFieldDisplacement3D, that is used to store x, the absolute spatial position of nodes (NOT
/// the displacement from the material reference position, d=x-X, as in some software).
/// Material properties are defined via a material from the ChMaterial3DStress subclasses
/// (the simplest of these materials is the ChMaterial3DStressStVenant, that corresponds to 
/// conventional linear elasticity for small strains). 
/// Not copyable: don't do __declspec(dllexport), ie. "class ChApi ChDomainDeformation...", just keep all in .h

class ChDomainDeformation : public ChDomainImpl<
    std::tuple<ChFieldDisplacement3D>, // per each node
    ChFieldDataAuxiliaryDeformation,   // auxiliary scratch data per each quadrature point needed by domain algos - materials can add other data too.
    ChElementDataKRM> {                // auxiliary data per each element (defaults to jacobian matrices K, R, M)
public:

    // The following just to provide a shortcut in type naming.
    using Base = ChDomainImpl<
        std::tuple<ChFieldDisplacement3D>, // per each node
        ChFieldDataAuxiliaryDeformation,   // auxiliary scratch data per each quadrature point needed by domain algos - materials can add other data too.
        ChElementDataKRM
    >;
    using DataPerElement = typename Base::DataPerElement;

    /// Construct the domain
    ChDomainDeformation(std::shared_ptr<ChFieldDisplacement3D> melasticfield)
        : Base(melasticfield)
    {
        // attach  default materials to simplify user side
        material = chrono_types::make_shared<ChMaterial3DStressStVenant>();

        automatic_gravity_load = false;
        num_points_gravity = 1;
        gravity_G_acceleration.Set(0, -9.81, 0);
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
        // Compute shape functions N at eta, and their material derivatives
        ChMatrixDynamic<> dNdX;
        ChRowVectorDynamic<> N;
        melement->ComputedNdX(eta, dNdX);
        melement->ComputeN(eta, N);

        // Compute the matrix  x_hh = [x1|x2|x3|x4..] with discrete values of spatial positions at nodes
        ChMatrixDynamic<> x_hh(3, melement->GetNumNodes());
        this->GetFieldPackedStateBlock(melement, data, x_hh, 0);

        ChMatrixDynamic<> dNde;
        melement->ComputedNde(eta, dNde);

        // F deformation tensor = J_x * J_X^{-1}
        //   J_X: already available via element->ComputeJ()
        //   J_x: compute via   [x1|x2|x3|x4..]*dNde'

        ChMatrix33d J_X_inv;
        melement->ComputeJinv(eta, J_X_inv);

        ChMatrix33d J_x = x_hh * dNde.transpose();

        ChMatrix33d F = J_x * J_X_inv;

        ChMatrix33d l;
        ChMatrix33d* a_l = nullptr;
        if (material->IsSpatialVelocityGradientNeeded()) {
            ChMatrixDynamic<> dot_x_hh(3, melement->GetNumNodes());
            this->GetFieldPackedStateBlockDt(melement, data, dot_x_hh, 0); // dot_x_hh = [v1 | v2 | v3 | v4..]
            l = dot_x_hh * dNde.transpose() * J_x.inverse();
            a_l = &l;
        }

        // Compute  2nd Piola-Kirchhoff tensor S_pk2 in Voigt notation using the constitutive relation of material
        ChStressTensor<> S_pk2;
        material->ComputeStress(S_pk2,
            F,
            a_l,
            data.matpoints_data.size() ? data.matpoints_data[i_point].get() : nullptr,
            &data.element_data);

        ChMatrixDynamic<> B(6, 3 * melement->GetNumNodes());
        ChDomainDeformation::ComputeB(B, dNdX, F);

        // We have:             Fi = - sum (B' * S * w * |J|)
        // so here we compute  Fi += - B' * S * s
        Fi += -(B.transpose() * S_pk2) * s;

        // Store auxiliary data in material point data (ex. for postprocessing). This is 
        // the F tensor in ChFieldDataAuxiliaryDeformation, the auxiliary data per each quadrature point required by domain.
        data.matpoints_data_aux[i_point].F = F;
        data.matpoints_data_aux[i_point].S_pk2 = S_pk2;
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

        ChMatrixDynamic<> x_hh(3, melement->GetNumNodes());   // x_hh = [x1 | x2 | x3 | x4..]
        this->GetFieldPackedStateBlock(melement, data, x_hh, 0);

        ChMatrixDynamic<> dNde;
        melement->ComputedNde(eta, dNde);

        // F deformation tensor = J_x * J_X^{-1}
        //   J_X: already available via element->ComputeJ()
        //   J_x: compute via   [x1|x2|x3|x4..]*dNde'

        ChMatrix33d J_X_inv;
        melement->ComputeJinv(eta, J_X_inv);

        ChMatrix33d J_x = x_hh * dNde.transpose();

        ChMatrix33d F = J_x * J_X_inv;

        ChMatrix33d l;
        ChMatrix33d* a_l = nullptr;
        if (material->IsSpatialVelocityGradientNeeded()) {
            ChMatrixDynamic<> dot_x_hh(3, melement->GetNumNodes());
            this->GetFieldPackedStateBlockDt(melement, data, dot_x_hh, 0); // dot_x_hh = [v1 | v2 | v3 | v4..]
            l = dot_x_hh * dNde.transpose() * J_x.inverse();
        }

        // K  matrix 
        // K = sum (B' * C * B  * w * |J|)  
        if (Kpfactor) {

            // Compute tangent modulus (assumed: dP=[C]dE with P  2nd Piola-Kirchhoff, E Green-Lagrange)
            ChMatrix66<double> C;
            this->material->ComputeTangentModulus(C,
                F,
                a_l,
                data.matpoints_data.size() ? data.matpoints_data[i_point].get() : nullptr,
                &data.element_data);

            ChMatrixDynamic<> B(6, 3 * melement->GetNumNodes());
            ChDomainDeformation::ComputeB(B, dNdX, F);

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
                    H(row_start, col_start) += scalar_entry; // xx
                    H(row_start + 1, col_start + 1) += scalar_entry; // yy  
                    H(row_start + 2, col_start + 2) += scalar_entry; // zz
                }
            }

            // ***TODO*** rayleigh damping
        }
    }

    /// Invoked at the end of each time step. If the material has some 
    /// custom handling of auxiliary data (states etc.) it will manage this in ComputeUpdateEndStep()
    virtual void PointUpdateEndStep(std::shared_ptr<ChFieldElement> melement,
        DataPerElement& data,
        const int i_point,
        const double time
    ) {
        material->ComputeUpdateEndStep(
            data.matpoints_data.size() ? data.matpoints_data[i_point].get() : nullptr,
            &data.element_data,
            time);
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
        unsigned int i_field = 0; // assume ChFieldDisplacement is the first in the tuple of this domain
        int nfield_coords = this->fields[i_field]->GetNumFieldCoordsVelLevel(); // should be 3 anyway

        auto mloadable_uvw = chrono_types::make_shared<ChFieldElementLoadableVolume>(nullptr, this->fields[i_field]);

        fea::ChLoaderGravity mygravity_loader(nullptr, this->material->GetDensity());
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
    class ExtractDeformationGradientF : public ChVisualDataExtractorMatrix33<ExtractDeformationGradientF, ChFieldDataAuxiliaryDeformation, DataAtMaterialpoint> {
        virtual ChMatrix33d ExtractImpl(const ChFieldDataAuxiliaryDeformation* fdata)  const override {
            return fdata->F;
        }
    };

    /// Extract the  C=F^T*F  right Cauchy-Green deformation tensor (should be plotted on material undeformed configuration)
    class ExtractRightCauchyGreenC : public ChVisualDataExtractorMatrix33<ExtractRightCauchyGreenC, ChFieldDataAuxiliaryDeformation, DataAtMaterialpoint> {
        virtual ChMatrix33d ExtractImpl(const ChFieldDataAuxiliaryDeformation* fdata)  const override {
            return fdata->F.transpose() * fdata->F;
        }
    };

    /// Extract the  B=F*F^T left Cauchy-Green deformation tensor (should be plotted on spatial deformed configuration)
    class ExtractLeftCauchyGreenB : public ChVisualDataExtractorMatrix33<ExtractLeftCauchyGreenB, ChFieldDataAuxiliaryDeformation, DataAtMaterialpoint> {
        virtual ChMatrix33d ExtractImpl(const ChFieldDataAuxiliaryDeformation* fdata)  const override {
            return fdata->F * fdata->F.transpose();
        }
    };

    /// Extract the E=1/2(F^T*F - I)  Green-Lagrange strain tensor (should be plotted on material undeformed configuration)
    class ExtractGreenLagrangeStrain : public ChVisualDataExtractorMatrix33<ExtractGreenLagrangeStrain, ChFieldDataAuxiliaryDeformation, DataAtMaterialpoint> {
        virtual ChMatrix33d ExtractImpl(const ChFieldDataAuxiliaryDeformation* fdata)  const override {
            return 0.5 * (fdata->F.transpose() * fdata->F - ChMatrix33d(1));
        }
    };

    /// Extract the e=1/2(I- (F*F^T)^-1)  Euler-Almansi strain tensor (should be plotted on spatial deformed configuration)
    class ExtractEulerAlmansiStrain : public ChVisualDataExtractorMatrix33<ExtractEulerAlmansiStrain, ChFieldDataAuxiliaryDeformation, DataAtMaterialpoint> {
        virtual ChMatrix33d ExtractImpl(const ChFieldDataAuxiliaryDeformation* fdata)  const override {
            return 0.5 * (ChMatrix33d(1) - (fdata->F * fdata->F.transpose()).inverse());
        }
    };

    /// Extract the stress, as 2nd Piola-Kirchhoff tensor (should be plotted on material undeformed configuration)
    class ExtractStress2ndPiolaKirchhoff : public ChVisualDataExtractorMatrix33<ExtractStress2ndPiolaKirchhoff, ChFieldDataAuxiliaryDeformation, DataAtMaterialpoint> {
        virtual ChMatrix33d ExtractImpl(const ChFieldDataAuxiliaryDeformation* fdata)  const override {
            ChMatrix33d S;
            fdata->S_pk2.ConvertToMatrix(S);
            return S;
        }
    };

    /// Extract the stress, as Cauchy true stress tensor (should be plotted on spatial deformed configuration)
    class ExtractStressCauchy : public ChVisualDataExtractorMatrix33<ExtractStressCauchy, ChFieldDataAuxiliaryDeformation, DataAtMaterialpoint> {
        virtual ChMatrix33d ExtractImpl(const ChFieldDataAuxiliaryDeformation* fdata)  const override {
            ChMatrix33d S;
            fdata->S_pk2.ConvertToMatrix(S);
            double det_J = fdata->F.determinant();
            return (1.0/det_J)* fdata->F * S * fdata->F.transpose();
        }
    };

    /// Utility: Compute  B as in  dE = B dx  where dE is variation in Green Lagrange strain (Voigt notation)
    /// and dx is the variation in spatial node coordinates (also works as  dE = B du  with du variation in displacements)
    static void ComputeB(ChMatrixRef B, ChMatrixConstRef dNdX, ChMatrixConstRef F) {
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

protected:

    /// Get the material of the domain.
    virtual std::shared_ptr<ChMaterial> GetMaterial() override {
        return material;
    };

    bool automatic_gravity_load;
    int num_points_gravity;
    ChVector3d gravity_G_acceleration;

};



/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
