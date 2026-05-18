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

#ifndef CHMATERIAL3DSTRESSPARALLEL_H
#define CHMATERIAL3DSTRESSPARALLEL_H

#include "chrono/fea/multiphysics/ChMaterial3DStress.h"
#include "chrono/core/ChTensors.h"


namespace chrono {
namespace fea {



/// @addtogroup chrono_fea
/// @{

/// Helper class for compounding material data from two materials 
/// into one single data structure.

class ChFieldDataParallel : public ChFieldData {
  public:
    std::unique_ptr<ChFieldData> data_A;
    std::unique_ptr<ChFieldData> data_B;

    ChFieldDataParallel(std::unique_ptr<ChFieldData> data_A, std::unique_ptr<ChFieldData> data_B) : data_A(std::move(data_A)), data_B(std::move(data_B)) {}

    virtual ChFieldData* GetNthSubData(int n) override { 
        if (n== 0) {
            return data_A.get();
        }
        else if (n == 1) {
            return data_B.get();
        }
        return nullptr;
    }  
};

/// Class for 3D stress materials A B that are composed by two generic ChMaterial3DStress in parallel, 
/// ex. an elastic law and a viscous law, that contribute in parallel to the total stress.
/// That is, stress is composed additively:  
///     S_total = S_a + S_b. 
/// Btw also tangent modulus is C_total = C_a + C_b, and D_total = D_a + D_b, where the 
/// contributions a and b are computed by the two materials in parallel.

class ChMaterial3DStressParallel : public ChMaterial3DStress {
  public:

    ChMaterial3DStressParallel() {}

    virtual ~ChMaterial3DStressParallel() {}

    /// Set the first material (A) in the parallel composition.
    void SetMaterialA(std::shared_ptr<ChMaterial3DStress> mA) { material_A = mA; }

    /// Set the second material (B) in the parallel composition.
    void SetMaterialB(std::shared_ptr<ChMaterial3DStress> mB) { material_B = mB; }

    /// Get the first material (A) in the parallel composition.
    std::shared_ptr<ChMaterial3DStress> GetMaterialA() const { return material_A; }

    /// Get the second material (B) in the parallel composition.
    std::shared_ptr<ChMaterial3DStress> GetMaterialB() const { return material_B; }


    // INTERFACE   to ChMaterial3DStress

    /// Compute elastic stress from finite strain, passed as 3x3 deformation gradient tensor F.
    /// Assuming stress is 2nd Piola-Kirchhoff tensor "S_stress", in Voigt notation.
    
    virtual void ComputeStress(ChStressTensor<>& S_stress,      ///< output stress, PK2
                               const ChMatrix33d& F,            ///< current deformation gradient tensor
                               const ChMatrix33d* l,            ///< current spatial velocity gradient (might be nullptr if IsSpatialVelocityGradientNeeded() is false)
                               ChFieldData* data_per_point,     ///< pointer to auxiliary data (ex states), if any, per quadrature point
                               ChElementData* data_per_element  ///< pointer to auxiliary data (ex states), if any, per element
                               ) override {
        ChStressTensor<> S_A, S_B;
        auto parallel_data_per_point = (ChFieldDataParallel*)data_per_point;
        material_A->ComputeStress(S_A, F, l, parallel_data_per_point->data_A.get(), data_per_element);
        material_B->ComputeStress(S_B, F, l, parallel_data_per_point->data_B.get(), data_per_element);
        S_stress = S_A + S_B;
    }

    /// Computes the 6x6 tangent modulus for a given strain, assuming the definition  dS = C * dE, that maps   
    /// variations of S, 2nd Piola-Kirchhoff stress, and of E, Green Lagrange strains, both in Voigt notation.

    virtual void ComputeTangentModulus(ChMatrixNM<double, 6, 6>& C, ///< output C tangent modulus, as dS=C*dE
                                       ChMatrixNM<double, 6, 6>* D, ///< output D tangent modulus, as dS=C*d(dE/dt) (might be nullptr if IsSpatialVelocityGradientNeeded() is false)
                                const ChMatrix33d& F,           ///< current deformation gradient tensor
                                const ChMatrix33d* l,           ///< current spatial velocity gradient (might be nullptr if IsSpatialVelocityGradientNeeded() is false)
                                ChFieldData* data_per_point,    ///< pointer to auxiliary data (ex states), if any, per quadrature point
                                ChElementData* data_per_element ///< pointer to auxiliary data (ex states), if any, per element 
    ) override {
        ChMatrixNM<double, 6, 6> C_A, C_B;
        auto parallel_data_per_point = (ChFieldDataParallel*)data_per_point;
        if (D)
            D->setZero();

        if (material_A->IsSpatialVelocityGradientNeeded() && D) {
            ChMatrixNM<double, 6, 6> D_A;
            material_A->ComputeTangentModulus(C_A, &D_A, F, l, parallel_data_per_point->data_A.get(), data_per_element);
            *D += D_A;
        } else {
            material_A->ComputeTangentModulus(C_A, nullptr, F, l, parallel_data_per_point->data_A.get(), data_per_element);
        }

        if (material_B->IsSpatialVelocityGradientNeeded() && D) {
            ChMatrixNM<double, 6, 6> D_B;
            material_B->ComputeTangentModulus(C_B, &D_B, F, l, parallel_data_per_point->data_B.get(), data_per_element);
            *D += D_B;
        } else {
            material_B->ComputeTangentModulus(C_B, nullptr, F, l, parallel_data_per_point->data_B.get(), data_per_element);
        }

        // sum moduli in parallel (damping moduli D_A and D_B are already summed above, if needed, allowing speed optimization)
        C = C_A + C_B;
    }

    /// Update your own auxiliary data, if any, at the end of time step (ex for plasticity).
    /// This is called at the end of every time step (or nl static step)
    virtual void ComputeUpdateEndStep(ChFieldData* data_per_point,          ///< pointer to auxiliary data (ex states), if any, per quadrature point
                                      ChElementData* data_per_element,      ///< pointer to auxiliary data (ex states), if any, per element 
                                      const double time
    ) {
        auto parallel_data_per_point = (ChFieldDataParallel*)data_per_point;
        material_A->ComputeUpdateEndStep(parallel_data_per_point->data_A.get(), data_per_element, time);
        material_B->ComputeUpdateEndStep(parallel_data_per_point->data_B.get(), data_per_element, time);
    }

    /// Some material need info on the spatial velocity gradient  l=\nabla_x v ,
    /// where the time derivative of the deformation gradient F is  dF/dt = l*F.
    /// Some others, do not need this info. For optimization reason, then, the ChFEModelXXYY 
    /// queries this, and knows if the "l" parameter could be left to null when calling ComputeStress(...)
    virtual bool IsSpatialVelocityGradientNeeded() const override { 
        return material_A->IsSpatialVelocityGradientNeeded() || material_B->IsSpatialVelocityGradientNeeded();
    };

    /// A compound material might generate custom data per material point because materials A or B needs it.
    /// In this case, it will create a custom data that contains the union of the custom data of A and B, and redirect to them the calls to update this data.
    virtual std::unique_ptr<ChFieldData> CreateMaterialPointData() const override { 
        std::unique_ptr<ChFieldData> data_A = material_A->CreateMaterialPointData();
        std::unique_ptr<ChFieldData> data_B = material_B->CreateMaterialPointData();
        return std::make_unique<ChFieldDataParallel>(std::move(data_A), std::move(data_B));
    }

protected:
    std::shared_ptr<ChMaterial3DStress> material_A;
    std::shared_ptr<ChMaterial3DStress> material_B;  


    //virtual void ArchiveOut(ChArchiveOut& archive_out) override;  TODO
    //virtual void ArchiveIn(ChArchiveIn& archive_in) override; 
};




/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
