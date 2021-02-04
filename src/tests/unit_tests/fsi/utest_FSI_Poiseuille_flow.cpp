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
// Authors: Wei Hu
// =============================================================================
//
// Unit test for poiseuille flow. This unit test uses analytical solution to 
// verify the implementation
// =============================================================================

// General Includes
#include <assert.h>
#include <stdlib.h>
#include <ctime>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"

// Chrono fsi includes
#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/utils/ChUtilsGeneratorFsi.h"
#include "chrono_fsi/utils/ChUtilsJSON.h"

// Chrono namespaces
using namespace chrono;
using namespace collision;

using std::cout;
using std::endl;
typedef fsi::Real Real;

//--------------------------------
// dimension of the fluid domain
//--------------------------------
Real bxDim = 0.2;
Real byDim = 0.1;
Real bzDim = 0.2;

Real fxDim = bxDim;
Real fyDim = byDim;
Real fzDim = bzDim;

const double rel_Tol = 1.0e-2;
Real error_rel;

//------------------------------------------------------------------
// Analytical solution of the poiseuille flow
//------------------------------------------------------------------
Real PoiseuilleAnalytical(Real Z,
                          Real L,
                          Real time,
                          std::shared_ptr<fsi::SimParams> paramsH){
    Real nu   = paramsH->mu0/paramsH->rho0;
    Real F    = paramsH->bodyForce3.x;
    Real initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;
    Real pi = 3.1415926;

    Real L_modify = L + initSpace0;
    Real Z_modify = Z + 0.5*initSpace0;

    Real theory = 1.0/(2.0*nu)*F*Z_modify*(L_modify-Z_modify);

    for (int n=0; n<50; n++){
        theory = theory - 
        4.0*F*pow(L_modify,2)/(nu*pow(pi,3)*pow(2*n+1,3))*sin(pi*Z_modify*(2*n+1)/L_modify)*exp(-pow(2*n+1,2)*pow(pi,2)*nu*time/pow(L_modify,2));
    }

    return theory;
}

//------------------------------------------------------------------
// Create the wall boundary and the BCE particles
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemSMC& mphysicalSystem,
                      fsi::ChSystemFsi& myFsiSystem,
                      std::shared_ptr<fsi::SimParams> paramsH) {
    auto mysurfmaterial = chrono_types::make_shared<ChMaterialSurfaceSMC>();

    // Set common material Properties
    mysurfmaterial->SetYoungModulus(6e4);
    mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetRestitution(0.2f);
    mysurfmaterial->SetAdhesion(0);

    // Ground body
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    ground->SetCollide(true);
    ground->GetCollisionModel()->ClearModel();
    Real initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;

    // Bottom and Top wall
    ChVector<> sizeBottom(bxDim / 2, byDim / 2 + 0 * initSpace0, 2 * initSpace0);
    ChVector<> posBottom(0, 0, -3 * initSpace0);
    ChVector<> posTop(0, 0, bzDim + 1 * initSpace0);

    chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, sizeBottom, posBottom, chrono::QUNIT, true);
    ground->GetCollisionModel()->BuildModel();
    mphysicalSystem.AddBody(ground);

    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, ground, posBottom, chrono::QUNIT, sizeBottom);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, ground, posTop, chrono::QUNIT, sizeBottom);
}

// ===============================
int main(int argc, char* argv[]) {
    // Create a physical system and a corresponding FSI system
    ChSystemSMC mphysicalSystem;
    fsi::ChSystemFsi myFsiSystem(mphysicalSystem);

    // Get the pointer to the system parameter and use a 
    // JSON file to fill it out with the user parameters
    std::shared_ptr<fsi::SimParams> paramsH = myFsiSystem.GetSimParams();

    // Use the default input file or you may enter 
    // your input parameters as a command line argument
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_Poiseuille_flow_Explicit.json");
    fsi::utils::ParseJSON(inputJson, paramsH, fsi::mR3(bxDim, byDim, bzDim));

    // Set up the solver based on the input value of the prameters
    myFsiSystem.SetFluidDynamics(paramsH->fluid_dynamic_type);
    myFsiSystem.SetFluidSystemLinearSolver(paramsH->LinearSolver);// this is only for ISPH

    // Set up the periodic boundary condition if needed
    Real initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;
    paramsH->cMin = fsi::mR3(-bxDim / 2 - initSpace0 / 2, -byDim / 2 - initSpace0 / 2, 0.0 - 5.0 * initSpace0);
    paramsH->cMax = fsi::mR3( bxDim / 2 + initSpace0 / 2,  byDim / 2 + initSpace0 / 2, bzDim + 5.0 * initSpace0);

    // Setup the binning for neighbor search
    fsi::utils::FinalizeDomain(paramsH);

    // Create SPH particles for the fluid domain
    utils::GridSampler<> sampler(initSpace0);
    ChVector<> boxCenter(-bxDim / 2 + fxDim / 2 , 0, fzDim * 0.5);
    ChVector<> boxHalfDim(fxDim / 2, fyDim / 2, fzDim / 2);
    utils::Generator::PointVector points = sampler.SampleBox(boxCenter, boxHalfDim);
    size_t numPart = points.size();
    for (int i = 0; i < numPart; i++) {
        Real v_x = PoiseuilleAnalytical(points[i].z(), bzDim, 0.5, paramsH);
        myFsiSystem.GetDataManager()->AddSphMarker(fsi::mR4(points[i].x(), points[i].y(), points[i].z(), paramsH->HSML),
                                                   fsi::mR3(v_x, 0.0, 0.0),
                                                   fsi::mR4(paramsH->rho0, paramsH->BASEPRES, paramsH->mu0, -1));
    }
    myFsiSystem.GetDataManager()->fsiGeneralData->referenceArray.push_back(mI4(0, (int)numPart, -1, -1));

    // Create SPH particles for the solid domain
    CreateSolidPhase(mphysicalSystem, myFsiSystem, paramsH);

    myFsiSystem.Finalize();

    Real time = 0;
    int stepEnd = 200;
    for (int tStep = 0; tStep < stepEnd + 1; tStep++) {
        myFsiSystem.DoStepDynamics_FSI();
        time += paramsH->dT;
    
        // Copy data from device to host
        fsi::ChUtilsDevice fsiUtils;
        fsiUtils.CopyD2H(myFsiSystem.GetDataManager()->sphMarkersD2->posRadD, myFsiSystem.GetDataManager()->sphMarkersH->posRadH);
        fsiUtils.CopyD2H(myFsiSystem.GetDataManager()->sphMarkersD2->velMasD, myFsiSystem.GetDataManager()->sphMarkersH->velMasH);
        thrust::host_vector<fsi::Real4> posRadH = myFsiSystem.GetDataManager()->sphMarkersH->posRadH;
        thrust::host_vector<fsi::Real3> velMasH = myFsiSystem.GetDataManager()->sphMarkersH->velMasH;
        
        // Calculate the relative error of the solution
        Real error = 0.0;
        Real abs_val = 0.0;
        for (int i = 0; i < numPart; i++) {
            Real pos_Z = posRadH[i].z;
            Real vel_X = velMasH[i].x;
            Real vel_X_ana = PoiseuilleAnalytical(pos_Z, bzDim, time + 0.5, paramsH);
            error = error + pow(vel_X - vel_X_ana, 2);
            abs_val = abs_val + pow(vel_X_ana, 2);
        }
        error_rel = sqrt(error/abs_val);
        if ((tStep>1) && (error_rel > rel_Tol)){
            printf("\n step = %d, error_rel =  %0.8f \n",  tStep, error_rel);
            return 1;
        }
    }

    printf("\n  error_rel =  %0.8f \n", error_rel);
    return 0;
}