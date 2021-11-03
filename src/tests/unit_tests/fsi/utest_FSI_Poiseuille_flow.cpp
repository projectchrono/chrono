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

// Chrono includes
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsGenerators.h"

// Chrono fsi includes
#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/utils/ChUtilsGeneratorFsi.h"
#include "chrono_fsi/utils/ChUtilsJSON.h"

// Chrono namespaces
using namespace chrono;
using namespace chrono::collision;
using namespace chrono::fsi;

// Set a tolerance to control the test
const double rel_Tol = 1.0e-2;
double error_rel;

//------------------------------------------------------------------
// dimension of the fluid domain
//------------------------------------------------------------------
double bxDim = 0.2;
double byDim = 0.1;
double bzDim = 0.2;

double fxDim = bxDim;
double fyDim = byDim;
double fzDim = bzDim;

//------------------------------------------------------------------
// Analytical solution of the poiseuille flow
//------------------------------------------------------------------
double PoiseuilleAnalytical(double Z,
                            double L,
                            double time,
                            std::shared_ptr<fsi::SimParams> paramsH){
    double nu   = paramsH->mu0/paramsH->rho0;
    double F    = paramsH->bodyForce3.x;
    double initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;
    double pi = 3.1415926;

    double L_modify = L + initSpace0;
    double Z_modify = Z + 0.5*initSpace0;

    double theory = 1.0/(2.0*nu)*F*Z_modify*(L_modify-Z_modify);

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
                      ChSystemFsi& myFsiSystem,
                      std::shared_ptr<fsi::SimParams> paramsH) {
    auto mysurfmaterial = chrono_types::make_shared<ChMaterialSurfaceSMC>();

    // Set common material Properties
    mysurfmaterial->SetYoungModulus(6e4);
    mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetRestitution(0.2f);
    mysurfmaterial->SetAdhesion(0);

    // Create a body for the wall
    auto body = chrono_types::make_shared<ChBody>();
    body->SetIdentifier(-1);
    body->SetBodyFixed(true);
    body->SetCollide(true);
    body->GetCollisionModel()->ClearModel();

    // Size and position of the bottom and top walls
    auto initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;
    ChVector<> sizeWall(bxDim / 2, byDim / 2, 2 * initSpace0);
    ChVector<> posBottom(0, 0, -3 * initSpace0);
    ChVector<> posTop(0, 0, bzDim + 1 * initSpace0);

    // Add a geometry to the body and set the collision model 
    chrono::utils::AddBoxGeometry(body.get(), mysurfmaterial, sizeWall, posBottom, QUNIT, true);
    body->GetCollisionModel()->BuildModel();
    mphysicalSystem.AddBody(body);

    // Add BCE particles to the bottom and top wall boundary
    myFsiSystem.AddBceBox(paramsH, body, posTop, QUNIT, sizeWall, 12);
    myFsiSystem.AddBceBox(paramsH, body, posBottom, QUNIT, sizeWall, 12);
}

// ===============================
int main(int argc, char* argv[]) {
    // Create a physical system and a corresponding FSI system
    ChSystemSMC mphysicalSystem;
    ChSystemFsi myFsiSystem(mphysicalSystem);

    // Get the pointer to the system parameter and use a 
    // JSON file to fill it out with the user parameters
    std::shared_ptr<fsi::SimParams> paramsH = myFsiSystem.GetSimParams();

    // Initialize the parameters using an input JSON file
    std::string myJson = GetChronoDataFile("fsi/input_json/demo_FSI_Poiseuille_flow_Explicit.json");
    myFsiSystem.SetSimParameter(myJson, paramsH, ChVector<>(bxDim, byDim, bzDim));

    // Reset the domain size to handle periodic boundary condition
    auto initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;
    ChVector<> cMin(-bxDim / 2 - initSpace0 / 2, -byDim / 2 - initSpace0 / 2, - 10.0 * initSpace0);
    ChVector<> cMax( bxDim / 2 + initSpace0 / 2,  byDim / 2 + initSpace0 / 2, bzDim + 10.0 * initSpace0);
    myFsiSystem.SetBoundaries(cMin, cMax, paramsH);

    // Set up the solver based on the input value of the prameters
    myFsiSystem.SetFluidDynamics(paramsH->fluid_dynamic_type);
    myFsiSystem.SetFluidSystemLinearSolver(paramsH->LinearSolver);// this is only for ISPH

    // Setup sub doamins for a faster neighbor particle searching
    myFsiSystem.SetSubDomain(paramsH);

    // Create SPH particles for the fluid domain
    chrono::utils::GridSampler<> sampler(initSpace0);
    ChVector<> boxCenter(-bxDim / 2 + fxDim / 2 , 0, fzDim * 0.5);
    ChVector<> boxHalfDim(fxDim / 2, fyDim / 2, fzDim / 2);
    std::vector<ChVector<>> points = sampler.SampleBox(boxCenter, boxHalfDim);
    size_t numPart = points.size();
    for (int i = 0; i < numPart; i++) {
        double v_x = PoiseuilleAnalytical(points[i].z(), bzDim, 0.5, paramsH);
        myFsiSystem.AddSphMarker(points[i], paramsH->rho0, paramsH->BASEPRES, paramsH->mu0, paramsH->HSML, -1,
                                 ChVector<>(v_x, 0.0, 0.0));
    }
    myFsiSystem.AddRefArray(0, (int)numPart, -1, -1);

    // Create SPH particles for the solid domain
    CreateSolidPhase(mphysicalSystem, myFsiSystem, paramsH);

    // Finalize the setup before the simulation
    myFsiSystem.Finalize();

    double time = 0;
    int stepEnd = 200;
    for (int tStep = 0; tStep < stepEnd + 1; tStep++) {
        myFsiSystem.DoStepDynamics_FSI();
        time += paramsH->dT;
    
        // Copy data from device to host
        auto posRad = myFsiSystem.GetParticlePosOrProperties();
        auto vel = myFsiSystem.GetParticleVel();
       
        // Calculate the relative error of the solution
        double error = 0.0;
        double abs_val = 0.0;
        for (int i = 0; i < numPart; i++) {
            double pos_Z = posRad[i].z();
            double vel_X = vel[i].x();
            double vel_X_ana = PoiseuilleAnalytical(pos_Z, bzDim, time + 0.5, paramsH);
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