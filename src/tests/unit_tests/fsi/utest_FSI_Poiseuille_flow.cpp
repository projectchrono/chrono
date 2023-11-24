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

#include <assert.h>
#include <stdlib.h>
#include <ctime>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsGenerators.h"

#include "chrono_fsi/ChSystemFsi.h"

// Chrono namespaces
using namespace chrono;
using namespace chrono::fsi;

// Set a tolerance to control the test
const double rel_Tol = 1.0e-2;
double error_rel;

//------------------------------------------------------------------
// dimension of the computational domain
//------------------------------------------------------------------
double bxDim = 0.2;
double byDim = 0.1;
double bzDim = 0.2;

//------------------------------------------------------------------
// Analytical solution of the poiseuille flow
//------------------------------------------------------------------
double PoiseuilleAnalytical(double Z, double L, double time, ChSystemFsi& sysFSI) {
    double nu = sysFSI.GetViscosity() / sysFSI.GetDensity();
    double F = sysFSI.GetBodyForce().x();
    double initSpace0 = sysFSI.GetInitialSpacing();
    double pi = 3.1415926;

    double L_modify = L + initSpace0;
    double Z_modify = Z + 0.5 * initSpace0;

    double theory = 1.0 / (2.0 * nu) * F * Z_modify * (L_modify - Z_modify);

    for (int n = 0; n < 50; n++) {
        theory = theory - 4.0 * F * pow(L_modify, 2) / (nu * pow(pi, 3) * pow(2 * n + 1, 3)) *
                              sin(pi * Z_modify * (2 * n + 1) / L_modify) *
                              exp(-pow(2 * n + 1, 2) * pow(pi, 2) * nu * time / pow(L_modify, 2));
    }

    return theory;
}

//------------------------------------------------------------------
// Create the wall boundary and the BCE particles
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemSMC& sysMBS, ChSystemFsi& sysFSI) {
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

    // Size and position of the bottom and top walls
    auto initSpace0 = sysFSI.GetInitialSpacing();
    ChVector<> size_XY(bxDim, byDim, 4 * initSpace0);
    ChVector<> pos_zn(0, 0, -3 * initSpace0);
    ChVector<> pos_zp(0, 0, bzDim + 1 * initSpace0);

    // Add a geometry to the body and set the collision model
    chrono::utils::AddBoxGeometry(body.get(), mysurfmaterial, size_XY, pos_zn, QUNIT, true);
    sysMBS.AddBody(body);

    // Add BCE particles to the bottom and top wall boundary
    sysFSI.AddBoxContainerBCE(body,                                           //
                              ChFrame<>(ChVector<>(0, 0, bzDim / 2), QUNIT),  //
                              ChVector<>(bxDim, byDim, bzDim),                //
                              ChVector<int>(0, 0, 2));
}

// ===============================
int main(int argc, char* argv[]) {
    // Create a physical system and a corresponding FSI system
    ChSystemSMC sysMBS;
    ChSystemFsi sysFSI(&sysMBS);

    // Initialize the parameters using an input JSON file
    std::string myJson = GetChronoDataFile("fsi/input_json/demo_FSI_Poiseuille_flow_Explicit.json");
    sysFSI.ReadParametersFromFile(myJson);

    // Reset the domain size to handle periodic boundary condition
    auto initSpace0 = sysFSI.GetInitialSpacing();
    ChVector<> cMin(-bxDim / 2 - initSpace0 / 2, -byDim / 2 - initSpace0 / 2, -10.0 * initSpace0);
    ChVector<> cMax(bxDim / 2 + initSpace0 / 2, byDim / 2 + initSpace0 / 2, bzDim + 10.0 * initSpace0);
    sysFSI.SetBoundaries(cMin, cMax);

    // Create SPH particles for the fluid domain
    chrono::utils::GridSampler<> sampler(initSpace0);
    ChVector<> boxCenter(0, 0, bzDim * 0.5);
    ChVector<> boxHalfDim(bxDim / 2, byDim / 2, bzDim / 2);
    std::vector<ChVector<>> points = sampler.SampleBox(boxCenter, boxHalfDim);
    size_t numPart = points.size();
    for (int i = 0; i < numPart; i++) {
        double v_x = PoiseuilleAnalytical(points[i].z(), bzDim, 0.5, sysFSI);
        sysFSI.AddSPHParticle(points[i], ChVector<>(v_x, 0.0, 0.0));
    }

    // Create SPH particles for the solid domain
    CreateSolidPhase(sysMBS, sysFSI);

    // Complete construction of the FSI system
    sysFSI.Initialize();

    double dT = sysFSI.GetStepSize();
    double time = 0;
    int stepEnd = 200;
    for (int tStep = 0; tStep < stepEnd + 1; tStep++) {
        sysFSI.DoStepDynamics_FSI();
        time += dT;

        // Copy data from device to host
        auto pos = sysFSI.GetParticlePositions();
        auto vel = sysFSI.GetParticleVelocities();

        // Calculate the relative error of the solution
        double error = 0.0;
        double abs_val = 0.0;
        for (int i = 0; i < numPart; i++) {
            double pos_Z = pos[i].z();
            double vel_X = vel[i].x();
            double vel_X_ana = PoiseuilleAnalytical(pos_Z, bzDim, time + 0.5, sysFSI);
            error = error + pow(vel_X - vel_X_ana, 2);
            abs_val = abs_val + pow(vel_X_ana, 2);
        }
        error_rel = sqrt(error / abs_val);
        if ((tStep > 1) && (error_rel > rel_Tol)) {
            printf("\n step = %d, error_rel =  %0.8f \n", tStep, error_rel);
            return 1;
        }
    }

    printf("\n  error_rel =  %0.8f \n", error_rel);
    return 0;
}