/*
 * SetupFsiParams.h
 *
 *  Created on: Mar 2, 2015
 *      Author: Arman Pazouki
 */
#ifndef FSI_TEST_CYLINDERDROP_NEW_
#define FSI_TEST_CYLINDERDROP_NEW_

/* C/C++ standard library*/
/* Chrono::FSI Library*/
#include "chrono_fsi/ChParams.cuh"
#include "chrono_fsi/utils/ChUtilsPrintStruct.h"

namespace chrono {
namespace fsi {
// -----------------------------------------------------------------------------
// Simulation parameters FSI
// -----------------------------------------------------------------------------

// Duration of the "hold time" (vehicle chassis fixed and no driver inputs).
// This can be used to allow the granular material to settle.

// Dimensions
// -----------------------------------------------------------------------------
// Simulation parameters Fluid
// -----------------------------------------------------------------------------
// When adding functionality using "useWallBce" and "haveFluid" macros, pay
// attention to  "initializeFluidFromFile"
// options. for a double security, do your best to set "haveFluid" and "useWallBce"
// based on the data you have from
// checkpoint files
// very important, since this option will overwrite the BCE pressure and
// paramsH->LARGE_PRES is only used for the initialization of the BCE markers

/**
 * @brief
 *    Fills in paramsH with simulation parameters.
 * @details
 *    The description of each parameter set here can be found in MyStruct.h
 *
 * @param paramsH: struct defined in MyStructs.cuh
 */
void SetupParamsH(SimParams* paramsH,
                  Real hdimX,
                  Real hdimY,
                  Real hthick,
                  Real basinDepth,
                  Real fluidInitDimX,
                  Real fluidHeight) {
    paramsH->sizeScale = 1;  // don't change it.
    paramsH->HSML = 0.2;
    paramsH->MULT_INITSPACE = 1.0;
    paramsH->epsMinMarkersDis = .001;
    paramsH->NUM_BOUNDARY_LAYERS = 3;
    paramsH->toleranceZone = paramsH->NUM_BOUNDARY_LAYERS * (paramsH->HSML * paramsH->MULT_INITSPACE);
    paramsH->BASEPRES = 0;
    paramsH->LARGE_PRES = 0;
    paramsH->deltaPress;
    paramsH->multViscosity_FSI = 1;
    paramsH->gravity = mR3(0, 0, -9.81);
    paramsH->bodyForce3 = mR3(0, 0, 0);
    paramsH->rho0 = 1000;
    paramsH->markerMass = pow(paramsH->MULT_INITSPACE * paramsH->HSML, 3) * paramsH->rho0;
    paramsH->mu0 = .001;
    paramsH->v_Max = 1;
    paramsH->EPS_XSPH = .5f;
    paramsH->dT = 1e-3;
    paramsH->tFinal = 2;
    paramsH->timePause = 0;
    paramsH->kdT = 5;
    paramsH->gammaBB = 0.5;
    paramsH->binSize0;
    paramsH->rigidRadius;
    paramsH->densityReinit = 0;
    paramsH->enableTweak = 1;
    paramsH->enableAggressiveTweak = 0;
    paramsH->tweakMultV = 0.1;
    paramsH->tweakMultRho = .002;
    paramsH->bceType = ADAMI;  // ADAMI, mORIGINAL
    paramsH->cMin = mR3(-hdimX, -hdimY, -basinDepth - hthick);
    paramsH->cMax = mR3(hdimX, hdimY, basinDepth);
    //****************************************************************************************
    int3 side0 = mI3(floor((paramsH->cMax.x - paramsH->cMin.x) / (2 * paramsH->HSML)),
                     floor((paramsH->cMax.y - paramsH->cMin.y) / (2 * paramsH->HSML)),
                     floor((paramsH->cMax.z - paramsH->cMin.z) / (2 * paramsH->HSML)));
    Real3 binSize3 = mR3((paramsH->cMax.x - paramsH->cMin.x) / side0.x, (paramsH->cMax.y - paramsH->cMin.y) / side0.y,
                         (paramsH->cMax.z - paramsH->cMin.z) / side0.z);
    paramsH->binSize0 = (binSize3.x > binSize3.y) ? binSize3.x : binSize3.y;
    paramsH->binSize0 = binSize3.x;  // for effect of distance. Periodic BC in x
                                     // direction. we do not care about
                                     // paramsH->cMax y and z.
    paramsH->cMax = paramsH->cMin + paramsH->binSize0 * mR3(side0);
    paramsH->boxDims = paramsH->cMax - paramsH->cMin;
    //****************************************************************************************
    paramsH->cMinInit = mR3(-fluidInitDimX, paramsH->cMin.y, -basinDepth + 1.0 * paramsH->HSML);  // 3D channel
    paramsH->cMaxInit = mR3(fluidInitDimX, paramsH->cMax.y, paramsH->cMinInit.z + fluidHeight);
    //****************************************************************************************
    //*** initialize straight channel
    paramsH->straightChannelBoundaryMin = paramsH->cMinInit;  // mR3(0, 0, 0);  // 3D channel
    paramsH->straightChannelBoundaryMax = paramsH->cMaxInit;  // SmR3(3, 2, 3) * paramsH->sizeScale;
    //************************** modify pressure ***************************
    //		paramsH->deltaPress = paramsH->rho0 * paramsH->boxDims *
    // paramsH->bodyForce3;  //did not work as I expected
    paramsH->deltaPress = mR3(0);  // Wrong: 0.9 * paramsH->boxDims *
                                   // paramsH->bodyForce3; // viscosity and
                                   // boundary shape should play a roll

    // modify bin size stuff
    //*************** bin size adjustment and contact detection *******************************
    int3 SIDE = mI3(int((paramsH->cMax.x - paramsH->cMin.x) / paramsH->binSize0 + .1),
                    int((paramsH->cMax.y - paramsH->cMin.y) / paramsH->binSize0 + .1),
                    int((paramsH->cMax.z - paramsH->cMin.z) / paramsH->binSize0 + .1));
    Real mBinSize = paramsH->binSize0;  // Best solution in that case may be to
                                        // change cMax or cMin such that periodic
                                        // sides be a multiple of binSize
    //**********************************************************************************************************
    paramsH->gridSize = SIDE;
    // paramsH->numCells = SIDE.x * SIDE.y * SIDE.z;
    paramsH->worldOrigin = paramsH->cMin;
    paramsH->cellSize = mR3(mBinSize, mBinSize, mBinSize);

    std::cout << "******************** paramsH Content" << std::endl;
    std::cout << "paramsH->sizeScale: " << paramsH->sizeScale << std::endl;
    std::cout << "paramsH->HSML: " << paramsH->HSML << std::endl;
    std::cout << "paramsH->bodyForce3: ";
    utils::printStruct(paramsH->bodyForce3);
    std::cout << "paramsH->gravity: ";
    utils::printStruct(paramsH->gravity);
    std::cout << "paramsH->rho0: " << paramsH->rho0 << std::endl;
    std::cout << "paramsH->mu0: " << paramsH->mu0 << std::endl;
    std::cout << "paramsH->v_Max: " << paramsH->v_Max << std::endl;
    std::cout << "paramsH->dT: " << paramsH->dT << std::endl;
    std::cout << "paramsH->tFinal: " << paramsH->tFinal << std::endl;
    std::cout << "paramsH->timePause: " << paramsH->timePause << std::endl;
    std::cout << "paramsH->timePauseRigidFlex: " << paramsH->timePauseRigidFlex << std::endl;
    std::cout << "paramsH->densityReinit: " << paramsH->densityReinit << std::endl;
    std::cout << "paramsH->cMin: ";
    utils::printStruct(paramsH->cMin);
    std::cout << "paramsH->cMax: ";
    utils::printStruct(paramsH->cMax);
    std::cout << "paramsH->MULT_INITSPACE: " << paramsH->MULT_INITSPACE << std::endl;
    std::cout << "paramsH->NUM_BOUNDARY_LAYERS: " << paramsH->NUM_BOUNDARY_LAYERS << std::endl;
    std::cout << "paramsH->toleranceZone: " << paramsH->toleranceZone << std::endl;
    std::cout << "paramsH->NUM_BCE_LAYERS: " << paramsH->NUM_BCE_LAYERS << std::endl;
    std::cout << "paramsH->solidSurfaceAdjust: " << paramsH->solidSurfaceAdjust << std::endl;
    std::cout << "paramsH->BASEPRES: " << paramsH->BASEPRES << std::endl;
    std::cout << "paramsH->LARGE_PRES: " << paramsH->LARGE_PRES << std::endl;
    std::cout << "paramsH->deltaPress: ";
    utils::printStruct(paramsH->deltaPress);
    std::cout << "paramsH->nPeriod: " << paramsH->nPeriod << std::endl;
    std::cout << "paramsH->EPS_XSPH: " << paramsH->EPS_XSPH << std::endl;
    std::cout << "paramsH->multViscosity_FSI: " << paramsH->multViscosity_FSI << std::endl;
    std::cout << "paramsH->rigidRadius: ";
    utils::printStruct(paramsH->rigidRadius);
    std::cout << "paramsH->binSize0: " << paramsH->binSize0 << std::endl;
    std::cout << "paramsH->boxDims: ";
    utils::printStruct(paramsH->boxDims);
    std::cout << "paramsH->gridSize: ";
    utils::printStruct(paramsH->gridSize);
    std::cout << "********************" << std::endl;
}

}  // end namespace fsi
}  // end namespace chrono
#endif  // end of FSI_HMMWV_PARAMS_H_
