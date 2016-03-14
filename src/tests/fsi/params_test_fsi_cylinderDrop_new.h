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
#include "chrono_fsi/include/utils.h"
#include "chrono_fsi/ChParams.cuh"


// -----------------------------------------------------------------------------
// Simulation parameters FSI
// -----------------------------------------------------------------------------

// Duration of the "hold time" (vehicle chassis fixed and no driver inputs).
// This can be used to allow the granular material to settle.

// Dimensions
namespace chrono {
namespace fsi {

Real hdimX = 14;  // 5.5;
Real hdimY = 1.75;

Real hthick = 0.25;
Real basinDepth = 2;

Real fluidInitDimX = 2;
Real fluidHeight = 1.4;  // 2.0;

// -----------------------------------------------------------------------------
// Simulation parameters Fluid
// -----------------------------------------------------------------------------
//	when adding functionality using "useWallBce" and "haveFluid" macros, pay attention to  "initializeFluidFromFile"
// options.
//	for a double security, do your best to set "haveFluid" and "useWallBce" based on the data you have from
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
void SetupParamsH(SimParams* paramsH) {
	paramsH->sizeScale = 1;  // don't change it.
	paramsH->HSML = 0.1; //0.06;    // 0.06;//0.04;
	paramsH->MULT_INITSPACE = 1.0;
	paramsH->epsMinMarkersDis = .001;
	paramsH->NUM_BOUNDARY_LAYERS = 3;
	paramsH->toleranceZone = paramsH->NUM_BOUNDARY_LAYERS
			* (paramsH->HSML * paramsH->MULT_INITSPACE);
	paramsH->BASEPRES = 0;    // 10;
	paramsH->LARGE_PRES = 0;  // paramsH->BASEPRES;//10000;
	paramsH->deltaPress;      //** modified below
	paramsH->multViscosity_FSI = 1;//5.0;
	paramsH->gravity = mR3(0, 0, -9.81);  // mR3(0);//mR3(0, -9.81, 0);
	paramsH->bodyForce3 =
	mR3(0, 0, 0); // mR4(3.2e-3,0,0,0);// mR4(0);;// /*Re = 100 */ //mR4(3.2e-4, 0, 0, 0);/*Re = 100 */
	paramsH->rho0 = 1000;
	paramsH->markerMass = pow(paramsH->MULT_INITSPACE * paramsH->HSML, 3) * paramsH->rho0;
	paramsH->mu0 = .001;
	paramsH->v_Max = 1; // Arman, I changed it to 0.1 for vehicle. Check this
									 // later;//10;//50e-3;//18e-3;//1.5;//2e-1; /*0.2 for Re = 100 */ //2e-3;
	paramsH->EPS_XSPH = .5f;
	paramsH->dT = 1e-3; // 0.2e-4;//1.0e-4;  // 2e-3;  // note you are using half of this for MBD system
	paramsH->tFinal = 2;                         // 20 * paramsH->dT; //400
	paramsH->timePause = 0; //.0001 * paramsH->tFinal;//.0001 * paramsH->tFinal; 	// time
	// before applying any
	// bodyforce. Particles move only due to initialization. keep it as small as possible.
	// the time step will be 1/10 * dT.
	paramsH->kdT = 5;  // I don't know what is kdT
	paramsH->gammaBB = 0.5;
	paramsH->binSize0;           // will be changed
	paramsH->rigidRadius;        // will be changed
	paramsH->densityReinit = 0; // 0: no re-initialization, 1: with initialization
	paramsH->enableTweak = 1;    // 0: no tweak, 1: have tweak
	paramsH->enableAggressiveTweak = 0; // 0: no aggressive tweak; 1: with aggressive tweak (if 1, enableTweak should be 1 too)
	paramsH->tweakMultV = 0.1;//paramsH->v_Max / (paramsH->HSML / paramsH->dT);//	maximum allowed velocity: tweakMultV * HSML / dT;  NOTE: HSML and dT must be defined. So this line comes after them
	paramsH->tweakMultRho = .002; // maximum allowed density change in one time step: tweakMultRho * rho0
	paramsH->bceType = ADAMI;  // ADAMI, mORIGINAL
	//********************************************************************************************************
	//**  reminiscent of the past******************************************************************************
	//	paramsH->cMin = mR3(-paramsH->toleranceZone, -paramsH->toleranceZone, -paramsH->toleranceZone);
	//// 3D channel
	//	paramsH->cMax = mR3( 3  + paramsH->toleranceZone, 2 + paramsH->toleranceZone,  3 + paramsH->toleranceZone);
	//  paramsH->cMin = mR3(0, 0, 0);  // 3D channel
	//  paramsH->cMax = mR3(3, 2, 3);
	paramsH->cMin = mR3(-hdimX, -hdimY, -basinDepth - hthick);  // 3D channel
	paramsH->cMax = mR3(hdimX, hdimY, basinDepth);
	//****************************************************************************************
	// printf("a1  paramsH->cMax.x, y, z %f %f %f,  binSize %f\n", paramsH->cMax.x, paramsH->cMax.y, paramsH->cMax.z, 2 *
	// paramsH->HSML);
	int3 side0 = mI3(
			floor((paramsH->cMax.x - paramsH->cMin.x) / (2 * paramsH->HSML)),
			floor((paramsH->cMax.y - paramsH->cMin.y) / (2 * paramsH->HSML)),
			floor((paramsH->cMax.z - paramsH->cMin.z) / (2 * paramsH->HSML)));
	Real3 binSize3 = mR3((paramsH->cMax.x - paramsH->cMin.x) / side0.x,
			(paramsH->cMax.y - paramsH->cMin.y) / side0.y,
			(paramsH->cMax.z - paramsH->cMin.z) / side0.z);
	paramsH->binSize0 = (binSize3.x > binSize3.y) ? binSize3.x : binSize3.y;
	//	paramsH->binSize0 = (paramsH->binSize0 > binSize3.z) ? paramsH->binSize0 : binSize3.z;
	paramsH->binSize0 = binSize3.x; // for effect of distance. Periodic BC in x direction. we do not care about paramsH->cMax y and z.
	paramsH->cMax = paramsH->cMin + paramsH->binSize0 * mR3(side0);
	paramsH->boxDims = paramsH->cMax - paramsH->cMin;
	//****************************************************************************************
	paramsH->cMinInit = mR3(-fluidInitDimX, paramsH->cMin.y,
			-basinDepth + 1.0 * paramsH->HSML);  // 3D channel
	paramsH->cMaxInit = mR3(fluidInitDimX, paramsH->cMax.y,
			paramsH->cMinInit.z + fluidHeight);
	//****************************************************************************************
	//*** initialize straight channel
	paramsH->straightChannelBoundaryMin = paramsH->cMinInit; // mR3(0, 0, 0);  // 3D channel
	paramsH->straightChannelBoundaryMax = paramsH->cMaxInit; // SmR3(3, 2, 3) * paramsH->sizeScale;
	//************************** modify pressure ***************************
	//		paramsH->deltaPress = paramsH->rho0 * paramsH->boxDims * paramsH->bodyForce3;  //did not work as I expected
	paramsH->deltaPress = mR3(0);//Wrong: 0.9 * paramsH->boxDims * paramsH->bodyForce3; // viscosity and boundary shape should play a roll

	// modify bin size stuff
	//****************************** bin size adjustement and contact detection stuff *****************************
	int3 SIDE = mI3(
			int((paramsH->cMax.x - paramsH->cMin.x) / paramsH->binSize0 + .1),
			int((paramsH->cMax.y - paramsH->cMin.y) / paramsH->binSize0 + .1),
			int((paramsH->cMax.z - paramsH->cMin.z) / paramsH->binSize0 + .1));
	Real mBinSize = paramsH->binSize0; // Best solution in that case may be to change cMax or cMin such that periodic
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
	printStruct(paramsH->bodyForce3);
	std::cout << "paramsH->gravity: ";
	printStruct(paramsH->gravity);
	std::cout << "paramsH->rho0: " << paramsH->rho0 << std::endl;
	std::cout << "paramsH->mu0: " << paramsH->mu0 << std::endl;
	std::cout << "paramsH->v_Max: " << paramsH->v_Max << std::endl;
	std::cout << "paramsH->dT: " << paramsH->dT << std::endl;
	std::cout << "paramsH->tFinal: " << paramsH->tFinal << std::endl;
	std::cout << "paramsH->timePause: " << paramsH->timePause << std::endl;
	std::cout << "paramsH->timePauseRigidFlex: " << paramsH->timePauseRigidFlex
			<< std::endl;
	std::cout << "paramsH->densityReinit: " << paramsH->densityReinit
			<< std::endl;
	std::cout << "paramsH->cMin: ";
	printStruct(paramsH->cMin);
	std::cout << "paramsH->cMax: ";
	printStruct(paramsH->cMax);
	std::cout << "paramsH->MULT_INITSPACE: " << paramsH->MULT_INITSPACE
			<< std::endl;
	std::cout << "paramsH->NUM_BOUNDARY_LAYERS: " << paramsH->NUM_BOUNDARY_LAYERS
			<< std::endl;
	std::cout << "paramsH->toleranceZone: " << paramsH->toleranceZone
			<< std::endl;
	std::cout << "paramsH->NUM_BCE_LAYERS: " << paramsH->NUM_BCE_LAYERS
			<< std::endl;
	std::cout << "paramsH->solidSurfaceAdjust: " << paramsH->solidSurfaceAdjust
			<< std::endl;
	std::cout << "paramsH->BASEPRES: " << paramsH->BASEPRES << std::endl;
	std::cout << "paramsH->LARGE_PRES: " << paramsH->LARGE_PRES << std::endl;
	std::cout << "paramsH->deltaPress: ";
	printStruct(paramsH->deltaPress);
	std::cout << "paramsH->nPeriod: " << paramsH->nPeriod << std::endl;
	std::cout << "paramsH->EPS_XSPH: " << paramsH->EPS_XSPH << std::endl;
	std::cout << "paramsH->multViscosity_FSI: " << paramsH->multViscosity_FSI
			<< std::endl;
	std::cout << "paramsH->rigidRadius: ";
	printStruct(paramsH->rigidRadius);
	std::cout << "paramsH->binSize0: " << paramsH->binSize0 << std::endl;
	std::cout << "paramsH->boxDims: ";
	printStruct(paramsH->boxDims);
	std::cout << "paramsH->gridSize: ";
	printStruct(paramsH->gridSize);
	std::cout << "********************" << std::endl;
}

// -----------------------------------------------------------------------------
// Specification of the terrain
// -----------------------------------------------------------------------------
//
//// Control visibility of containing bin walls
//bool visible_walls = false;
//
//int Id_g = 100;
//Real r_g = 0.02;
//Real rho_g = 2500;
//Real vol_g = (4.0 / 3) * chrono::CH_C_PI * r_g * r_g * r_g; // Arman: PI or CH_C_PI
//Real mass_g = rho_g * vol_g;
//chrono::ChVector<> inertia_g = 0.4 * mass_g * r_g * r_g
//		* chrono::ChVector<>(1, 1, 1);
//
//
//
//int num_particles = 1000;

// -----------------------------------------------------------------------------
// Output parameters
// -----------------------------------------------------------------------------


//Real vertical_offset = 0;  // vehicle vertical offset

} // end namespace fsi
} // end namespace chrono
#endif  // end of FSI_HMMWV_PARAMS_H_
