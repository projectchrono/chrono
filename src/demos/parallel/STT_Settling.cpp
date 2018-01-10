// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Michal Kwarta
// =============================================================================
//
//
// =============================================================================

#include <iostream>
#include <vector>
#include <valarray>
#include <string>
#include <sstream>
#include <cmath>

#include "core/ChFileutils.h"
#include "core/ChStream.h"

#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono/core/ChFileutils.h"
#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"


#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

// Control use of OpenGL run-time rendering
//#define CHRONO_PARALLEL_HAS_OPENGL

#ifdef CHRONO_PARALLEL_HAS_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;
using namespace chrono::collision;

using std::cout;
using std::flush;
using std::endl;

// =========================== headers above ===================================
// =============================================================================

// Comment the following line to use DVI/DEM-C contact
#define USE_DEM

double gravity = 9.81; // m/s^2

// Predefined specimen particle size distributions
enum SpecimenDistrib { UNIFORM_BEADS_5mm, MONO_SPHERES_1mm, TRI_SPHERES_4mm_5mm_6mm, OTTAWA_SAND, MONTEREY_SAND, CAT_LIMESTONE, CAT_LIMESTONE_2, CAT_LIMESTONE_3};
SpecimenDistrib distrib = TRI_SPHERES_4mm_5mm_6mm;

// Predefined specimen materials
enum SpecimenMaterial { GLASS, STEEL, QUARTZ, CLAY, LIMESTONE, LIMESTONE_DENSE };
SpecimenMaterial material = STEEL;

// Predefined specimen geometries
enum SpecimenGeom { HARTL_OOI, OSULLIVAN, STANDARD_BOX, SMALL_BOX, JENIKE_SHEAR, STANDARD_TRIAXIAL, SMALL_TRIAXIAL, TRIAXIAL_CLAY, TRIAXIAL_LIMESTONE };
SpecimenGeom geom = OSULLIVAN;

const int maxParticleTypes = 5;
int numParticleTypes, i;

std::string out_dir1 = "OSULLIVAN_min";
std::string out_dir2 = out_dir1 + "/TRI";
std::string out_dir3 = out_dir2 + "/SET";

// Output
#ifdef USE_DEM
std::string out_dir = out_dir3 + "/DEM-P";
#else
std::string out_dir = out_dir3 + "/DEM-C";
#endif

std::string pov_dir = out_dir + "/POVRAY";
std::string checkpt_dir = out_dir + "/CHECKPOINTS";

std::string stress_file = out_dir + "/specimen_stress.dat";
std::string force_file = out_dir + "/specimen_force.dat";
std::string stats_file = out_dir + "/stats.dat";
std::string specimen_ckpnt_file = out_dir + "/specimen.dat";

std::string time_step_file = out_dir + "/time_step.dat";
std::string ext_mech_params_file = out_dir + "/mech_params.dat";

std::string residuals_file = out_dir + "/residuals.dat";

std::string maxvel_file = out_dir + "/maxvel.dat";


// Compressive pre-stress (Pa)
double sigma_a = 80e3;
double sigma_b = 80e3;
double sigma_c = 80e3;

// Solver settings
#ifdef USE_DEM
double time_step = 4e-6;
double tolerance = 1.0;
int max_iteration_bilateral = 100;
#else
double time_step = 1e-4;
double tolerance = 0.1;
int max_iteration_normal = 0;

int max_iteration_spinning = 0;
int max_iteration_bilateral = 50;//100;

#endif

int max_iteration_sliding = 700;//200;//300;//500;
double contact_recovery_speed = 0.02;

bool clamp_bilaterals = false;
double bilateral_clamp_speed = 0.1;

// Simulation parameters
#ifdef USE_DEM
double settling_time = 0.5; // (maximum)
double simulation_time = 1.0; // (maximum)
#else
double settling_time = 0.5; // (maximum)
double simulation_time = 1.0; // (maximum)
#endif

// Packing density
bool dense = false;

// Desired number of OpenMP threads (will be clamped to maximum available)
int threads = 0;
// Perform dynamic tuning of number of threads?
bool thread_tuning = false;

bool start_walls_fixed = true; // for debugging
bool visible_walls = true;

// Use automatic critical time step calculation (for DEM/DEM-P only)?
bool auto_time_step = false;

bool write_povray_data = false;
bool write_checkpts_data = true;

double data_out_step = 1e-3;       // time interval between data outputs
double visual_out_step = 1e-3;     // time interval between PovRay outputs
double checkpt_out_step = 1e-1;     // time interval between checkpoints outputs




double FindHighest(ChSystem* sys, double radius) {
	double highest = -1000;
	for (size_t i = 0; i < sys->Get_bodylist()->size(); ++i) {
		auto body = (*sys->Get_bodylist())[i];
		ChVector<> rDist3 = body->GetPos();
		rDist3.z = 0;
		double rDist = rDist3.Length();
		if ((body->GetIdentifier() > 0 && body->GetPos().z > highest)
			&& rDist < radius) {
			highest = body->GetPos().z;
		}
	}
	return highest;
}

// -----------------------------------------------------------------------------
// Find the height of the lowest sphere in the granular mix.
// We only look at bodies whith stricty positive
// identifiers (to exclude the containing bin).
// -----------------------------------------------------------------------------
double FindLowest(ChSystem* sys) {
	double lowest = 1000;
	for (size_t i = 0; i < sys->Get_bodylist()->size(); ++i) {
		auto body = (*sys->Get_bodylist())[i];
		if (body->GetIdentifier() > 0 && body->GetPos().z < lowest)
			lowest = body->GetPos().z;
	}
	return lowest;
}





// -----------------------------------------------------------------------------
// Find and print max velocity
// -----------------------------------------------------------------------------
void Print_MaxVel_MaxAcc(ChSystem* sys, double time) {

	double max_vel = 0;
	double max_acc = 0;

	for (size_t i = 0; i < sys->Get_bodylist()->size(); ++i) {
		auto body = (*sys->Get_bodylist())[i];
		if (body->GetIdentifier() > 0) {
			double vel2 = body->GetPos_dt().Length2();
			double acc2 = body->GetPos_dtdt().Length2();
			if (vel2 > max_vel) max_vel = vel2;
			if (acc2 > max_acc) max_acc = acc2;
		}
	}
	
	cout << time << " " << max_vel << " " << max_acc << "\n";

}




int main(int argc, char* argv[]) {

	{
		const char* text = argv[1];
		int int_text = atoi(text);
		if (int_text == 1){
			distrib = UNIFORM_BEADS_5mm;// 1 = uniform
			out_dir2 = out_dir1 + "/UNI";
		}

		if (int_text == 3){
			distrib = TRI_SPHERES_4mm_5mm_6mm;// 3 = tri_spheres
			out_dir2 = out_dir1 + "/TRI";
		}
	}

	 // "/UNI" or "/TRI";
	out_dir3 = out_dir2 + argv[2]; //"/SET_dt1e-*_iter**00"; * = 3, 4; ** = 1, 2, 3

	{
		const char* text = argv[3];
		time_step = atof(text);// 0.001 or 0.0001
	}
	{
		 const char* text = argv[4];
		 max_iteration_sliding = atoi(text); 
	}

	// argv[5] has to be used inside the code
	//{
	//	const char* text = argv[5];
	//	tolerance = atof(text); 
	//}

	{
		const char* text = argv[6];
		tolerance = atof(text); 
	}

	{
		const char* text = argv[7];
		threads = atoi(text); 
	}
	int zz = 0;
	{
		const char* text = argv[8];
		zz = atoi(text);
	}


	// updadting all the directories and files

	// Output
#ifdef USE_DEM
	out_dir = out_dir3 + "/DEM-P";
#else
	out_dir = out_dir3 + "/DEM-C";
#endif

	pov_dir = out_dir + "/POVRAY";
	checkpt_dir = out_dir + "/CHECKPOINTS";

	stress_file = out_dir + "/specimen_stress.dat";
	force_file = out_dir + "/specimen_force.dat";
	stats_file = out_dir + "/stats.dat";
	specimen_ckpnt_file = out_dir + "/specimen.dat";

	time_step_file = out_dir + "/time_step.dat";
	ext_mech_params_file = out_dir + "/mech_params.dat";

	residuals_file = out_dir + "/residuals.dat";

	maxvel_file = out_dir + "/maxvel.dat";



	cout << out_dir << " " << time_step << " " << tolerance << " " << max_iteration_sliding << " "
		<< contact_recovery_speed << "\n";

	float mu_value = 0.096f;
	float mu_ext_value = 0.228f;
	float COR_value = 0.597f;
	float COR_ext_value = 0.5f;

	cout << "mupp\t" << mu_value << "\n";
	cout << "mupw\t" << mu_ext_value << "\n";
	cout << "crpp\t" << COR_value << "\n";
	cout << "crpp\t" << COR_ext_value << "\n";
	cout << "Way of handling mu_eff: min(mu_1, mu_2)\n";


	// Create output directories
	if (ChFileutils::MakeDirectory(out_dir1.c_str()) < 0) {
		cout << "Error creating directory " << out_dir1 << endl;
		return 1;
	}
	if (ChFileutils::MakeDirectory(out_dir2.c_str()) < 0) {
		cout << "Error creating directory " << out_dir2 << endl;
		return 1;
	}
	if (ChFileutils::MakeDirectory(out_dir3.c_str()) < 0) {
		cout << "Error creating directory " << out_dir3 << endl;
		return 1;
	}
	if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
		cout << "Error creating directory " << out_dir << endl;
		return 1;
	}
	if (ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
		cout << "Error creating directory " << pov_dir << endl;
		return 1;
	}
	if (ChFileutils::MakeDirectory(checkpt_dir.c_str()) < 0) {
		cout << "Error creating directory " << checkpt_dir << endl;
		return 1;
	}

	// Setup output

	ChStreamOutAsciiFile TimeStepStream(time_step_file.c_str());
	ChStreamOutAsciiFile extMechParamsStream(ext_mech_params_file.c_str());
	TimeStepStream.SetNumFormat("%16.10e");
	extMechParamsStream.SetNumFormat("%16.4");


	ChStreamOutAsciiFile residualsStream(residuals_file.c_str());

	ChStreamOutAsciiFile maxvelStream(maxvel_file.c_str());


	ChStreamOutAsciiFile stressStream(stress_file.c_str());
	ChStreamOutAsciiFile forceStream(force_file.c_str());
	ChStreamOutAsciiFile statsStream(stats_file.c_str());
	stressStream.SetNumFormat("%16.5e");
	forceStream.SetNumFormat("%16.5e");

	residualsStream.SetNumFormat("%16.5e");

	maxvelStream.SetNumFormat("%16.5e");


	// Create the system

#ifdef USE_DEM
	cout << "Create DEM/DEM-P system" << endl;
	const std::string title = "soft-sphere (DEM/DEM-P) granular material specimen";
	//ChBody::ContactMethod contact_method = ChBody::DEM;
	ChSystemParallelDEM* my_system = new ChSystemParallelDEM();
#else
	cout << "Create DVI/DEM-C system" << endl;
	const std::string title = "hard-sphere (DVI/DEM-C) granular material specimen";
	//ChBody::ContactMethod contact_method = ChBody::DVI;
	ChSystemParallelDVI* my_system = new ChSystemParallelDVI();
#endif

	my_system->Set_G_acc(ChVector<>(0, 0, -gravity));

	// ============================== System settings ==============================

	// Set number of threads

	int max_threads = my_system->GetParallelThreadNumber();
	if (threads > max_threads) threads = max_threads;
	my_system->SetParallelThreadNumber(threads);
	omp_set_num_threads(threads);

	my_system->GetSettings()->max_threads = threads;
	my_system->GetSettings()->perform_thread_tuning = thread_tuning;

	// Edit system settings

	my_system->GetSettings()->solver.use_full_inertia_tensor = false;
	my_system->GetSettings()->solver.tolerance = tolerance;
	my_system->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
	my_system->GetSettings()->solver.clamp_bilaterals = clamp_bilaterals;
	my_system->GetSettings()->solver.bilateral_clamp_speed = bilateral_clamp_speed;

#ifdef USE_DEM
	my_system->GetSettings()->solver.contact_force_model = ChSystemDEM::ContactForceModel::Hertz;
	my_system->GetSettings()->solver.tangential_displ_mode = ChSystemDEM::TangentialDisplacementModel::MultiStep;
#else
	my_system->GetSettings()->solver.solver_mode = SLIDING;
	my_system->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
	my_system->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
	my_system->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
	my_system->GetSettings()->solver.alpha = 0;
	my_system->GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
	my_system->ChangeSolverType(APGD);
#endif



	my_system->GetSettings()->collision.bins_per_axis = vec3(10, 10, zz);
	my_system->GetSettings()->collision.narrowphase_algorithm = NARROWPHASE_HYBRID_MPR;

	// ============================== Bodies ==============================

	// ============================== Particles ==============================

	// Particle size distribution (PSD) properties
	double particleDiameter[maxParticleTypes];
	double percentbyWeight[maxParticleTypes];
	double percentbyNumber[maxParticleTypes];
	utils::MixtureType particleType[maxParticleTypes];

	switch (distrib) {
	case UNIFORM_BEADS_5mm: // Uniform size 5 mm diameter beads
		numParticleTypes = 1;
		particleType[0] = utils::SPHERE;
		particleDiameter[0] = 0.005; // m

		percentbyWeight[0] = 1.00; // x100 %
		percentbyNumber[0] = 1.00; // x100 %
		break;
	case MONO_SPHERES_1mm: // O'Sullivan and Cui (2009)
		numParticleTypes = 1;
		particleType[0] = utils::SPHERE;
		particleDiameter[0] = 0.001; // m

		percentbyWeight[0] = 1.00; // x100 %
		percentbyNumber[0] = 1.00; // x100 %
		break;
	case TRI_SPHERES_4mm_5mm_6mm: // O'Sullivan and Cui (2009)
		numParticleTypes = 3;
		particleType[0] = utils::SPHERE;
		particleType[1] = utils::SPHERE;
		particleType[2] = utils::SPHERE;
		particleDiameter[0] = 0.004; // m
		particleDiameter[1] = 0.005; // m
		particleDiameter[2] = 0.006; // m

		// The following is correct:
		percentbyNumber[0] = 1.0 / 3; // x100 %
		percentbyNumber[1] = 1.0 / 3; // x100 %
		percentbyNumber[2] = 1.0 / 3; // x100 %

		// The following is incorrect...
		percentbyWeight[0] = 1.0 / 3; // x100 %
		percentbyWeight[1] = 1.0 / 3; // x100 %
		percentbyWeight[2] = 1.0 / 3; // x100 %
		break;
	case OTTAWA_SAND: // ASTM C 778-06 standard graded Ottawa sand
		numParticleTypes = 4;
		particleType[0] = utils::SPHERE;
		particleType[1] = utils::SPHERE;
		particleType[2] = utils::SPHERE;
		particleType[3] = utils::SPHERE;
		particleDiameter[0] = 0.0003; // m
		particleDiameter[1] = 0.0004; // m
		particleDiameter[2] = 0.0006; // m
		particleDiameter[3] = 0.0008; // m

		// The following is correct:
		percentbyWeight[0] = 0.20; // x100 %
		percentbyWeight[1] = 0.45; // x100 %
		percentbyWeight[2] = 0.30; // x100 %
		percentbyWeight[3] = 0.05; // x100 %

		// The following is incorrect...
		percentbyNumber[0] = 0.20; // x100 %
		percentbyNumber[1] = 0.45; // x100 %
		percentbyNumber[2] = 0.30; // x100 %
		percentbyNumber[3] = 0.05; // x100 %
		break;
	case MONTEREY_SAND: // test
		numParticleTypes = 4;
		particleType[0] = utils::SPHERE;
		particleType[1] = utils::CYLINDER;
		particleType[2] = utils::BOX;
		particleType[3] = utils::CYLINDER;
		particleDiameter[0] = 0.0003; // m
		particleDiameter[1] = 0.0004; // m
		particleDiameter[2] = 0.0006; // m
		particleDiameter[3] = 0.0008; // m

		// The following is correct:
		percentbyWeight[0] = 0.20; // x100 %
		percentbyWeight[1] = 0.45; // x100 %
		percentbyWeight[2] = 0.30; // x100 %
		percentbyWeight[3] = 0.05; // x100 %

		// The following is incorrect...
		percentbyNumber[0] = 0.20; // x100 %
		percentbyNumber[1] = 0.45; // x100 %
		percentbyNumber[2] = 0.30; // x100 %
		percentbyNumber[3] = 0.05; // x100 %
		break;

	case CAT_LIMESTONE: // test
		numParticleTypes = 5;
		particleType[0] = utils::SPHERE;
		particleType[1] = utils::SPHERE;
		particleType[2] = utils::SPHERE;
		particleType[3] = utils::SPHERE;
		particleType[4] = utils::SPHERE;
		particleDiameter[0] = 0.00953; // m
		particleDiameter[1] = 0.012; // m
		particleDiameter[2] = 0.015; // m
		particleDiameter[3] = 0.017; // m
		particleDiameter[4] = 0.01905; // m

		// The following is correct:
		percentbyWeight[0] = 0.15; // x100 %
		percentbyWeight[1] = 0.20; // x100 %
		percentbyWeight[2] = 0.30; // x100 %
		percentbyWeight[3] = 0.20; // x100 %
		percentbyWeight[4] = 0.15; // x100 %
		break;
	case CAT_LIMESTONE_2: // test
		numParticleTypes = 4;
		particleType[0] = utils::SPHERE;
		particleType[1] = utils::SPHERE;
		particleType[2] = utils::SPHERE;
		particleType[3] = utils::SPHERE;
		particleDiameter[0] = 0.00953; // m
		particleDiameter[1] = 0.013; // m
		particleDiameter[2] = 0.016; // m
		particleDiameter[3] = 0.01905; // m

		// The following is correct:
		percentbyWeight[0] = 0.20; // x100 %
		percentbyWeight[1] = 0.45; // x100 %
		percentbyWeight[2] = 0.30; // x100 %
		percentbyWeight[3] = 0.05; // x100 %
		break;
	case CAT_LIMESTONE_3: // test
		numParticleTypes = 4;
		particleType[0] = utils::SPHERE;
		particleType[1] = utils::SPHERE;
		particleType[2] = utils::SPHERE;
		particleType[3] = utils::SPHERE;
		particleDiameter[0] = 0.00953; // m
		particleDiameter[1] = 0.013; // m
		particleDiameter[2] = 0.016; // m
		particleDiameter[3] = 0.01905; // m

		// The following is correct:
		percentbyWeight[0] = 0.05; // x100 %
		percentbyWeight[1] = 0.20; // x100 %
		percentbyWeight[2] = 0.30; // x100 %
		percentbyWeight[3] = 0.45; // x100 %
		break;
	}


	// Particle material properties
	double particleDensity[maxParticleTypes];
	double particleMass[maxParticleTypes];
	float Y[maxParticleTypes];
	float nu[maxParticleTypes];
	float COR[maxParticleTypes];
	float mu[maxParticleTypes];

	switch (material) {
	case GLASS:
		for (i = 0; i < numParticleTypes; i++) {
			particleDensity[i] = 2550; // kg/m^3
			particleMass[i] = particleDensity[i] * CH_C_PI * particleDiameter[i] * particleDiameter[i] * particleDiameter[i] / 6.0;
			Y[i] = 4.0e7; // Pa (about 1000 times too soft)
			nu[i] = 0.22f;
			COR[i] = 0.87f;
			mu[i] = 0.18f; // particle-on-particle friction
		}
		break;
	case STEEL:
		for (i = 0; i < numParticleTypes; i++) {
			particleDensity[i] = 7800; // kg/m^3
			particleMass[i] = particleDensity[i] * CH_C_PI * particleDiameter[i] * particleDiameter[i] * particleDiameter[i] / 6.0;
			Y[i] = 2.0e8; // Pa (about 1000 times too soft)
			nu[i] = 0.28f;
			COR[i] = COR_value;//0.597f;
			mu[i] = mu_value; // particle-on-particle friction (O'Sullivan and Cui, 2009)
		}
		break;
	case QUARTZ:
		for (i = 0; i < numParticleTypes; i++) {
			particleDensity[i] = 2650; // kg/m^3
			particleMass[i] = particleDensity[i] * CH_C_PI * particleDiameter[i] * particleDiameter[i] * particleDiameter[i] / 6.0;
			Y[i] = 8.0e7; // Pa (about 1000 times too soft)
			nu[i] = 0.3f;
			COR[i] = 0.5f;
			mu[i] = 0.5f; // particle-on-particle friction
		}
		break;
	case CLAY:
		for (i = 0; i < numParticleTypes; i++) {
			particleDensity[i] = 1800; // kg/m^3
			particleMass[i] = particleDensity[i] * CH_C_PI * particleDiameter[i] * particleDiameter[i] * particleDiameter[i] / 6.0;
			Y[i] = 1.0e7; // Pa (about 1000 times too soft) // true -> estimated value is 1e10
			nu[i] = 0.3f;
			COR[i] = 0.2f;
			mu[i] = 0.3f; // particle-on-particle friction
		}
		break;
	case LIMESTONE:
		for (i = 0; i < numParticleTypes; i++) {
			particleDensity[i] = 1600; // kg/m^3
			particleMass[i] = particleDensity[i] * CH_C_PI * particleDiameter[i] * particleDiameter[i] * particleDiameter[i] / 6.0;
			Y[i] = 6.0e7; // Pa (about 1000 times too soft) // true -> estimated value is 6.0e10
			nu[i] = 0.28f;
			COR[i] = 0.87f;
			mu[i] = 0.75f; // particle-on-particle friction
		}
		break;
	case LIMESTONE_DENSE:
		for (i = 0; i < numParticleTypes; i++) {
			particleDensity[i] = 6000;//2500; // kg/m^3
			particleMass[i] = particleDensity[i] * CH_C_PI * particleDiameter[i] * particleDiameter[i] * particleDiameter[i] / 6.0;
			Y[i] = 6.0e7; // Pa (about 1000 times too soft) // true -> estimated value is 6.0e10
			nu[i] = 0.28f;
			COR[i] = 0.87f;
			mu[i] = 0.75f;//0.9f;//0.75f; // particle-on-particle friction
		}
		break;
	}

	// calculating maximal/minimal parameters
	double max_diameter = 0;
	double max_mass = 0;
	double Y_max = 0;
	double nu_max = 0;
	double min_diameter = 1e30;
	double min_mass = 1e30;
	double Y_min = 1e30;
	double nu_min = 1e30;

	for (i = 0; i < numParticleTypes; i++) {
		if (particleDiameter[i] > max_diameter) max_diameter = particleDiameter[i];
		if (particleDiameter[i] < min_diameter) min_diameter = particleDiameter[i];
		if (particleMass[i] > max_mass) max_mass = particleMass[i];
		if (particleMass[i] < min_mass) min_mass = particleMass[i];
		if (Y[i] > Y_max) Y_max = Y[i];
		if (Y[i] < Y_min) Y_min = Y[i];
		if (nu[i] > nu_max) nu_max = nu[i];
		if (nu[i] < nu_min) nu_min = nu[i];
	}

	double thickness = 1.0*max_diameter;
	double wall_mass = 1.0*max_mass;



#ifndef USE_DEM
	my_system->GetSettings()->collision.collision_envelope = 0.05 * max_diameter;//0.00 * max_diameter;
#endif

//#ifdef USE_DEM
//
//
//	// auto time-step - optional 
//	if (auto_time_step == true) {
//		double sigma_max = 0;
//		double F_max, Q_max, K_max;
//		double max_contact_normal_strain;
//
//		if (sigma_a > sigma_max) sigma_max = sigma_a;
//		if (sigma_b > sigma_max) sigma_max = sigma_b;
//		if (sigma_c > sigma_max) sigma_max = sigma_c;
//
//		// For Hertzian Contact:
//		F_max = max_diameter*max_diameter*sigma_max;
//		Q_max = pow(3 * (1 - nu_min*nu_min)*F_max*max_diameter / (8 * Y_min), 1.0 / 3);
//
//		// For Hookean Contact:
//		//	  max_contact_normal_strain = sigma_max/Y_min;
//		//	  Q_max = max_diameter*sqrt(max_contact_normal_strain);
//
//		K_max = Y_max*Q_max / (1 - nu_max*nu_max);
//		time_step = 0.2*sqrt(min_mass / K_max);
//		cout << "** simulation time step = " << time_step << "\n";
////
////#ifndef USE_DEM
////		time_step = time_step * 50;
////#endif
//	}
//
//#endif

	// because uniform simulation is unstable
	// and tri_spheres is stable
//#ifdef USE_DEM
//	time_step = time_step*0.2;
//#else
//	time_step = time_step*0.5;
//#endif


	// ============================== Container ==============================

	int particle_Id = 1;  // first particle id
	int wall_1Id = -1;
	int wall_2Id = -2;
	int wall_3Id = -3;
	int wall_4Id = -4;
	int wall_5Id = -5;
	int wall_6Id = -6;
	int wall_7Id = -7;
	int wall_8Id = -8;
	int wall_9Id = -9;
	int wall_10Id = -10;
	int wall_11Id = -11;
	int wall_12Id = -12;
	int wall_13Id = -13;
	int wall_14Id = -14;

	// Define five quaternions representing:
	// - a rotation of -90 degrees around x (z2y)
	// - a rotation of +90 degrees around y (z2x)
	// - a rotation of +30 degrees around z (z30)
	// - a rotation of +45 degrees around z (z45)
	// - a rotation of +60 degrees around z (z60)

	ChQuaternion<> z2y;
	ChQuaternion<> z2x;
	ChQuaternion<> z30;
	ChQuaternion<> z45;
	ChQuaternion<> z60;
	z2y.Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));
	z2x.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(0, 1, 0));
	z30.Q_from_AngAxis(CH_C_PI / 6, ChVector<>(0, 0, 1));
	z45.Q_from_AngAxis(CH_C_PI / 4, ChVector<>(0, 0, 1));
	z60.Q_from_AngAxis(CH_C_PI / 3, ChVector<>(0, 0, 1));

	// Containing wall material properties
	float Y_ext, nu_ext, COR_ext, mu_ext;

	switch (geom) {
	case HARTL_OOI:
		Y_ext = 1.0e7; // Pa
		nu_ext = 0.3f;
		COR_ext = 0.5f;
		mu_ext = 0.13f; // particle-on-wall friction 
	case OSULLIVAN:
		Y_ext = 1.0e7; // Pa
		nu_ext = 0.3f;
		COR_ext = COR_ext_value;//0.5f;
		mu_ext = mu_ext_value;//0.228f;//2 * 0.228 - 0.096; //0.228f; // particle-on-wall friction
		break;

	case STANDARD_BOX:
	case SMALL_BOX:
	case JENIKE_SHEAR:
	case STANDARD_TRIAXIAL:
	case SMALL_TRIAXIAL:
		Y_ext = 1.0e7; // Pa
		nu_ext = 0.3f;
		COR_ext = 0.5f;
		//	  mu_ext = 0.01f; // particle-on-wall friction
		mu_ext = 0.1f; // particle-on-wall friction
		break;
	case TRIAXIAL_CLAY: 
		Y_ext = 1.0e7; // Pa
		nu_ext = 0.3f;
		COR_ext = 0.2f;
		mu_ext = 0.4f; // particle-on-wall friction // 0.1 more than particle-on-particle friction (it's just my guess...)
		break;
	case TRIAXIAL_LIMESTONE:
		Y_ext = 6.0e7; // Pa
		nu_ext = 0.28f;
		COR_ext = 0.87f;
		mu_ext = 1.05f;//1.1f;//1.05f;//0.75f; // particle-on-wall friction 
		break;
	}

	// Create wall material

#ifdef USE_DEM
	auto mat_ext = std::make_shared<ChMaterialSurfaceDEM>();
	mat_ext->SetYoungModulus(Y_ext);
	mat_ext->SetPoissonRatio(nu_ext);
	mat_ext->SetRestitution(COR_ext);
	mat_ext->SetFriction(mu_ext);
#else
	auto mat_ext = std::make_shared<ChMaterialSurface>();
	mat_ext = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);
	mat_ext->SetRestitution(COR_ext);
	mat_ext->SetFriction(mu_ext);
#endif

	// Unconsolidated specimen dimensions
	double Lx0, Ly0, Lz0;
	bool cylinder = false;

	switch (geom) {
	case HARTL_OOI:
		Lx0 = Ly0 = 0.14; // m
		Lz0 = 0.10; // m
		cylinder = true;
		break;
	case OSULLIVAN:
		Lx0 = Ly0 = 0.101; // m
		//Lz0 = 0.765; // m
		{
			const char* text = argv[5];
			Lz0 = atof(text);// 0.765;
		}
		cylinder = true;
		break;
	case STANDARD_BOX:
		Lx0 = 0.06; // m
		Ly0 = 0.06; // m
		Lz0 = 0.12; // m
		break;
	case SMALL_BOX:
		Lx0 = 0.01; // m
		Ly0 = 0.01; // m
		Lz0 = 0.10; // m
		break;
	case JENIKE_SHEAR:
		Lx0 = Ly0 = 0.06; // m
		Lz0 = 0.12; // m
		cylinder = true;
		break;
	case STANDARD_TRIAXIAL:
		Lx0 = Ly0 = 0.06; // m
		Lz0 = 0.24; // m
		cylinder = true;
		break;
	case SMALL_TRIAXIAL:
		Lx0 = Ly0 = 0.02; // m
		Lz0 = 0.06; // m
		cylinder = true;
		break;
	case TRIAXIAL_CLAY:
		Lx0 = Ly0 = 0.075; // m
		Lz0 = 0.15; // m
		cylinder = true;
		break;
	case TRIAXIAL_LIMESTONE:
		Lx0 = Ly0 = 0.1016; // m
		Lz0 = 0.430; // m // -> to obtain 0.2025 after settling
		cylinder = true;
		break;
	}

	// Start creating and adding the walls

	double aLx0 = Lx0 / 2 + thickness;
	double aLy0 = Ly0 / 2 + thickness;
	double aLz0 = Lz0 / 2 + thickness;

	// Create wall 1 (bottom)

	std::shared_ptr<ChBody>  wall_1;

	wall_1->SetIdentifier(wall_1Id);
	wall_1->SetMass(wall_mass);
	wall_1->SetPos(ChVector<>(0, 0, -thickness / 2));
	wall_1->SetBodyFixed(true);
	wall_1->SetCollide(true);

	wall_1->SetMaterialSurface(mat_ext);

	wall_1->GetCollisionModel()->ClearModel();
	wall_1->GetCollisionModel()->AddBox(aLx0, aLy0, thickness / 2, ChVector<>(0, 0, 0));
	wall_1->GetCollisionModel()->SetFamily(1);
	wall_1->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
	wall_1->GetCollisionModel()->BuildModel();

	// Create wall 2 (top)

	std::shared_ptr<ChBody>  wall_2;

	wall_2->SetIdentifier(wall_2Id);
	wall_2->SetMass(wall_mass);
	wall_2->SetPos(ChVector<>(0, 0, Lz0 + thickness / 2));
	wall_2->SetBodyFixed(false);
	wall_2->SetCollide(true);

	wall_2->SetMaterialSurface(mat_ext);

	wall_2->GetCollisionModel()->ClearModel();
	wall_2->GetCollisionModel()->AddBox(aLx0, aLy0, thickness / 2, ChVector<>(0, 0, 0));
	wall_2->GetCollisionModel()->SetFamily(1);
	wall_2->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
	wall_2->GetCollisionModel()->BuildModel();

	// Create wall 3 (side)

	std::shared_ptr<ChBody>  wall_3;

	wall_3->SetIdentifier(wall_3Id);
	wall_3->SetMass(wall_mass);
	wall_3->SetPos(ChVector<>(-(Lx0 / 2 + thickness / 2), 0, Lz0 / 2));
	wall_3->SetBodyFixed(start_walls_fixed);
	wall_3->SetCollide(true);

	wall_3->SetMaterialSurface(mat_ext);

	wall_3->GetCollisionModel()->ClearModel();
	wall_3->GetCollisionModel()->AddBox(thickness / 2, aLy0, aLz0, ChVector<>(0, 0, 0));
	wall_3->GetCollisionModel()->SetFamily(1);
	wall_3->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
	wall_3->GetCollisionModel()->BuildModel();

	// Create wall 4 (side)

	std::shared_ptr<ChBody>  wall_4;

	wall_4->SetIdentifier(wall_4Id);
	wall_4->SetMass(wall_mass);
	wall_4->SetPos(ChVector<>(Lx0 / 2 + thickness / 2, 0, Lz0 / 2));
	wall_4->SetBodyFixed(start_walls_fixed);
	wall_4->SetCollide(true);

	wall_4->SetMaterialSurface(mat_ext);

	wall_4->GetCollisionModel()->ClearModel();
	wall_4->GetCollisionModel()->AddBox(thickness / 2, aLy0, aLz0, ChVector<>(0, 0, 0));
	wall_4->GetCollisionModel()->SetFamily(1);
	wall_4->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
	wall_4->GetCollisionModel()->BuildModel();

	// Create wall 5 (side)

	std::shared_ptr<ChBody>  wall_5;

	wall_5->SetIdentifier(wall_5Id);
	wall_5->SetMass(wall_mass);
	wall_5->SetPos(ChVector<>(0, -(Ly0 / 2 + thickness / 2), Lz0 / 2));
	wall_5->SetBodyFixed(start_walls_fixed);
	wall_5->SetCollide(true);

	wall_5->SetMaterialSurface(mat_ext);

	wall_5->GetCollisionModel()->ClearModel();
	wall_5->GetCollisionModel()->AddBox(aLx0, thickness / 2, aLz0, ChVector<>(0, 0, 0));
	wall_5->GetCollisionModel()->SetFamily(1);
	wall_5->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
	wall_5->GetCollisionModel()->BuildModel();

	// Create wall 6 (side)

	std::shared_ptr<ChBody>  wall_6;

	wall_6->SetIdentifier(wall_6Id);
	wall_6->SetMass(wall_mass);
	wall_6->SetPos(ChVector<>(0, Ly0 / 2 + thickness / 2, Lz0 / 2));
	wall_6->SetBodyFixed(start_walls_fixed);
	wall_6->SetCollide(true);

	wall_6->SetMaterialSurface(mat_ext);

	wall_6->GetCollisionModel()->ClearModel();
	wall_6->GetCollisionModel()->AddBox(aLx0, thickness / 2, aLz0, ChVector<>(0, 0, 0));
	wall_6->GetCollisionModel()->SetFamily(1);
	wall_6->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
	wall_6->GetCollisionModel()->BuildModel();

	// For cylinder only

	// Create wall 7 (side)

	std::shared_ptr<ChBody>  wall_7;

	wall_7->SetIdentifier(wall_7Id);
	wall_7->SetMass(wall_mass);
	wall_7->SetPos(ChVector<>(-(Lx0 / 2 + thickness / 2) * cos(CH_C_PI / 6),
		-(Ly0 / 2 + thickness / 2) * sin(CH_C_PI / 6),
		Lz0 / 2));
	wall_7->SetRot(z30);
	wall_7->SetBodyFixed(start_walls_fixed);
	wall_7->SetCollide(true);

	wall_7->SetMaterialSurface(mat_ext);

	wall_7->GetCollisionModel()->ClearModel();
	wall_7->GetCollisionModel()->AddBox(thickness / 2, aLy0, aLz0, ChVector<>(0, 0, 0));
	wall_7->GetCollisionModel()->SetFamily(1);
	wall_7->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
	wall_7->GetCollisionModel()->BuildModel();

	// Create wall 8 (side)

	std::shared_ptr<ChBody>  wall_8;

	wall_8->SetIdentifier(wall_8Id);
	wall_8->SetMass(wall_mass);
	wall_8->SetPos(ChVector<>((Lx0 / 2 + thickness / 2) * cos(CH_C_PI / 6),
		(Ly0 / 2 + thickness / 2) * sin(CH_C_PI / 6),
		Lz0 / 2));
	wall_8->SetRot(z30);
	wall_8->SetBodyFixed(start_walls_fixed);
	wall_8->SetCollide(true);

	wall_8->SetMaterialSurface(mat_ext);

	wall_8->GetCollisionModel()->ClearModel();
	wall_8->GetCollisionModel()->AddBox(thickness / 2, aLy0, aLz0, ChVector<>(0, 0, 0));
	wall_8->GetCollisionModel()->SetFamily(1);
	wall_8->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
	wall_8->GetCollisionModel()->BuildModel();

	// Create wall 9 (side)

	std::shared_ptr<ChBody>  wall_9;

	wall_9->SetIdentifier(wall_9Id);
	wall_9->SetMass(wall_mass);
	wall_9->SetPos(ChVector<>(-(Lx0 / 2 + thickness / 2) * cos(CH_C_PI / 3),
		-(Ly0 / 2 + thickness / 2) * sin(CH_C_PI / 3),
		Lz0 / 2));
	wall_9->SetRot(z60);
	wall_9->SetBodyFixed(start_walls_fixed);
	wall_9->SetCollide(true);

	wall_9->SetMaterialSurface(mat_ext);

	wall_9->GetCollisionModel()->ClearModel();
	wall_9->GetCollisionModel()->AddBox(thickness / 2, aLy0, aLz0, ChVector<>(0, 0, 0));
	wall_9->GetCollisionModel()->SetFamily(1);
	wall_9->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
	wall_9->GetCollisionModel()->BuildModel();

	// Create wall 10 (side)

	std::shared_ptr<ChBody> wall_10;

	wall_10->SetIdentifier(wall_10Id);
	wall_10->SetMass(wall_mass);
	wall_10->SetPos(ChVector<>((Lx0 / 2 + thickness / 2) * cos(CH_C_PI / 3),
		(Ly0 / 2 + thickness / 2) * sin(CH_C_PI / 3),
		Lz0 / 2));
	wall_10->SetRot(z60);
	wall_10->SetBodyFixed(start_walls_fixed);
	wall_10->SetCollide(true);

	wall_10->SetMaterialSurface(mat_ext);

	wall_10->GetCollisionModel()->ClearModel();
	wall_10->GetCollisionModel()->AddBox(thickness / 2, aLy0, aLz0, ChVector<>(0, 0, 0));
	wall_10->GetCollisionModel()->SetFamily(1);
	wall_10->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
	wall_10->GetCollisionModel()->BuildModel();

	// Create wall 11 (side)

	std::shared_ptr<ChBody> wall_11;

	wall_11->SetIdentifier(wall_11Id);
	wall_11->SetMass(wall_mass);
	wall_11->SetPos(ChVector<>((Lx0 / 2 + thickness / 2) * sin(CH_C_PI / 6),
		-(Ly0 / 2 + thickness / 2) * cos(CH_C_PI / 6),
		Lz0 / 2));
	wall_11->SetRot(z30);
	wall_11->SetBodyFixed(start_walls_fixed);
	wall_11->SetCollide(true);

	wall_11->SetMaterialSurface(mat_ext);

	wall_11->GetCollisionModel()->ClearModel();
	wall_11->GetCollisionModel()->AddBox(aLx0, thickness / 2, aLz0, ChVector<>(0, 0, 0));
	wall_11->GetCollisionModel()->SetFamily(1);
	wall_11->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
	wall_11->GetCollisionModel()->BuildModel();

	// Create wall 12 (side)

	std::shared_ptr<ChBody> wall_12;

	wall_12->SetIdentifier(wall_12Id);
	wall_12->SetMass(wall_mass);
	wall_12->SetPos(ChVector<>(-(Lx0 / 2 + thickness / 2) * sin(CH_C_PI / 6),
		(Ly0 / 2 + thickness / 2) * cos(CH_C_PI / 6),
		Lz0 / 2));
	wall_12->SetRot(z30);
	wall_12->SetBodyFixed(start_walls_fixed);
	wall_12->SetCollide(true);

	wall_12->SetMaterialSurface(mat_ext);

	wall_12->GetCollisionModel()->ClearModel();
	wall_12->GetCollisionModel()->AddBox(aLx0, thickness / 2, aLz0, ChVector<>(0, 0, 0));
	wall_12->GetCollisionModel()->SetFamily(1);
	wall_12->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
	wall_12->GetCollisionModel()->BuildModel();

	// Create wall 13 (side)

	std::shared_ptr<ChBody>  wall_13;

	wall_13->SetIdentifier(wall_13Id);
	wall_13->SetMass(wall_mass);
	wall_13->SetPos(ChVector<>((Lx0 / 2 + thickness / 2) * sin(CH_C_PI / 3),
		-(Ly0 / 2 + thickness / 2) * cos(CH_C_PI / 3),
		Lz0 / 2));
	wall_13->SetRot(z60);
	wall_13->SetBodyFixed(start_walls_fixed);
	wall_13->SetCollide(true);

	wall_13->SetMaterialSurface(mat_ext);

	wall_13->GetCollisionModel()->ClearModel();
	wall_13->GetCollisionModel()->AddBox(aLx0, thickness / 2, aLz0, ChVector<>(0, 0, 0));
	wall_13->GetCollisionModel()->SetFamily(1);
	wall_13->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
	wall_13->GetCollisionModel()->BuildModel();

	// Create wall 14 (side)

	std::shared_ptr<ChBody>  wall_14;

	wall_14->SetIdentifier(wall_14Id);
	wall_14->SetMass(wall_mass);
	wall_14->SetPos(ChVector<>(-(Lx0 / 2 + thickness / 2) * sin(CH_C_PI / 3),
		(Ly0 / 2 + thickness / 2) * cos(CH_C_PI / 3),
		Lz0 / 2));
	wall_14->SetRot(z60);
	wall_14->SetBodyFixed(start_walls_fixed);
	wall_14->SetCollide(true);

	wall_14->SetMaterialSurface(mat_ext);

	wall_14->GetCollisionModel()->ClearModel();
	wall_14->GetCollisionModel()->AddBox(aLx0, thickness / 2, aLz0, ChVector<>(0, 0, 0));
	wall_14->GetCollisionModel()->SetFamily(1);
	wall_14->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(1);
	wall_14->GetCollisionModel()->BuildModel();

	// Add all walls to system

	my_system->AddBody(wall_1);
	my_system->AddBody(wall_2);

	my_system->AddBody(wall_3);
	my_system->AddBody(wall_4);
	my_system->AddBody(wall_5);
	my_system->AddBody(wall_6);

	if (cylinder == true) {
		my_system->AddBody(wall_7);
		my_system->AddBody(wall_8);
		my_system->AddBody(wall_9);
		my_system->AddBody(wall_10);
		my_system->AddBody(wall_11);
		my_system->AddBody(wall_12);
		my_system->AddBody(wall_13);
		my_system->AddBody(wall_14);
	}

	if (visible_walls == true) {
		ChVector<> box_1_size(aLx0, aLy0, thickness / 2);
		ChVector<> box_1_pos(0., 0., 0.);

		ChVector<> box_2_size(thickness / 2, aLy0, aLz0);
		ChVector<> box_2_pos(0., 0., 0.);

		ChVector<> box_3_size(aLx0, thickness / 2, aLz0);
		ChVector<> box_3_pos(0., 0., 0.);
	


		utils::AddBoxGeometry(wall_1.get(), box_1_size, box_1_pos, QUNIT, true);
		utils::AddBoxGeometry(wall_2.get(), box_1_size, box_1_pos, QUNIT, true);

		utils::AddBoxGeometry(wall_3.get(), box_2_size, box_2_pos, QUNIT, true);
		utils::AddBoxGeometry(wall_4.get(), box_2_size, box_2_pos, QUNIT, true);

		utils::AddBoxGeometry(wall_5.get(), box_3_size, box_3_pos, QUNIT, true);
		utils::AddBoxGeometry(wall_6.get(), box_3_size, box_3_pos, QUNIT, true);


		if (cylinder == true) {
			utils::AddBoxGeometry(wall_7.get(), box_2_size, box_2_pos, QUNIT, true);
			utils::AddBoxGeometry(wall_8.get(), box_2_size, box_2_pos, QUNIT, true);
			utils::AddBoxGeometry(wall_9.get(), box_2_size, box_2_pos, QUNIT, true);
			utils::AddBoxGeometry(wall_10.get(), box_2_size, box_2_pos, QUNIT, true);

			utils::AddBoxGeometry(wall_11.get(), box_3_size, box_3_pos, QUNIT, true);
			utils::AddBoxGeometry(wall_12.get(), box_3_size, box_3_pos, QUNIT, true);
			utils::AddBoxGeometry(wall_13.get(), box_3_size, box_3_pos, QUNIT, true);
			utils::AddBoxGeometry(wall_14.get(), box_3_size, box_3_pos, QUNIT, true);
		}
	}

	// Start creating and adding the links

	// Create prismatic (translational) joint between the plate and the ground.
	// The translational axis of a prismatic joint is along the Z axis of the
	// specified joint coordinate system.

	auto prismatic_wall_2_1 = std::make_shared<ChLinkLockPrismatic>();
	prismatic_wall_2_1->SetName("prismatic_wall_2_1");
	prismatic_wall_2_1->Initialize(wall_2, wall_1, ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));
	my_system->AddLink(prismatic_wall_2_1);

	// Create prismatic (translational) joints between each of the four walls and the ground.

	auto prismatic_wall_3_1 = std::make_shared<ChLinkLockPrismatic>();
	prismatic_wall_3_1->SetName("prismatic_wall_3_1");
	prismatic_wall_3_1->Initialize(wall_3, wall_1, ChCoordsys<>(ChVector<>(0, 0, 0), z2x));
	 
	auto prismatic_wall_4_1 = std::make_shared<ChLinkLockPrismatic>();
	prismatic_wall_4_1->SetName("prismatic_wall_4_1");
	prismatic_wall_4_1->Initialize(wall_4, wall_1, ChCoordsys<>(ChVector<>(0, 0, 0), z2x));

	auto prismatic_wall_5_1 = std::make_shared<ChLinkLockPrismatic>();
	prismatic_wall_5_1->SetName("prismatic_wall_5_1");
	prismatic_wall_5_1->Initialize(wall_5, wall_1, ChCoordsys<>(ChVector<>(0, 0, 0), z2y));

	auto prismatic_wall_6_1 = std::make_shared<ChLinkLockPrismatic>();
	prismatic_wall_6_1->SetName("prismatic_wall_6_1");
	prismatic_wall_6_1->Initialize(wall_6, wall_1, ChCoordsys<>(ChVector<>(0, 0, 0), z2y));

	ChQuaternion<> z2wall_7_8 = z30%z2x;
	ChQuaternion<> z2wall_9_10 = z60%z2x;
	ChQuaternion<> z2wall_11_12 = z30%z2y;
	ChQuaternion<> z2wall_13_14 = z60%z2y;

	// For cylinder only
	// Create prismatic (translational) joints between an additional eight walls and the ground.

	auto prismatic_wall_7_1 = std::make_shared<ChLinkLockPrismatic>();
	prismatic_wall_7_1->SetName("prismatic_wall_7_1");
	prismatic_wall_7_1->Initialize(wall_7, wall_1, ChCoordsys<>(ChVector<>(0, 0, 0), z2wall_7_8));

	auto prismatic_wall_8_1 = std::make_shared<ChLinkLockPrismatic>();
	prismatic_wall_8_1->SetName("prismatic_wall_8_1");
	prismatic_wall_8_1->Initialize(wall_8, wall_1, ChCoordsys<>(ChVector<>(0, 0, 0), z2wall_7_8));

	auto prismatic_wall_9_1 = std::make_shared<ChLinkLockPrismatic>();
	prismatic_wall_9_1->SetName("prismatic_wall_9_1");
	prismatic_wall_9_1->Initialize(wall_9, wall_1, ChCoordsys<>(ChVector<>(0, 0, 0), z2wall_9_10));

	auto prismatic_wall_10_1 = std::make_shared<ChLinkLockPrismatic>();
	prismatic_wall_10_1->SetName("prismatic_wall_10_1");
	prismatic_wall_10_1->Initialize(wall_10, wall_1, ChCoordsys<>(ChVector<>(0, 0, 0), z2wall_9_10));

	auto prismatic_wall_11_1 = std::make_shared<ChLinkLockPrismatic>();
	prismatic_wall_11_1->SetName("prismatic_wall_11_1");
	prismatic_wall_11_1->Initialize(wall_11, wall_1, ChCoordsys<>(ChVector<>(0, 0, 0), z2wall_11_12));

	auto prismatic_wall_12_1 = std::make_shared<ChLinkLockPrismatic>();
	prismatic_wall_12_1->SetName("prismatic_wall_12_1");
	prismatic_wall_12_1->Initialize(wall_12, wall_1, ChCoordsys<>(ChVector<>(0, 0, 0), z2wall_11_12));

	auto prismatic_wall_13_1 = std::make_shared<ChLinkLockPrismatic>();
	prismatic_wall_13_1->SetName("prismatic_wall_13_1");
	prismatic_wall_13_1->Initialize(wall_13, wall_1, ChCoordsys<>(ChVector<>(0, 0, 0), z2wall_13_14));

	auto prismatic_wall_14_1 = std::make_shared<ChLinkLockPrismatic>();
	prismatic_wall_14_1->SetName("prismatic_wall_14_1");
	prismatic_wall_14_1->Initialize(wall_14, wall_1, ChCoordsys<>(ChVector<>(0, 0, 0), z2wall_13_14));


	// Start creating and adding the particles
	// Create the particle generator

	utils::Generator gen(my_system);

#ifdef USE_DEM
	auto mat = std::make_shared<ChMaterialSurfaceDEM>();
	mat->SetYoungModulus(Y_ext);
	mat->SetFriction(mu_ext);
	mat->SetRestitution(COR_ext);
#else
	auto mat = std::make_shared<ChMaterialSurface>();
#endif

	for (i = 0; i < numParticleTypes; i++) {
		auto type = gen.AddMixtureIngredient(particleType[i], percentbyNumber[i]);
#ifdef USE_DEM
		type->setDefaultMaterial(mat);
#else
		mat[i] = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);
		mat[i]->SetRestitution(COR[i]);
		mat[i]->SetFriction(mu[i]);
		type->setDefaultMaterialDVI(mat[i]);
#endif
		type->setDefaultDensity(particleDensity[i]);
		type->setDefaultSize(particleDiameter[i] / 2);
	}

	// Ensure that all generated particle bodies will have positive IDs.
	gen.setBodyIdentifier(particle_Id);

	// Generate the particles

	if (cylinder == true) {
		gen.createObjectsCylinderZ(utils::POISSON_DISK, max_diameter,
			ChVector<>(0, 0, Lz0 / 2),
			(Lx0 - max_diameter) / 2, (Lz0 - max_diameter) / 2);
	}
	else {
		gen.createObjectsBox(utils::POISSON_DISK, max_diameter,
			ChVector<>(0, 0, Lz0 / 2),
			ChVector<>((Lx0 - max_diameter) / 2, (Ly0 - max_diameter) / 2, (Lz0 - max_diameter) / 2));
	}

	int num_of_spheres = gen.getTotalNumBodies();

	// ============================== Visualization ==============================

	// Create the OpenGL visualization window

#ifdef CHRONO_PARALLEL_HAS_OPENGL
	opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
	gl_window.Initialize(800, 600, title.c_str(), my_system);
	gl_window.SetCamera(ChVector<>(10 * Lx0, 0, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), max_diameter, max_diameter);
	gl_window.SetRenderMode(opengl::WIREFRAME);
#endif

	// ============================== Output ==============================

	if (cylinder == true) {
		stressStream << "time" << "\t";
		stressStream << "strain_a" << "\t" << "strain_b" << "\t";
		stressStream << "stress_a" << "\t" << "stress_b" << "\n";
	}
	else {
		stressStream << "time" << "\t";
		stressStream << "strain_a" << "\t" << "strain_b" << "\t" << "strain_c" << "\t";
		stressStream << "stress_a" << "\t" << "stress_b" << "\t" << "stress_c" << "\n";
	}

	// Begin simulation

	bool settled = false;
	int data_out_frame = 0;
	int visual_out_frame = 0;
	int checkpt_out_frame = 0;
	real3 force1, force2, force3, force4, force5;
	real3 force6, force7, force8, force9, force10;
	real3 force11, force12, force13, force14;
	double Lx, Ly, Lz;
	double L7, L8, L9, L10, L11, L12, L13, L14, diam;

	//if (dense == true)
	//	for (i = 0; i < numParticleTypes; i++) mat[i]->SetFriction(0.01f);

	TimeStepStream << time_step << "\t" << max_diameter;
	extMechParamsStream << Y_ext << "\t" << nu_ext << "\t" << COR_ext << "\t" << mu_ext;

	double time = 0;
	double exec_time = 0;

	double max_vel = 0;
	double max_acc = 0;
	double total_kinen = 0;
	double avg_kinen = 0;
	double var_kinen = 0;

	maxvelStream << num_of_spheres << "\n";

	while (my_system->GetChTime() < simulation_time) { //2 * time_step){//

		if (my_system->GetChTime() > settling_time && settled == false) {
			settled = true;
			wall_3->SetBodyFixed(false);
			wall_4->SetBodyFixed(false);
			wall_5->SetBodyFixed(false);
			wall_6->SetBodyFixed(false);
			my_system->AddLink(prismatic_wall_3_1);
			my_system->AddLink(prismatic_wall_4_1);
			my_system->AddLink(prismatic_wall_5_1);
			my_system->AddLink(prismatic_wall_6_1);
			if (cylinder == true) {
				wall_7->SetBodyFixed(false);
				wall_8->SetBodyFixed(false);
				wall_9->SetBodyFixed(false);
				wall_10->SetBodyFixed(false);
				wall_11->SetBodyFixed(false);
				wall_12->SetBodyFixed(false);
				wall_13->SetBodyFixed(false);
				wall_14->SetBodyFixed(false);
				my_system->AddLink(prismatic_wall_7_1);
				my_system->AddLink(prismatic_wall_8_1);
				my_system->AddLink(prismatic_wall_9_1);
				my_system->AddLink(prismatic_wall_10_1);
				my_system->AddLink(prismatic_wall_11_1);
				my_system->AddLink(prismatic_wall_12_1);
				my_system->AddLink(prismatic_wall_13_1);
				my_system->AddLink(prismatic_wall_14_1);
			}
			Lz0 = wall_2->GetPos().z - wall_1->GetPos().z - thickness;
			Lx0 = wall_4->GetPos().x - wall_3->GetPos().x - thickness;
			Ly0 = wall_6->GetPos().y - wall_5->GetPos().y - thickness;
		}

		Lz = wall_2->GetPos().z - wall_1->GetPos().z - thickness;
		Lx = wall_4->GetPos().x - wall_3->GetPos().x - thickness;
		Ly = wall_6->GetPos().y - wall_5->GetPos().y - thickness;

		if (cylinder == true) {
			L7 = sqrt(wall_7->GetPos().x*wall_7->GetPos().x + wall_7->GetPos().y*wall_7->GetPos().y) - thickness / 2;
			L8 = sqrt(wall_8->GetPos().x*wall_8->GetPos().x + wall_8->GetPos().y*wall_8->GetPos().y) - thickness / 2;
			L9 = sqrt(wall_9->GetPos().x*wall_9->GetPos().x + wall_9->GetPos().y*wall_9->GetPos().y) - thickness / 2;
			L10 = sqrt(wall_10->GetPos().x*wall_10->GetPos().x + wall_10->GetPos().y*wall_10->GetPos().y) - thickness / 2;
			L11 = sqrt(wall_11->GetPos().x*wall_11->GetPos().x + wall_11->GetPos().y*wall_11->GetPos().y) - thickness / 2;
			L12 = sqrt(wall_12->GetPos().x*wall_12->GetPos().x + wall_12->GetPos().y*wall_12->GetPos().y) - thickness / 2;
			L13 = sqrt(wall_13->GetPos().x*wall_13->GetPos().x + wall_13->GetPos().y*wall_13->GetPos().y) - thickness / 2;
			L14 = sqrt(wall_14->GetPos().x*wall_14->GetPos().x + wall_14->GetPos().y*wall_14->GetPos().y) - thickness / 2;
			diam = (L7 + L8 + L9 + L10 + L11 + L12 + L13 + L14) / 4;
		}

		if (settled == true) {
			wall_2->Empty_forces_accumulators();
			wall_3->Empty_forces_accumulators();
			wall_4->Empty_forces_accumulators();
			wall_5->Empty_forces_accumulators();
			wall_6->Empty_forces_accumulators();
			if (cylinder == true) {
				wall_7->Empty_forces_accumulators();
				wall_8->Empty_forces_accumulators();
				wall_9->Empty_forces_accumulators();
				wall_10->Empty_forces_accumulators();
				wall_11->Empty_forces_accumulators();
				wall_12->Empty_forces_accumulators();
				wall_13->Empty_forces_accumulators();
				wall_14->Empty_forces_accumulators();
				wall_2->Accumulate_force(ChVector<>(0, 0, -sigma_a*CH_C_PI*diam*diam / 4.0), wall_2->GetPos(), false);
				wall_3->Accumulate_force(ChVector<>(sigma_b*Lz*CH_C_PI*diam / 12.0, 0, 0), wall_3->GetPos(), false);
				wall_4->Accumulate_force(ChVector<>(-sigma_b*Lz*CH_C_PI*diam / 12.0, 0, 0), wall_4->GetPos(), false);
				wall_5->Accumulate_force(ChVector<>(0, sigma_b*Lz*CH_C_PI*diam / 12.0, 0), wall_5->GetPos(), false);
				wall_6->Accumulate_force(ChVector<>(0, -sigma_b*Lz*CH_C_PI*diam / 12.0, 0), wall_6->GetPos(), false);
				wall_7->Accumulate_force(ChVector<>(sigma_b*Lz*CH_C_PI*diam / 12.0*cos(CH_C_PI / 6),
					sigma_b*Lz*CH_C_PI*diam / 12.0*sin(CH_C_PI / 6), 0), wall_7->GetPos(), false);
				wall_8->Accumulate_force(ChVector<>(-sigma_b*Lz*CH_C_PI*diam / 12.0*cos(CH_C_PI / 6),
					-sigma_b*Lz*CH_C_PI*diam / 12.0*sin(CH_C_PI / 6), 0), wall_8->GetPos(), false);
				wall_9->Accumulate_force(ChVector<>(sigma_b*Lz*CH_C_PI*diam / 12.0*cos(CH_C_PI / 3),
					sigma_b*Lz*CH_C_PI*diam / 12.0*sin(CH_C_PI / 3), 0), wall_9->GetPos(), false);
				wall_10->Accumulate_force(ChVector<>(-sigma_b*Lz*CH_C_PI*diam / 12.0*cos(CH_C_PI / 3),
					-sigma_b*Lz*CH_C_PI*diam / 12.0*sin(CH_C_PI / 3), 0), wall_10->GetPos(), false);
				wall_11->Accumulate_force(ChVector<>(-sigma_b*Lz*CH_C_PI*diam / 12.0*sin(CH_C_PI / 6),
					sigma_b*Lz*CH_C_PI*diam / 12.0*cos(CH_C_PI / 6), 0), wall_11->GetPos(), false);
				wall_12->Accumulate_force(ChVector<>(sigma_b*Lz*CH_C_PI*diam / 12.0*sin(CH_C_PI / 6),
					-sigma_b*Lz*CH_C_PI*diam / 12.0*cos(CH_C_PI / 6), 0), wall_12->GetPos(), false);
				wall_13->Accumulate_force(ChVector<>(-sigma_b*Lz*CH_C_PI*diam / 12.0*sin(CH_C_PI / 3),
					sigma_b*Lz*CH_C_PI*diam / 12.0*cos(CH_C_PI / 3), 0), wall_13->GetPos(), false);
				wall_14->Accumulate_force(ChVector<>(sigma_b*Lz*CH_C_PI*diam / 12.0*sin(CH_C_PI / 3),
					-sigma_b*Lz*CH_C_PI*diam / 12.0*cos(CH_C_PI / 3), 0), wall_14->GetPos(), false);
			}
			else {
				wall_2->Accumulate_force(ChVector<>(0, 0, -sigma_a*Lx*Ly), wall_2->GetPos(), false);
				wall_3->Accumulate_force(ChVector<>(sigma_b*Ly*Lz, 0, 0), wall_3->GetPos(), false);
				wall_4->Accumulate_force(ChVector<>(-sigma_b*Ly*Lz, 0, 0), wall_4->GetPos(), false);
				wall_5->Accumulate_force(ChVector<>(0, sigma_c*Lx*Lz, 0), wall_5->GetPos(), false);
				wall_6->Accumulate_force(ChVector<>(0, -sigma_c*Lx*Lz, 0), wall_6->GetPos(), false);
			}
		}

		//  Do time step

#ifdef CHRONO_PARALLEL_HAS_OPENGL
		if (gl_window.Active()) {
			gl_window.DoStepDynamics(time_step);
			gl_window.Render();
		}
		else
			break;
#else
		my_system->DoStepDynamics(time_step);
#endif

		//  Output to files and screen

		time += time_step;
		exec_time += my_system->GetTimerStep();


		if (my_system->GetChTime() >= data_out_frame * data_out_step) {

			cout << endl;
			//cout << "---- Frame:          " << out_frame << endl;
			//cout << "     Sim frame:      " << sim_frame << endl;
			cout << "     Time:           " << time << endl;
			cout << "     Lowest point:   " << FindLowest(my_system) << endl;
			//cout << "     Avg. contacts:  " << num_contacts / out_steps << endl;
			cout << "     Execution time: " << exec_time << endl;
			//cout << "     Highest point (half of the container): " << FindHighest(my_system, 0.5 * Lx) << endl;
			cout << "     Highest point (full container):        " << FindHighest(my_system, Lx) << endl;



#ifndef USE_DEM
			my_system->CalculateContactForces();
#endif
			force1 = my_system->GetBodyContactForce(0);
			force2 = my_system->GetBodyContactForce(1);
			force3 = my_system->GetBodyContactForce(2);
			force4 = my_system->GetBodyContactForce(3);
			force5 = my_system->GetBodyContactForce(4);
			force6 = my_system->GetBodyContactForce(5);
			if (cylinder == true) {
				force7 = my_system->GetBodyContactForce(6);
				force8 = my_system->GetBodyContactForce(7);
				force9 = my_system->GetBodyContactForce(8);
				force10 = my_system->GetBodyContactForce(9);
				force11 = my_system->GetBodyContactForce(10);
				force12 = my_system->GetBodyContactForce(11);
				force13 = my_system->GetBodyContactForce(12);
				force14 = my_system->GetBodyContactForce(13);
			}

			forceStream << my_system->GetChTime() << "\t";
			forceStream << Lx << "\t" << Ly << "\t";
			if (cylinder == true) {
				forceStream << L7 + L8 << "\t" << L9 + L10 << "\t" << L11 + L12 << "\t" << L13 + L14 << "\t";
			}
			forceStream << Lz << "\n";
			forceStream << "\t" << force4.x << "\t" << force6.y << "\t";
			if (cylinder == true) {
				forceStream << sqrt(force8.x*force8.x + force8.y*force8.y) << "\t";
				forceStream << sqrt(force10.x*force10.x + force10.y*force10.y) << "\t";
				forceStream << sqrt(force12.x*force12.x + force12.y*force12.y) << "\t";
				forceStream << sqrt(force14.x*force14.x + force14.y*force14.y) << "\t";
			}
			forceStream << force2.z << "\n";
			forceStream << "\t" << force4.x + force3.x << "\t" << force6.y + force5.y << "\t";
			if (cylinder == true) {
				forceStream << sqrt(force8.x*force8.x + force8.y*force8.y) - sqrt(force7.x*force7.x + force7.y*force7.y) << "\t";
				forceStream << sqrt(force10.x*force10.x + force10.y*force10.y) - sqrt(force9.x*force9.x + force9.y*force9.y) << "\t";
				forceStream << sqrt(force12.x*force12.x + force12.y*force12.y) - sqrt(force11.x*force11.x + force11.y*force11.y) << "\t";
				forceStream << sqrt(force14.x*force14.x + force14.y*force14.y) - sqrt(force13.x*force13.x + force13.y*force13.y) << "\t";
			}
			forceStream << force2.z + force1.z << "\n";

			//cout << my_system->GetChTime() << "\t";
			//cout << Lx << "\t" << Ly << "\t";
			//if (cylinder == true) {
			//	cout << L7 + L8 << "\t" << L9 + L10 << "\t" << L11 + L12 << "\t" << L13 + L14 << "\t";
			//}
			//cout << Lz << "\n";
			//cout << "\t" << force4.x << "\t" << force6.y << "\t";
			//if (cylinder == true) {
			//	cout << sqrt(force8.x*force8.x + force8.y*force8.y) << "\t";
			//	cout << sqrt(force10.x*force10.x + force10.y*force10.y) << "\t";
			//	cout << sqrt(force12.x*force12.x + force12.y*force12.y) << "\t";
			//	cout << sqrt(force14.x*force14.x + force14.y*force14.y) << "\t";
			//}
			//cout << force2.z << "\n";
			//cout << "\t" << force4.x + force3.x << "\t" << force6.y + force5.y << "\t";
			//if (cylinder == true) {
			//	cout << sqrt(force8.x*force8.x + force8.y*force8.y) - sqrt(force7.x*force7.x + force7.y*force7.y) << "\t";
			//	cout << sqrt(force10.x*force10.x + force10.y*force10.y) - sqrt(force9.x*force9.x + force9.y*force9.y) << "\t";
			//	cout << sqrt(force12.x*force12.x + force12.y*force12.y) - sqrt(force11.x*force11.x + force11.y*force11.y) << "\t";
			//	cout << sqrt(force14.x*force14.x + force14.y*force14.y) - sqrt(force13.x*force13.x + force13.y*force13.y) << "\t";
			//}
			//cout << force2.z + force1.z << "\n";

			stressStream << my_system->GetChTime() << "\t";
			stressStream << (Lz0 - Lz) / Lz0 << "\t";
			if (cylinder == true) {
				stressStream << ((Lx0 - Lx) / Lx0 + (Ly0 - Ly) / Ly0) / 2.0 << "\t";
				stressStream << force2.z / (CH_C_PI*diam*diam / 4.0) << "\t";
				stressStream << (force4.x - force3.x + force6.y - force5.y) / (Lz*CH_C_PI*diam / 3.0) << "\n";
			}
			else {
				stressStream << (Lx0 - Lx) / Lx0 << "\t" << (Ly0 - Ly) / Ly0 << "\t";
				stressStream << force2.z / (Lx*Ly) << "\t";
				stressStream << (force4.x - force3.x) / (2.0*Ly*Lz) << "\t" << (force6.y - force5.y) / (2.0*Lx*Lz) << "\n";
			}





			// ========================================================

			//std::vector<double> history = ((ChLcpIterativeSolver*)(my_system->GetLcpSolverSpeed()))->GetViolationHistory();
			//int numIters = history.size();
			//double residual = 0;
			//if (numIters) residual = history[numIters - 1];
			//residualsStream << my_system->GetChTime() << "\t" << residual << "\t" << numIters << "\t";

			// ========================================================

			//my_system->data_manager->host_data.hf;

/*			int my_size1 = my_system->data_manager->host_data.gamma.size();
			std::vector<double> my_vector(my_size1, 0);
			double norm_v1 = 0;
			double max_v1 = 0;
			for (int ii = 0; ii < my_size1; ii++){
				my_vector[ii] = my_system->data_manager->host_data.gamma[ii];
				norm_v1 += my_vector[ii] * my_vector[ii];
				if (max_v1 < abs(my_vector[ii]))
					max_v1 = abs(my_vector[ii]);
			}
			norm_v1 = sqrt(norm_v1);

			int my_size2 = my_size1 / 3.;
			std::vector<double> my_vector2(my_size2, 0);
			int jj = 0;
			double norm_v2 = 0;
			double max_v2 = 0;
			for (int ii = 0; ii < my_size2; ii++){
				my_vector2[ii] = my_system->data_manager->host_data.gamma[jj];
				norm_v2 += my_vector2[ii] * my_vector2[ii];
				if (max_v2 < abs(my_vector2[ii]))
					max_v2 = abs(my_vector2[ii]);
				jj += 3;
			}
			norm_v2 = sqrt(norm_v2);
*/
			double my_residual = my_system->data_manager->measures.solver.residual;

			residualsStream << my_residual << "\n"; 
			// << "\t" << my_size1 << "\t" << norm_v1 << "\t" << max_v1 << "\t" <<
			//	my_size2 << "\t" << norm_v2 << "\t" << max_v2 << "\n";

			// ========================================================
			// ========================================================
			
			// Max Velocity
			// Print_MaxVel_MaxAcc(my_system, time);

			//Print_MaxVel_MaxAcc(msystem, time);
			max_vel = 0;
			max_acc = 0;
			total_kinen = 0;
			var_kinen = 0;

			for (size_t i = 0; i < my_system->Get_bodylist()->size(); ++i) {
				auto body = (*my_system->Get_bodylist())[i];
				if (body->GetIdentifier() > 0) {

					double vel2 = body->GetPos_dt().Length2();
					double acc2 = body->GetPos_dtdt().Length2();
					if (vel2 > max_vel) max_vel = vel2;
					if (acc2 > max_acc) max_acc = acc2;

					double kinen_i = 0.5 * body->GetMass() * vel2;

					total_kinen += kinen_i;
					var_kinen += kinen_i * kinen_i;
				}
			}

			total_kinen = total_kinen;
			avg_kinen = total_kinen / num_of_spheres;
			var_kinen = var_kinen / num_of_spheres - avg_kinen * avg_kinen;


			maxvelStream << time << "\t" << max_vel << "\t" << max_acc << "\t" << fabs(wall_2->GetPos_dt().z) << "\t" << fabs(wall_2->GetPos_dtdt().z) << "\t" <<
				avg_kinen << "\t" << var_kinen << "\n"; 


			data_out_frame++;
		}

		//  Output to POV-Ray

		if (write_povray_data && my_system->GetChTime() >= visual_out_frame * visual_out_step) {
			char filename[100];
			sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), visual_out_frame + 1);
			utils::WriteShapesPovray(my_system, filename, false);

			visual_out_frame++;
		}

		if (write_checkpts_data && my_system->GetChTime() >= checkpt_out_frame * checkpt_out_step) {

			char filename[100];
			sprintf(filename, "%s/data_%03d.dat", checkpt_dir.c_str(),  checkpt_out_frame + 1);
			utils::WriteCheckpoint(my_system, filename);

			checkpt_out_frame++;
		}

		// Check for early finish

		double settling_tolerance = 1e-5;
		double prestress_tolerance_vel = 1e-5;
		double prestress_tolerance_acc = 1e-5;
		
		if (settled == false) {
			// Check if velocity of top wall is positive
			if (wall_2->GetPos_dt().z > settling_tolerance) {
				cout << "** settling finished at " << my_system->GetChTime() << "\n";
				settling_time = 0;
			}
		}
		//else {
		//	// Check if velocity and acceleration of top wall are very very small
		//	//if (fabs(wall_2->GetPos_dt().z) < prestress_tolerance_vel && fabs(wall_2->GetPos_dtdt().z) < prestress_tolerance_acc) {
		//	if (fabs(wall_2->GetPos_dt().z) < prestress_tolerance_vel && fabs(wall_2->GetPos_dtdt().z) < prestress_tolerance_acc &&
		//		max_vel < prestress_tolerance_vel && max_acc < prestress_tolerance_acc) {
		//		cout << "** simulation finished at " << my_system->GetChTime() << "\n";
		//		simulation_time = 0;
		//	}
		//}
	}

	//if (dense == true)
	//	for (i = 0; i < numParticleTypes; i++) mat[i]->SetFriction(mu[i]);

	// Create a checkpoint file for the prepared granular material specimen

	cout << "Write checkpoint data to " << flush;
	utils::WriteCheckpoint(my_system, specimen_ckpnt_file);
	cout << my_system->Get_bodylist()->size() << " bodies" << endl;

	return 0;

}