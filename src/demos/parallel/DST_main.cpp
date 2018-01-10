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
// Shear Test with PIV
// Shear Test stage
//
// =============================================================================

#include <stdio.h>
#include <vector>
#include <cmath>
#include <valarray>



#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono/core/ChFileutils.h"
#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"


using namespace chrono;
using namespace chrono::collision;

// -----------------------------------------------------
// uncomment lines below if you
// want to use DEMP and/or disable OpenGL
// -----------------------------------------------------
// #define USE_DEMP
#undef CHRONO_OPENGL

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;
using namespace chrono::collision;


uint max_iteration = 100;

//
// Create directories
//
#ifdef USE_DEMP
std::string out_folder = "DST/DEMP/TILT_ANGLE_30/vel05";
#else
std::string out_folder = "DST/DEMC/TILT_ANGLE_30/vel05";
#endif

std::string KinEn_file = out_folder + "/KinEnFile_main.dat";
std::string pos_file = out_folder + "/posFile.dat";
std::string checkpoint_file = out_folder + "/CHECKPOINT_tilt_angle_30_1.dat";

std::string chckpoint_folder = out_folder + "/CHECKPOINT";
std::string povray_folder = out_folder + "/POVRAY";

std::string IDs_file = povray_folder + "/001_BallsIDs.dat";


// Tilt angle (measured from the horizontal surface)
int tilt_angle_d = 30; //18; // 24; // 30;
float tilt_angle = tilt_angle_d * CH_C_PI / 180.;

// skaluj = scale the shearing velocity
// and simulation time
double skaluj = 1.;

int shear_vel_int = 5;
double shear_vel = shear_vel_int * 1.e-4 / 60. * 500. / skaluj;

bool ifdense = false;
bool if_povray = false;

// plate, everything lays on
float plate_length = 200.e-3;
float plate_width = 200.e-3;
float thickness = 10.e-3;



// -----------------------------------------------------------------------------
// Generate postprocessing output with current system state.
// -----------------------------------------------------------------------------
void PovrayOutputData(ChSystemParallel* sys, int out_frame, double time) {
	char filename[100];
	sprintf(filename, "%s/POVRAY/data_%03d.dat", out_folder.c_str(), out_frame);
	utils::WriteShapesPovray(sys, filename);
	std::cout << "time = " << time << std::flush << std::endl;
}

// -----------------------------------------------------------------------------
// Generate postprocessing output with current system state.
// -----------------------------------------------------------------------------
void CheckpointOutputData(ChSystemParallel* sys, int check_num) {
	char filename[100];
	sprintf(filename, "%s/CHECKPOINT/data_%03d.dat", out_folder.c_str(),check_num);
	utils::WriteCheckpoint(sys, filename);
	std::cout << "Checkpoint" << std::endl;
}


double CalculateKineticEnergy(ChSystemParallel *sys, int beginning, int ending) {
	double KineticEnergy = 0;

	int numParticles = ending - beginning;
	for (int i = 0; i < numParticles; i++) {
		std::shared_ptr<ChBody> body = sys->Get_bodylist()->at(beginning + i);
		ChVector<> RotationalEnergy = 0.5 * body->GetInertiaXX() * body->GetWvel_loc() * body->GetWvel_loc();
		KineticEnergy += 0.5 * body->GetMass() * body->GetPos_dt().Length2() + RotationalEnergy.x + RotationalEnergy.y + RotationalEnergy.z;
	}
	return KineticEnergy;
}

// -----------------------------------------------------------------------------
// Find bead that is the closest to the searched one
// -----------------------------------------------------------------------------
int FindSphere(ChSystemParallel *sys, int beginning, int ending, ChVector<> pos_star, int* zapamietaj, int miejsce) {

	int closest = beginning;
	double distance = 1.e3;

	int numParticles = ending - beginning;
	for (int i = 1; i < numParticles; i++) {
		std::shared_ptr<ChBody> body = sys->Get_bodylist()->at(beginning + i);
		if (body->GetIdentifier() > 0){
			double xx = body->GetPos().x;
			double yy = body->GetPos().y;
			double zz = body->GetPos().z;

			double running_distance = sqrt((xx - pos_star.x)*(xx - pos_star.x)
				+ (yy - pos_star.y)*(yy - pos_star.y)
				+ (zz - pos_star.z)*(zz - pos_star.z));

			if (running_distance < distance){

				int czy_juz_bylo = 0;
				for (int jj = 0; jj < miejsce; jj++)
					if (body->GetIdentifier() == zapamietaj[jj])
						czy_juz_bylo = 1;

				if (czy_juz_bylo == 0){
					distance = running_distance;
					closest = beginning + i;
				}
			}
		}
	}

	std::shared_ptr<ChBody> body = sys->Get_bodylist()->at(closest);
	double xx = body->GetPos().x;
	double yy = body->GetPos().y;
	double zz = body->GetPos().z;

	double closest_distance = sqrt((xx - pos_star.x)*(xx - pos_star.x)
		+ (yy - pos_star.y)*(yy - pos_star.y)
		+ (zz - pos_star.z)*(zz - pos_star.z));

	std::cout << "Found sphere's ID: " << body->GetIdentifier() << "\n";
	std::cout << "x: " << pos_star.x << " " << xx << "\n";
	std::cout << "y: " << pos_star.y << " " << yy << "\n";
	std::cout << "z: " << pos_star.z << " " << zz << "\n";
	std::cout << "Distance: " << closest_distance << "\n" ;

	zapamietaj[miejsce] = body->GetIdentifier();

	std::cout << "Hello from function " << zapamietaj[miejsce] << "\n";

	return closest;
}



std::shared_ptr<ChBody> GetBodyForMe(ChSystemParallel *sys, int ii, int list_id){
	std::shared_ptr<ChBody> ball_x = sys->Get_bodylist()->at(list_id);
	std::cout << "Ball " <<  ii << " ID is : " << ball_x->GetIdentifier() << "\n";
	return ball_x;
}

// shift vector a little bit to make it matching experimental data
ChVector<> modifyVector(ChVector<> pos){
	pos = pos * 1e-3;
	return pos;
}


void InitialPositionLibrary(ChVector<> *pos_1, ChVector<> *pos_2, ChVector<> *pos_3,
		ChVector<> *pos_4, ChVector<> *pos_5, ChVector<> *pos_6 ){

	// przypadek loose case
	if (ifdense == false) {

		if ((tilt_angle_d == 18) && (shear_vel_int == 5)){
			*(pos_1) = ChVector<>(2.72, 7.17, 0.);
			*(pos_2) = ChVector<>(0.53, 4.99, 0.);
			*(pos_3) = ChVector<>(2.67, 3.01, 0.);

			*(pos_4) = ChVector<>(0.26, -2.81, 0.);
			*(pos_5) = ChVector<>(2.96, -4.11, 0.);
			*(pos_6) = ChVector<>(0.82, -6.38, 0.);
		}

		if ((tilt_angle_d == 18) && (shear_vel_int == 10)){
			*(pos_1) = ChVector<>(-1.08, 9.68, 0.);
			*(pos_2) = ChVector<>(-1.56, 6.49, 0.);
			*(pos_3) = ChVector<>(-1.49, 3.52, 0.);

			*(pos_4) = ChVector<>(-1.10, -2.86, 0.);
			*(pos_5) = ChVector<>(-0.41, -5.70, 0.);
			*(pos_6) = ChVector<>(-2.59, -7.82, 0.);
		}

		if ((tilt_angle_d == 24) && (shear_vel_int == 5)){
			*(pos_1) = ChVector<>(-1.13, 8.37, 0.);
			*(pos_2) = ChVector<>(-0.66, 5.45, 0.);
			*(pos_3) = ChVector<>(0.77, 3.05, 0.);

			*(pos_4) = ChVector<>(1.40, -3.29, 0.);
			*(pos_5) = ChVector<>(-0.67, -5.50, 0.);
			*(pos_6) = ChVector<>(1.33, -7.58, 0.);
		}

		if ((tilt_angle_d == 24) && (shear_vel_int == 10)){
			*(pos_1) = ChVector<>(2.67, 8.47, 0.);
			*(pos_2) = ChVector<>(1.70, 5.73, 0.);
			*(pos_3) = ChVector<>(1.33, 2.85, 0.);

			*(pos_4) = ChVector<>(1.77, -2.65, 0.);
			*(pos_5) = ChVector<>(-0.03, -5.00, 0.);
			*(pos_6) = ChVector<>(1.75, -7.35, 0.);
		}

		if ((tilt_angle_d == 30) && (shear_vel_int == 5)){
			*(pos_1) = ChVector<>(-6.87, 7.08, 0.);
			*(pos_2) = ChVector<>(-5.61, 4.42, 0.);
			*(pos_3) = ChVector<>(-2.95, 3.22, 0.);

			*(pos_4) = ChVector<>(0.24, -1.62, 0.);
			*(pos_5) = ChVector<>(0.56, -4.48, 0.);
			*(pos_6) = ChVector<>(0.23, -7.45, 0.);
		}

		if ((tilt_angle_d == 30) && (shear_vel_int == 10)){
			*(pos_1) = ChVector<>(-4.19, 7.60, 0.);
			*(pos_2) = ChVector<>(-2.17, 5.47, 0.);
			*(pos_3) = ChVector<>(-0.10, 3.31, 0.);

			*(pos_4) = ChVector<>(-1.94, -3.15, 0.);
			*(pos_5) = ChVector<>(-2.24, -5.97, 0.);
			*(pos_6) = ChVector<>(-1.75, -8.86, 0.);
		}
	}

	if (ifdense == true) {
		if ((tilt_angle_d == 18) && (shear_vel_int == 5)){
			*(pos_1) = ChVector<>(-3.63, 8.13, 0.);
			*(pos_2) = ChVector<>(-2.13, 5.56, 0.);
			*(pos_3) = ChVector<>(-0.68, 3.04, 0.);

			*(pos_4) = ChVector<>(-0.57, -2.11, 0.);
			*(pos_5) = ChVector<>(-1.94, -4.71, 0.);
			*(pos_6) = ChVector<>(-3.40, -7.40, 0.);
		}

		if ((tilt_angle_d == 18) && (shear_vel_int == 10)){
			*(pos_1) = ChVector<>(-3.97, 9.34, 0.);
			*(pos_2) = ChVector<>(-5.41, 6.77, 0.);
			*(pos_3) = ChVector<>(-3.79, 4.32, 0.);

			*(pos_4) = ChVector<>(-5.04, -3.36, 0.);
			*(pos_5) = ChVector<>(-3.46, -5.90, 0.);
			*(pos_6) = ChVector<>(-4.92, -8.58, 0.);
		}

		if ((tilt_angle_d == 24) && (shear_vel_int == 5)){
			*(pos_1) = ChVector<>(-2.81, 7.86, 0.);
			*(pos_2) = ChVector<>(-1.24, 5.33, 0.);
			*(pos_3) = ChVector<>(0.27, 2.68, 0.);

			*(pos_4) = ChVector<>(0.33, -2.34, 0.);
			*(pos_5) = ChVector<>(-1.08, -4.91, 0.);
			*(pos_6) = ChVector<>(-2.50, -7.57, 0.);
		}

		if ((tilt_angle_d == 24) && (shear_vel_int == 10)){
			*(pos_1) = ChVector<>(-2.45, 7.59, 0.);
			*(pos_2) = ChVector<>(-0.89, 5.07, 0.);
			*(pos_3) = ChVector<>(0.69, 2.55, 0.);

			*(pos_4) = ChVector<>(0.81, -2.52, 0.);
			*(pos_5) = ChVector<>(-0.56, -5.06, 0.);
			*(pos_6) = ChVector<>(-2.02, -7.63, 0.);
		}


		if ((tilt_angle_d == 30) && (shear_vel_int == 5)){
			*(pos_1) = ChVector<>(-5.42, 7.77, 0.);
			*(pos_2) = ChVector<>(-3.86, 5.14, 0.);
			*(pos_3) = ChVector<>(-2.32, 2.66, 0.);

			*(pos_4) = ChVector<>(-2.25, -2.32, 0.);
			*(pos_5) = ChVector<>(-3.55, -5.01, 0.);
			*(pos_6) = ChVector<>(-4.97, -7.59, 0.);
		}

		if ((tilt_angle_d == 30) && (shear_vel_int == 10)){
			*(pos_1) = ChVector<>(-6.74, 7.43, 0.);
			*(pos_2) = ChVector<>(-5.18, 5.04, 0.);
			*(pos_3) = ChVector<>(-3.62, 2.49, 0.);

			*(pos_4) = ChVector<>(-3.50, -2.62, 0.);
			*(pos_5) = ChVector<>(-4.91, -5.27, 0.);
			*(pos_6) = ChVector<>(-6.33, -7.94, 0.);
		}
	}
}

// -----------------------------------------------------------------------------
// 
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {

	//-------------------------------------------------
	// use input data from argv vector
	//---------------------------------------------------
	out_folder = argv[1]; //"DST/DEMP/TILT_ANGLE_30/vel05";
	KinEn_file = out_folder + "/KinEnFile_main.dat";
	pos_file = out_folder + "/posFile.dat";

	checkpoint_file = argv[2];

	chckpoint_folder = out_folder + "/CHECKPOINT";
	povray_folder = out_folder + "/POVRAY";

	IDs_file = povray_folder + "/001_BallsIDs.dat";

	if (ChFileutils::MakeDirectory(out_folder.c_str()) < 0) {
		std::cout << "Error creating directory " << out_folder << std::endl;
		return 1;
	}
	if (ChFileutils::MakeDirectory(chckpoint_folder.c_str()) < 0) {
		std::cout << "Error creating directory " << chckpoint_folder << std::endl;
		return 1;
	}
	if (ChFileutils::MakeDirectory(povray_folder.c_str()) < 0) {
		std::cout << "Error creating directory " << povray_folder << std::endl;
		return 1;
	}

	{
		const char* text = argv[3];
		tilt_angle_d = atoi(text); //18; // 24; // 30;
		tilt_angle = tilt_angle_d * CH_C_PI / 180.;
	}
	{
		const char* text = argv[4];
		shear_vel_int = atoi(text); // 5; 10;
		shear_vel = shear_vel_int * 1.e-4 / 60. * 500.;
	}

	{
		const char* text = argv[5];
		if (atoi(text) == 1)
			ifdense = true;
		else
			ifdense = false;
	}

	{
		const char* text = argv[6];
		max_iteration = atoi(text);
	}

	{
		const char* text = argv[7];
		skaluj = atof(text); // 1; 2; 4; 8;
		shear_vel = shear_vel / skaluj;
	}

	{
		const char* text = argv[8];
		int czy_pov = atoi(text); 
		if (czy_pov > 0)
			if_povray = true;
		else
			if_povray = false;

	}

	double time_step;
	{
		const char* text = argv[9];
		time_step = atof(text);
	}
	
	int threads = 1;
	{
		const char* text = argv[10];
		threads = atoi(text);
	}

	int howManyParticles = 0;
	{
		std::string howManyParticles_file = argv[11];
		ChStreamInAsciiFile howManyParticlesStream(howManyParticles_file.c_str());
		howManyParticlesStream >> howManyParticles;
	}

	ChStreamOutAsciiFile KinEnStream(KinEn_file.c_str());
	ChStreamOutAsciiFile PosStream(pos_file.c_str());
	
	// --------------------------------------------------
	// --------------------------------------------------


	// -----------------------------------------------------
    // Simulation parameters
    // ---------------------

    double gravity = 9.81;

	// sklauj = scale the shearing velocity
	// and length of time simulated (LOTS)
	double time_end = 0.;
	if (shear_vel_int == 5)
		time_end = 2.4 * skaluj;
	if (shear_vel_int == 10)
		time_end = 1.2 * skaluj; // OK. We are not going to use vel = 0.5 anymore. skaluj will be from 0001 to 1000

	double out_fps = 300;
	double out_step = time_end / out_fps;
	double time_out_step = 0.;

	int povray_out_frame = 0;


	int checkpoint_out_frame = 0;
	double checkpoint_out_step = 0;
	double checkpt_difference = time_end / 10.;
	bool if_checkpoints = true;


	double radius_bead = 2.84e-3 / 2.;

#ifdef USE_DEMP
	//double time_step = 1e-5;
#else
	//double time_step = 1e-4;
	double crs_value = 10.;// shear_vel;
#endif

real tolerance = 1e-5 / time_step;

	// Create system
#ifdef USE_DEMP
	ChSystemParallelDEM msystem;
#else
	ChSystemParallelDVI msystem;
#endif



	// Set number of threads.
	int max_threads = CHOMPfunctions::GetNumProcs();
	if (threads > max_threads)
		threads = max_threads;
	msystem.SetParallelThreadNumber(threads);
	CHOMPfunctions::SetNumThreads(threads);

	// Set gravitational acceleration
	msystem.Set_G_acc(ChVector<>(0, -gravity * sin(tilt_angle), -gravity * cos(tilt_angle)));

	// Set solver parameters
	msystem.GetSettings()->solver.max_iteration_bilateral = max_iteration;
	msystem.GetSettings()->solver.tolerance = tolerance;

	msystem.GetSettings()->collision.narrowphase_algorithm = NARROWPHASE_HYBRID_MPR;
	msystem.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

#ifdef USE_DEMP
	// The following two lines are optional, since they are the default options. They are added for future reference,
	// i.e. when needed to change those models.
	msystem.GetSettings()->solver.contact_force_model = ChSystemDEM::ContactForceModel::Hertz;
	msystem.GetSettings()->solver.adhesion_force_model = ChSystemDEM::AdhesionForceModel::Constant;
#else
	// Set solver parameters
	msystem.GetSettings()->solver.solver_mode = SLIDING;
	msystem.GetSettings()->solver.max_iteration_normal = 0;
	msystem.GetSettings()->solver.max_iteration_sliding = max_iteration;
	msystem.GetSettings()->solver.max_iteration_spinning = 0;
	msystem.GetSettings()->solver.max_iteration_bilateral = max_iteration;
	msystem.GetSettings()->solver.tolerance = tolerance;
	msystem.GetSettings()->solver.alpha = 0;
	msystem.GetSettings()->solver.contact_recovery_speed = crs_value;
	msystem.ChangeSolverType(APGD);
	msystem.GetSettings()->collision.narrowphase_algorithm = NARROWPHASE_HYBRID_MPR;

	msystem.GetSettings()->collision.collision_envelope = 0.01* radius_bead;
#endif


	// Create the granular material and the container from the checkpoint file.
	//std::cout << "Read checkpoint data from " << checkpoint_file;
	utils::ReadCheckpoint(&msystem, checkpoint_file);
	std::cout << "  done.  Read " << msystem.Get_bodylist()->size()
		<< " bodies." << std::endl;


	std::shared_ptr<ChBody> plate = msystem.Get_bodylist()->at(0);
	// TOP FRAME
	std::shared_ptr<ChBody> top_frame = msystem.Get_bodylist()->at(2);
	std::cout << top_frame->GetIdentifier() << "\n";


	// only if angle is lagrer than 20 add a top plate (called klin in here)
	if (tilt_angle_d > 20){
		std::shared_ptr<ChBody> klin = msystem.Get_bodylist()->at(howManyParticles + 2);

		std::cout << klin->GetIdentifier() << "\n";


		// make z-axes of klin and plate always parallel to each other
		auto constraint_parallel = std::make_shared<ChLinkLockParallel>();
		constraint_parallel->Initialize(klin, plate, ChCoordsys<>(plate->GetPos(), Q_from_AngAxis(CH_C_PI / 2, VECT_Z)));
		//Q_from_AngAxis(CH_C_PI / 2, VECT_Z) = z axis is parallel
		//Q_from_AngAxis(CH_C_PI / 2, VECT_X) = y axis is parallel
		//Q_from_AngAxis(CH_C_PI / 2, VECT_Y) = x axis is parallel

		// ---------------------------------------------------
		// I decided to remove point on plane constraint
		// -----------------------------------------------------
		//auto constraint_plane = std::make_shared<ChLinkLockPointPlane>();
		//constraint_plane->Initialize(klin, plate, ChCoordsys<>(klin->GetPos(), Q_from_AngAxis(CH_C_PI / 2, VECT_Z)));


		msystem.AddLink(constraint_parallel);
		//msystem.AddLink(constraint_plane);
	}


	// before adding granular material we added plate, bottom and top frames
	// that is why starting value (id value) is set to 2
	int starting_value = 2;

	// ------------------------------------------------------------------
	// grab positions of 6 monitored particles from the library 
	// ------------------------------------------------------------------
	ChVector<> pos_1;
	ChVector<> pos_2;
	ChVector<> pos_3;
	ChVector<> pos_4;
	ChVector<> pos_5;
	ChVector<> pos_6;
	InitialPositionLibrary(&(pos_1), &(pos_2), &(pos_3), &(pos_4), &(pos_5), &(pos_6));

	pos_1 = modifyVector(pos_1);
	pos_2 = modifyVector(pos_2);
	pos_3 = modifyVector(pos_3);
	pos_4 = modifyVector(pos_4);
	pos_5 = modifyVector(pos_5);
	pos_6 = modifyVector(pos_6);

	std::cout << pos_1.x << " " << pos_1.y << " " << pos_1.z << "\n";

	int tablica[6];

	// --------------------------------
	// find those six particles' ID
	// ----------------------------------
	int ID_6 = FindSphere(&msystem, starting_value, msystem.Get_bodylist()->size() - 1, pos_6, tablica, 0);
	int ID_5 = FindSphere(&msystem, starting_value, msystem.Get_bodylist()->size() - 1, pos_5, tablica, 1);
	int ID_4 = FindSphere(&msystem, starting_value, msystem.Get_bodylist()->size() - 1, pos_4, tablica, 2);
	int ID_3 = FindSphere(&msystem, starting_value, msystem.Get_bodylist()->size() - 1, pos_3, tablica, 3);
	int ID_2 = FindSphere(&msystem, starting_value, msystem.Get_bodylist()->size() - 1, pos_2, tablica, 4);
	int ID_1 = FindSphere(&msystem, starting_value, msystem.Get_bodylist()->size() - 1, pos_1, tablica, 5);

	for (int ii = 5; ii >= 0; ii--)
		std::cout << "Hello from main " << tablica[ii] << "\n";

	// -----------------------------------------------------
	// pring particle IDs. Useful when using POV-Ray
	// -----------------------------------------------------
	{
		ChStreamOutAsciiFile BallsIDsStream(IDs_file.c_str());
		BallsIDsStream << tablica[0] << "," << tablica[1] << "," << tablica[2] << "," << tablica[3] << "," <<
			tablica[4] << "," << tablica[5] << "\n";
	}

	// --------------------------------
	// find those six particles, based on their ID
	// ----------------------------------
	std::shared_ptr<ChBody> ball_1 = GetBodyForMe(&msystem, 1, ID_1);
	std::shared_ptr<ChBody> ball_2 = GetBodyForMe(&msystem, 2, ID_2);
	std::shared_ptr<ChBody> ball_3 = GetBodyForMe(&msystem, 3, ID_3);
	std::shared_ptr<ChBody> ball_4 = GetBodyForMe(&msystem, 4, ID_4);
	std::shared_ptr<ChBody> ball_5 = GetBodyForMe(&msystem, 5, ID_5);
	std::shared_ptr<ChBody> ball_6 = GetBodyForMe(&msystem, 6, ID_6);



// Perform the simulation
// ----------------------

#ifdef CHRONO_OPENGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "Shear Test | Michal Kwarta", &msystem);
	gl_window.SetCamera(ChVector<>(plate_width / 2., -plate_length / 2., 10*thickness), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), (5e-3F));
    gl_window.SetRenderMode(opengl::WIREFRAME);

    // Uncomment the following two lines for the OpenGL manager to automatically
    // run the simulation in an infinite loop.
    // gl_window.StartDrawLoop(time_step);
    // return 0;

     while (msystem.GetChTime() < time_end) {
		 if (gl_window.Active()) {
			 gl_window.DoStepDynamics(time_step);
			 gl_window.Render();
			 if (gl_window.Running()) {

					 // ==================================
					 // ========= WHILE ==================
					 // ==================================

				 // Print cumulative contact force on container bin.
				 //real3 frc = msystem.GetBodyContactForce(0;)
				 //std::cout << frc.x << "  " << frc.y << "  " << frc.z << std::endl;

				 // ---------------------------------------------
				 // move the top frame a little bit
				 // ---------------------------------------------
				 top_frame->SetPos(ChVector<>(top_frame->GetPos().x - shear_vel * time_step,
					 top_frame->GetPos().y,
					 top_frame->GetPos().z));



				 if (msystem.GetChTime() > time_out_step){

					 double KinEn = CalculateKineticEnergy(&msystem, starting_value, msystem.Get_bodylist()->size() - 1);
					 KinEnStream << msystem.GetChTime() << "\t" << KinEn << "\n";


					 PosStream << msystem.GetChTime() << "," <<
						 ball_1->GetPos().x << "," << ball_1->GetPos().y << "," << ball_1->GetPos().z << "," <<
						 ball_2->GetPos().x << "," << ball_2->GetPos().y << "," << ball_2->GetPos().z << "," <<
						 ball_3->GetPos().x << "," << ball_3->GetPos().y << "," << ball_3->GetPos().z << "," <<
						 ball_4->GetPos().x << "," << ball_4->GetPos().y << "," << ball_4->GetPos().z << "," <<
						 ball_5->GetPos().x << "," << ball_5->GetPos().y << "," << ball_5->GetPos().z << "," <<
						 ball_6->GetPos().x << "," << ball_6->GetPos().y << "," << ball_6->GetPos().z << "," <<
						 "\n";

					 if (if_povray) {
						 povray_out_frame++;
						 PovrayOutputData(&msystem, povray_out_frame, msystem.GetChTime());
					 }
					 time_out_step = time_out_step + out_step;
				 }

				
				 if (msystem.GetChTime() > checkpoint_out_step && if_checkpoints == true){
					 checkpoint_out_frame++;
					 CheckpointOutputData(&msystem, checkpoint_out_frame);
					 checkpoint_out_step += checkpt_difference;
				 }


					 // ==================================
					 // ========= End of WHILE ===========
					 // ==================================

			 }

		 }
		 else {
            break;
        }
    }
#else
    // Run simulation for specified time
    int num_steps = std::ceil(time_end / time_step);
    int out_steps = std::ceil((1 / time_step) / out_fps);
    int out_frame = 0;
    double time = 0;

	while (msystem.GetChTime() < time_end){
	
		
		// ==================================
		// ========= WHILE ==================
		// ==================================

		msystem.DoStepDynamics(time_step);

		// Print cumulative contact force on container bin.
		//real3 frc = msystem.GetBodyContactForce(0;)
		//std::cout << frc.x << "  " << frc.y << "  " << frc.z << std::endl;

		// ---------------------------------------------
		// move the top frame a little bit
		// ---------------------------------------------
		top_frame->SetPos(ChVector<>(top_frame->GetPos().x - shear_vel * time_step,
			top_frame->GetPos().y,
			top_frame->GetPos().z));



		if (msystem.GetChTime() > time_out_step){

			double KinEn = CalculateKineticEnergy(&msystem, starting_value, msystem.Get_bodylist()->size() - 1);
			KinEnStream << msystem.GetChTime() << "\t" << KinEn << "\n";


			PosStream << msystem.GetChTime() << "," <<
				ball_1->GetPos().x << "," << ball_1->GetPos().y << "," << ball_1->GetPos().z << "," <<
				ball_2->GetPos().x << "," << ball_2->GetPos().y << "," << ball_2->GetPos().z << "," <<
				ball_3->GetPos().x << "," << ball_3->GetPos().y << "," << ball_3->GetPos().z << "," <<
				ball_4->GetPos().x << "," << ball_4->GetPos().y << "," << ball_4->GetPos().z << "," <<
				ball_5->GetPos().x << "," << ball_5->GetPos().y << "," << ball_5->GetPos().z << "," <<
				ball_6->GetPos().x << "," << ball_6->GetPos().y << "," << ball_6->GetPos().z << "," <<
				"\n";

			if (if_povray) {
				povray_out_frame++;
				PovrayOutputData(&msystem, povray_out_frame, msystem.GetChTime());
			}
			time_out_step = time_out_step + out_step;
		}

		if (msystem.GetChTime() > checkpoint_out_step && if_checkpoints == true){
			checkpoint_out_frame++;
			CheckpointOutputData(&msystem, checkpoint_out_frame);
			checkpoint_out_step += checkpt_difference;
		}

		// ==================================
		// ========= End of WHILE ===========
		// ==================================

	}

    //for (int i = 0; i < num_steps; i++) {
    //    if (i % out_steps == 0) {
    //        OutputData(&msystem, out_frame, time);
    //        out_frame++;
    //    }
    //    msystem.DoStepDynamics(time_step);
    //    time += time_step;
    //}
#endif

    return 0;
}
