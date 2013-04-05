#include "unit_POSTPROCESS/ChMitsubaRender.h"
#include "physics/ChSystem.h"
#include "assets/ChSphereShape.h"
#include "physics/ChApidll.h"
#include "physics/ChSystem.h"
#include "lcp/ChLcpIterativeMINRES.h"
#include "core/ChRealtimeStep.h"
#include "lcp/ChLcpIterativeSOR.h"
#include "lcp/ChLcpIterativeJacobi.h"
#include "collision/ChCModelBullet.h"
#include "collision/ChCCollisionSystemBullet.h"
#include "physics/ChContactContainer.h"
#include "unit_GPU/ChSystemGPU.h"
#include "unit_GPU/collision/ChContactContainerGPU.h"
#include "unit_OPENGL/ChOpenGL.h"
using namespace chrono;
using namespace postprocess;
using namespace geometry;
using namespace std;

#define PI 3.1415

#define CHBODYSHAREDPTR ChSharedBodyGPUPtr
#define CHBODY ChBodyGPU
#define CHCOLLISIONSYS ChCollisionSystemGPU
#define CHLCPDESC ChLcpSystemDescriptorGPU
#define CHCONTACTCONT ChContactContainerGPUsimple
#define CHSOLVER ChLcpSolverGPU
#define CHMODEL ChCollisionModelGPU
#define CHSYS ChSystemGPU

ChVector<> lpos(0, 0, 0);
ChQuaternion<> quat(1, 0, 0, 0);

//all dimensions are in millimeters, milligrams
real plate_height = -10;
real plate_thickness = 1;
real plate_radius = 20;
real plate_friction = 1;

real particle_radius = .1;
real particle_mass = .5263;
real particle_friction = 1.0;
Vector particle_initial_vel = Vector(0, -5, 0); //initial velocity

real gravity = -9810;
real timestep = .0001;
int num_steps = 5000000;

int particle_grid_x = 10;
int particle_grid_z = 10;
int particles_every = 50; //add particles every n steps
int save_every = 100; //save data every n steps

string data_folder = "data/";

template<class T>
void RunTimeStep(T* mSys, const int frame) {
	cout << frame << endl;
	if (frame % particles_every == 0) {
		ChSharedBodyGPUPtr sphere;
		for (int i = 0; i < particle_grid_x; i++) {
			for (int j = 0; j < particle_grid_z; j++) {
				sphere = ChSharedBodyGPUPtr(new ChBodyGPU);

				ChVector<> position(i * particle_radius * 2 - particle_grid_x * .5 * particle_radius * 2, 0, j * particle_radius * 2 - particle_grid_z * .5 * particle_radius * 2);
				InitObject(sphere, particle_mass, position, quat, particle_friction, particle_friction, 0, true, false, -1, i);
				AddCollisionGeometry(sphere, SPHERE, ChVector<>(particle_radius, particle_radius, particle_radius), lpos, quat);

				sphere->SetPos_dt(particle_initial_vel);
				FinalizeObject(sphere, (ChSystemGPU *) mSys);

				ChSharedPtr<ChSphereShape> sphere_shape = ChSharedPtr<ChAsset>(new ChSphereShape);
				sphere_shape->SetColor(ChColor(1, 0, 0));
				sphere_shape->GetSphereGeometry().rad = particle_radius;
				sphere->GetAssets().push_back(sphere_shape);
			}
		}
	}
}
int main(int argc, char* argv[]) {
	omp_set_num_threads(1);

	//=========================================================================================================
	ChSystemGPU * system_gpu = new ChSystemGPU;
	ChLcpSystemDescriptorGPU *mdescriptor = new ChLcpSystemDescriptorGPU();
	ChContactContainerGPU *mcontactcontainer = new ChContactContainerGPU();
	//ChCollisionSystemBulletGPU *mcollisionengine = new ChCollisionSystemBulletGPU();
	ChCollisionSystemGPU *mcollisionengine = new ChCollisionSystemGPU();
	system_gpu->ChangeLcpSystemDescriptor(mdescriptor);
	system_gpu->ChangeContactContainer(mcontactcontainer);
	system_gpu->ChangeCollisionSystem(mcollisionengine);
	system_gpu->SetIntegrationType(ChSystem::INT_ANITESCU);

	//=========================================================================================================
	((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetMaxIteration(120);
	((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetTolerance(1e-5);
	((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetCompliance(0, 0, 0);
	((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetContactRecoverySpeed(.6);
	((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetSolverType(BLOCK_JACOBI);
	//ACCELERATED_PROJECTED_GRADIENT_DESCENT
	mcollisionengine->broadphase.setBinsPerAxis(R3(40, 15, 40));
	mcollisionengine->broadphase.setBodyPerBin(100, 50);

	//=========================================================================================================
	ChMitsubaRender output(system_gpu);
	output.SetIntegrator("photonmapper");
	output.SetIntegratorOption("integer", "maxDepth", "32");
	output.SetFilm("ldrfilm");
	output.SetFilmOption("integer", "height", "1200");
	output.SetFilmOption("integer", "width", "1920");
	output.camera_target = ChVector<>(0, -5, 0);
	output.camera_origin = ChVector<>(0, -5, -40);
	output.camera_up = ChVector<>(0, 1, 0);
	output.SetDataFolder("data");
	output.ExportScript("test.xml");
	//=========================================================================================================
	cout << "Mass, Radius, Friction_Sphere, Friction_Plate" << endl;
	if (argc == 6) {
		particle_mass = atof(argv[1]);
		particle_radius = atof(argv[2]);
		particle_friction = atof(argv[3]);
		plate_friction = atof(argv[4]);
		data_folder = argv[5];
	}

	cout << particle_mass << " " << particle_radius << " " << particle_friction << " " << plate_friction << data_folder << endl;
	//=========================================================================================================
	system_gpu->Set_G_acc(ChVector<>(0, gravity, 0));
	system_gpu->SetStep(timestep);

	//=========================================================================================================

	ChSharedBodyGPUPtr PLATE = ChSharedBodyGPUPtr(new ChBodyGPU);
	InitObject(PLATE, 1, ChVector<>(0, plate_height, 0), quat, plate_friction, plate_friction, 0, true, true, -1000, -20000);
	AddCollisionGeometry(PLATE, BOX, ChVector<>(plate_radius, plate_thickness, plate_radius), lpos, quat);
	FinalizeObject(PLATE, (ChSystemGPU *) system_gpu);

	real container_width = 7.0, container_thickness = .25, container_height = 7.0, wscale = 1;
	ChSharedBodyGPUPtr L = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr R = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr F = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr B = ChSharedBodyGPUPtr(new ChBodyGPU);

	InitObject(L, 100000, Vector(-container_width + container_thickness, plate_height, 0), quat, plate_friction, plate_friction, 0, true, true, -20, -20);
	InitObject(R, 100000, Vector(container_width - container_thickness, plate_height, 0), quat, plate_friction, plate_friction, 0, true, true, -20, -20);
	InitObject(F, 100000, Vector(0, plate_height, -container_width + container_thickness), quat, plate_friction, plate_friction, 0, true, true, -20, -20);
	InitObject(B, 100000, Vector(0, plate_height, container_width - container_thickness), quat, plate_friction, plate_friction, 0, true, true, -20, -20);

	AddCollisionGeometry(L, BOX, Vector(container_thickness, container_height, container_width), lpos, quat);
	AddCollisionGeometry(R, BOX, Vector(container_thickness, container_height, container_width), lpos, quat);
	AddCollisionGeometry(F, BOX, Vector(container_width * wscale, container_height, container_thickness), lpos, quat);
	AddCollisionGeometry(B, BOX, Vector(container_width * wscale, container_height, container_thickness), lpos, quat);

	FinalizeObject(L, (ChSystemGPU *) system_gpu);
	FinalizeObject(R, (ChSystemGPU *) system_gpu);
	FinalizeObject(F, (ChSystemGPU *) system_gpu);
	FinalizeObject(B, (ChSystemGPU *) system_gpu);

	//=========================================================================================================
	//Rendering specific stuff:
	ChOpenGLManager * window_manager = new ChOpenGLManager();
	ChOpenGL openGLView(window_manager, system_gpu, 800, 600, 0, 0, "Test_Solvers");
	//openGLView.render_camera->camera_pos = Vector(0, -5, -40);
	//openGLView.render_camera->look_at = Vector(0, -5, 0);
	openGLView.SetCustomCallback(RunTimeStep);
	openGLView.StartSpinning(window_manager);
	window_manager->CallGlutMainLoop();

	//=========================================================================================================

	int file = 0;
	for (int i = 0; i < num_steps; i++) {
		cout << "step " << i << endl;
		system_gpu->DoStepDynamics(timestep);
		RunTimeStep(system_gpu, i);
		if (i % save_every == 0) {
			stringstream ss;
			cout << "Frame: " << file << endl;
			ss << data_folder << file << ".xml";
			output.ExportData(ss.str());
			file++;
		}

	}
	return 0;
}
