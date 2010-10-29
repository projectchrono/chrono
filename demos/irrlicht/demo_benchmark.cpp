///////////////////////////////////////////////////
//
//   Demo code about  
//
//     - testing the simulation speed for 
//       profiling and optimizing purposes.
//       NOTE: this benchmark uses IRRLICHT to
//       make the 3d display. 
//    
//	 CHRONO 
//   ------
//   Multibody dinamics engine
//    
// ------------------------------------------------ 
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------ 
/////////////////////////////////////////////////// 
 
    

#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"
#include "geometry/ChCSphere.h"
#include "geometry/ChCBox.h"
#include "irrlicht_interface/ChBodySceneNode.h"
#include "irrlicht_interface/ChBodySceneNodeTools.h" 
#include "irrlicht_interface/ChIrrAppInterface.h"
#include "core/ChTimer.h"
#include "core/ChRealtimeStep.h"

#include "lcp/ChLcpIterativeJacobi.h"

#include <irrlicht.h>



// Use the namespace of Chrono

using namespace chrono;

// Use the main namespaces of Irrlicht
using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io; 
using namespace gui;

// defaults
static int STATIC_lcp_iters      = 20;
static int STATIC_lcp_iters_stab = 20; 
static double SPHERE_RAD = 0.35;  //0.9
static double SPHERE_MASS = 1.0;  
static int    SPHERE_NUM = 1000; //400
      
void create_some_falling_items(ChSystem& mphysicalSystem,  
							   ISceneManager* msceneManager,  
							   IVideoDriver* driver,
							   ISceneNode* mparent,
							   int numspheres,
							   double sphereradius,
							   double spheremass, 
							   double mwidth=20.0) 
{   
	ChBodySceneNode* mrigidBody; 
	 
	double flock_height = ((4./3.)*CH_C_PI*pow(sphereradius,3)*numspheres*(1.0/0.40))/(mwidth*mwidth);
	double flock_size   = mwidth-sphereradius;
	double mfriction = 0.4; 

	ChCollisionModel::SetDefaultSuggestedEnvelope(sphereradius*0.2);

	for (int bi = 0; bi < numspheres; bi++) 
	{ 
		// Create many ChronoENGINE rigid bodies which will fall..
		// Bodies are Irrlicht nodes of the special class ChBodySceneNode, 
		// which encapsulates ChBody items).

		mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easySphere(
											&mphysicalSystem, msceneManager,
											spheremass,
											ChVector<>( -flock_size*0.5+ChRandom()*flock_size, 
														2.+ flock_height*((double)bi/(double)numspheres), 
														-flock_size*0.5+ChRandom()*flock_size),
											sphereradius, 15,10,
											mparent);
/*
		mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											spheremass,
											ChVector<>( -flock_size*0.5+ChRandom()*flock_size, 
														4.+ flock_height*((double)bi/(double)numspheres), 
														-flock_size*0.5+ChRandom()*flock_size),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(sphereradius,sphereradius,sphereradius) , 
											mparent);
*/

				// set additional ChronoENGINE specific properties for the body...
		mrigidBody->GetBody()->SetFriction(mfriction);
		double inertia = (2./5.)*sphereradius*sphereradius * spheremass;
		mrigidBody->GetBody()->SetInertiaXX(ChVector<>(inertia,inertia,inertia));
		 
		video::ITexture* sphereMap = driver->getTexture("../data/pinkwhite.png");
		mrigidBody->setMaterialTexture(0,	sphereMap);
	}
  
	// Create the four walls 

	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											200.0,
											ChVector<>(0,-2,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(60,4,60) , 
											mparent);
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->GetBody()->SetFriction(mfriction);
	mrigidBody->GetBody()->SetInertiaXX(ChVector<>(200,200,200));

	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											200.0,
											ChVector<>(-0.5*mwidth-1.0,mwidth,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(2,80,mwidth) ,
											mparent );
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->GetBody()->SetFriction(mfriction);
	mrigidBody->GetBody()->SetInertiaXX(ChVector<>(200,200,200));
	
	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											200.0,
											ChVector<>(0.5*mwidth+1.0,mwidth,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(2,80,mwidth) ,
											mparent);
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->GetBody()->SetFriction(mfriction);
	mrigidBody->GetBody()->SetInertiaXX(ChVector<>(200,200,200));

	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											200.0,
											ChVector<>(0,mwidth,0.5*mwidth+1.0), 
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(mwidth,80,2) ,
											mparent); 
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->GetBody()->SetSfriction(mfriction);
	mrigidBody->GetBody()->SetKfriction(mfriction);
	mrigidBody->GetBody()->SetInertiaXX(ChVector<>(200,200,200));

	mrigidBody = (ChBodySceneNode*)addChBodySceneNode(
											&mphysicalSystem, msceneManager,
											0, 200.0,
											ChVector<>(0,mwidth, -0.5*mwidth-1.0),
											ChQuaternion<>(1,0,0,0), 
											mparent);
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->GetBody()->SetFriction(mfriction);
	mrigidBody->GetBody()->SetInertiaXX(ChVector<>(200,200,200));

	mrigidBody->GetBody()->SetCollide(true);
	mrigidBody->GetBody()->GetCollisionModel()->ClearModel();
		// Describe the (invisible) colliding shape by adding boxes
	mrigidBody->GetBody()->GetCollisionModel()->AddBox(mwidth,80,2); 
		// Complete the description.
	mrigidBody->GetBody()->GetCollisionModel()->BuildModel();



}

 



int main(int argc, char* argv[])
{

	// In CHRONO engine, The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed.
	DLL_CreateGlobals();

	// Create a ChronoENGINE physical system
	ChSystem mphysicalSystem;

	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrAppInterface application(&mphysicalSystem, L"Benchmark application",core::dimension2d<u32>(800,600),false);

 
	// Easy shortcuts to add logo, camera, lights and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Lights(application.GetDevice());
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0,14,-40));



 	// 
	// CREATE THE MEChANICAL SYSTEM OF CHRONO...
	// 

 
	application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));
	application.GetVideoDriver()->endScene(); 
	GetLog() << "  ** PLEASE WAIT FOR INITIALIZATION! \n  ** This may take seconds or minutes..\n\n";

    
	// Prepare the physical system for the simulation
 
	mphysicalSystem.SetIntegrationType(ChSystem::INT_ANITESCU); 
	//mphysicalSystem.SetIntegrationType(ChSystem::INT_TASORA); 

	//mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SYMMSOR); // here would be the better choiche since the wall is build ground up.
	//mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
	//mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_JACOBI); 
	//mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR_MULTITHREAD); 
	//mphysicalSystem.SetParallelThreadNumber(4);

	mphysicalSystem.SetMaxPenetrationRecoverySpeed(1.6); // used by Anitescu stepper only
	mphysicalSystem.SetIterLCPmaxItersSpeed(20);
	mphysicalSystem.SetIterLCPmaxItersStab(10); // unuseful for Anitescu, only Tasora uses this
	//mphysicalSystem.SetIterLCPwarmStarting(true);


	// Test convergence for varying omega 
	//
	// (solver:SOR)
	//
	if (false)
	{
		ChStreamOutAsciiFile data_err_SOR_Niter("data_err_SOR_Niter.dat");
		ChStreamOutAsciiFile data_dgamma_SOR_Niter("data_dgamma_SOR_Niter.dat");
		int plotframe = 500;
		int ntestspheres = 220;

		mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
		mphysicalSystem.SetIterLCPmaxItersSpeed(80);
		mphysicalSystem.SetMaxPenetrationRecoverySpeed(2.0); 

		ChLcpIterativeSolver* msolver = (ChLcpIterativeSolver*)mphysicalSystem.GetLcpSolverSpeed();
		msolver->SetRecordViolation(true);

		for (double momega = 0.2; (momega <= 1.401 && application.GetDevice()->run()); momega +=0.2)
		{
			ISceneNode* parent = application.GetSceneManager()->addEmptySceneNode();

			mphysicalSystem.SetIterLCPomega(0.8);
			mphysicalSystem.SetIterLCPsharpnessLambda(1.0);
			mphysicalSystem.SetMaxPenetrationRecoverySpeed(2.0);

				// create spheres and walls, and insert into that scene node
			ChSetRandomSeed(123);
			create_some_falling_items(mphysicalSystem, application.GetSceneManager(), application.GetVideoDriver(), parent, ntestspheres, 1.6, 10); 

			int totframes = 0;
										// ------- begin simulation loop -----
			while(application.GetDevice()->run())
			{
				totframes++;

				application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));
				application.DrawAll();

				if (totframes==plotframe)
				{
					mphysicalSystem.SetIterLCPomega(momega);
					mphysicalSystem.SetIterLCPsharpnessLambda(0.7);
					mphysicalSystem.SetMaxPenetrationRecoverySpeed(0.0);
				}

				// integrate
				mphysicalSystem.DoStepDynamics(0.01);

				application.GetVideoDriver()->endScene(); 

				//***PLOT***
				if (totframes==plotframe)
				{
					GetLog()<< "Saving data\n";
					ChLcpIterativeSolver* msolver = (ChLcpIterativeSolver*)mphysicalSystem.GetLcpSolverSpeed();
					for (int k=0; k< msolver->GetViolationHistory().size(); k++)
					{
						data_err_SOR_Niter <<  msolver->GetViolationHistory()[k] << " ";
					}
					for (int j=0; j< msolver->GetDeltalambdaHistory().size(); j++)
					{
						data_dgamma_SOR_Niter <<  msolver->GetDeltalambdaHistory()[j] << " ";
					}
				}


				

				if(totframes>plotframe) break;

			}							// ------- end of simulation loop -----

			GetLog()<< "  num contacts " << mphysicalSystem.GetNcontacts() << "\n";
			
			data_err_SOR_Niter << "\n";
			data_dgamma_SOR_Niter << "\n";

			// remove the root scene node (contains walls & spheres but no camera no lights)
			// to reset and prepare for new scene at next for() loop.
			parent->remove();

			// Now the system should already have no bodies, because of full removal from Irrlicht scene.
			// Then, the following clearing should be unuseful...(do it anyway..)
			mphysicalSystem.Clear(); 
		}

	} // end test



	// Test convergence for varying omega 
	//
	// (solver:JACOBI)
	//
	if (false)
	{
		ChStreamOutAsciiFile data_err_JACOBI_Niter("data_err_JACOBI_Niter_L10_at500_ok.dat");
		ChStreamOutAsciiFile data_dgamma_JACOBI_Niter("data_dgamma_JACOBI_Niter_L10_at500_ok.dat");
		int plotframe = 500;
		int ntestspheres = 220;

		mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
		mphysicalSystem.SetIterLCPomega(0.8);
		mphysicalSystem.SetIterLCPmaxItersSpeed(80);
		mphysicalSystem.SetMaxPenetrationRecoverySpeed(2.0); 

		ChLcpIterativeSolver* msolver = (ChLcpIterativeSolver*)mphysicalSystem.GetLcpSolverSpeed();
		msolver->SetRecordViolation(true);

		for (double momega = 0.1; (momega <= 0.71 && application.GetDevice()->run()); momega +=0.1)
		{
			ISceneNode* parent = application.GetSceneManager()->addEmptySceneNode();

			mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
			mphysicalSystem.SetIterLCPomega(0.8);
			mphysicalSystem.SetIterLCPmaxItersSpeed(80);
			mphysicalSystem.SetMaxPenetrationRecoverySpeed(2.0); 
			mphysicalSystem.SetIterLCPsharpnessLambda(1.0);

				// create spheres and walls, and insert into that scene node
			ChSetRandomSeed(123);
			create_some_falling_items(mphysicalSystem, application.GetSceneManager(), application.GetVideoDriver(), parent, ntestspheres, 1.6, 10); 

			int totframes = 0;
										// ------- begin simulation loop -----
			while(application.GetDevice()->run())
			{
				totframes++;

				application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));
				application.DrawAll();

				if (totframes==plotframe)
				{
					mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_JACOBI);
					ChLcpIterativeSolver* msolver = (ChLcpIterativeSolver*)mphysicalSystem.GetLcpSolverSpeed();
					msolver->SetRecordViolation(true);
					mphysicalSystem.SetIterLCPomega(momega);
					mphysicalSystem.SetIterLCPsharpnessLambda(1.0);
					mphysicalSystem.SetMaxPenetrationRecoverySpeed(0.0);
				}

				// integrate
				mphysicalSystem.DoStepDynamics(0.01);

				application.GetVideoDriver()->endScene(); 

				//***PLOT***
				if (totframes==plotframe)
				{
					GetLog()<< "Saving data\n";
					ChLcpIterativeSolver* msolver = (ChLcpIterativeSolver*)mphysicalSystem.GetLcpSolverSpeed();
					for (int k=0; k< msolver->GetViolationHistory().size(); k++)
					{
						data_err_JACOBI_Niter <<  msolver->GetViolationHistory()[k] << " ";
					}
					for (int j=0; j< msolver->GetDeltalambdaHistory().size(); j++)
					{
						data_dgamma_JACOBI_Niter <<  msolver->GetDeltalambdaHistory()[j] << " ";
					}
				}

				if(totframes>plotframe) break;

			}							// ------- end of simulation loop -----

			GetLog()<< "  num contacts " << mphysicalSystem.GetNcontacts() << "\n";
			
			data_err_JACOBI_Niter << "\n";
			data_dgamma_JACOBI_Niter << "\n";

			// remove the root scene node (contains walls & spheres but no camera no lights)
			// to reset and prepare for new scene at next for() loop.
			parent->remove();

			// Now the system should already have no bodies, because of full removal from Irrlicht scene.
			// Then, the following clearing should be unuseful...(do it anyway..)
			mphysicalSystem.Clear(); 
		}

	} // end test



	// Test lambda effect
	//
	// Velocity error  vs.  Num. velocity iterations  vs. Lambda
	// (solver:SOR)
	//
	if (false)
	{
		ChStreamOutAsciiFile data_err_SOR_Niter("data_err_SOR_Niter_om1.dat");
		ChStreamOutAsciiFile data_dgamma_SOR_Niter("data_dgamma_SOR_Niter_om1.dat");
		int plotframe = 500;
		int ntestspheres = 220;

		mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
		mphysicalSystem.SetIterLCPmaxItersSpeed(80);
		mphysicalSystem.SetMaxPenetrationRecoverySpeed(2.0); 

		ChLcpIterativeSolver* msolver = (ChLcpIterativeSolver*)mphysicalSystem.GetLcpSolverSpeed();
		msolver->SetRecordViolation(true);

		for (double mlambda = 0.2; (mlambda <= 1.01 && application.GetDevice()->run()); mlambda +=0.2)
		{
			ISceneNode* parent = application.GetSceneManager()->addEmptySceneNode();

			mphysicalSystem.SetIterLCPomega(0.8);
			mphysicalSystem.SetIterLCPsharpnessLambda(1.0);
			mphysicalSystem.SetMaxPenetrationRecoverySpeed(2.0);

				// create spheres and walls, and insert into that scene node
			ChSetRandomSeed(123);
			create_some_falling_items(mphysicalSystem, application.GetSceneManager(), application.GetVideoDriver(), parent, ntestspheres, 1.6, 10); 

			int totframes = 0;
										// ------- begin simulation loop -----
			while(application.GetDevice()->run())
			{
				totframes++;

				application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));
				application.DrawAll();

				if (totframes==plotframe)
				{
					mphysicalSystem.SetIterLCPomega(1.0);
					mphysicalSystem.SetIterLCPsharpnessLambda(mlambda);
					mphysicalSystem.SetMaxPenetrationRecoverySpeed(0.0);
				}

				// integrate
				mphysicalSystem.DoStepDynamics(0.01);

				application.GetVideoDriver()->endScene(); 

				//***PLOT***
				if (totframes==plotframe)
				{
					GetLog()<< "Saving data\n";
					ChLcpIterativeSolver* msolver = (ChLcpIterativeSolver*)mphysicalSystem.GetLcpSolverSpeed();
					for (int k=0; k< msolver->GetViolationHistory().size(); k++)
					{
						data_err_SOR_Niter <<  msolver->GetViolationHistory()[k] << " ";
					}
					for (int j=0; j< msolver->GetDeltalambdaHistory().size(); j++)
					{
						data_dgamma_SOR_Niter <<  msolver->GetDeltalambdaHistory()[j] << " ";
					}
				}


				

				if(totframes>plotframe) break;

			}							// ------- end of simulation loop -----

			GetLog()<< "  num contacts " << mphysicalSystem.GetNcontacts() << "\n";
			
			data_err_SOR_Niter << "\n";
			data_dgamma_SOR_Niter << "\n";

			// remove the root scene node (contains walls & spheres but no camera no lights)
			// to reset and prepare for new scene at next for() loop.
			parent->remove();

			// Now the system should already have no bodies, because of full removal from Irrlicht scene.
			// Then, the following clearing should be unuseful...(do it anyway..)
			mphysicalSystem.Clear(); 
		}

	} // end test



	// Test stack aspect ratio
	//
	//
	if (false)
	{
		ChStreamOutAsciiFile data_err_SOR_Niter("data_err_aspectratio3.dat");
		int plotframe = 500;
		int ntestspheres = 220;

		mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
		mphysicalSystem.SetIterLCPmaxItersSpeed(30);
		mphysicalSystem.SetMaxPenetrationRecoverySpeed(2.0); 

		ChLcpIterativeSolver* msolver = (ChLcpIterativeSolver*)mphysicalSystem.GetLcpSolverSpeed();
		msolver->SetRecordViolation(true);

		for (double mspheres=10; (mspheres <= 330 && application.GetDevice()->run()); mspheres +=40)
		{
			for (int nviters = 20; (nviters <= 101 && application.GetDevice()->run()); nviters +=20)
			{
				ISceneNode* parent = application.GetSceneManager()->addEmptySceneNode();

				mphysicalSystem.SetIterLCPomega(0.9);
				mphysicalSystem.SetIterLCPsharpnessLambda(1.0);
				mphysicalSystem.SetIterLCPmaxItersSpeed(30);
				mphysicalSystem.SetMaxPenetrationRecoverySpeed(2.0);
				
					// create spheres and walls, and insert into that scene node
				ChSetRandomSeed(13);
				create_some_falling_items(mphysicalSystem, application.GetSceneManager(), application.GetVideoDriver(), parent, mspheres, 1.6, 10,20.1); 

				int totframes = 0;
											// ------- begin simulation loop -----
				while(application.GetDevice()->run())
				{
					totframes++;

					application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));
					application.DrawAll();

					if (totframes==plotframe)
					{
						mphysicalSystem.SetIterLCPomega(0.9);
						mphysicalSystem.SetIterLCPmaxItersSpeed(nviters);
						mphysicalSystem.SetMaxPenetrationRecoverySpeed(0.0);
					}

					application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));
					application.DrawAll();

					// integrate
					mphysicalSystem.DoStepDynamics(0.01);

					application.GetVideoDriver()->endScene(); 

					//***PLOT***
					if (totframes==plotframe)
					{
						GetLog()<< "Saving data\n";
						ChLcpIterativeSolver* msolver = (ChLcpIterativeSolver*)mphysicalSystem.GetLcpSolverSpeed();
						data_err_SOR_Niter <<  msolver->GetViolationHistory()[msolver->GetViolationHistory().size()-1] << " ";
					}

					if(totframes>plotframe) break;

				}							// ------- end of simulation loop -----

				
				// remove the root scene node (contains walls & spheres but no camera no lights)
				// to reset and prepare for new scene at next for() loop.
				parent->remove();

				// Now the system should already have no bodies, because of full removal from Irrlicht scene.
				// Then, the following clearing should be unuseful...(do it anyway..)
				mphysicalSystem.Clear(); 
			}

			data_err_SOR_Niter << "\n";
		}

	} // end test




	// Test pos error
	//
	//
	if (false)
	{
		ChStreamOutAsciiFile data_err_pos("data_err_pos.dat");
		int plotframe = 500;
		int ntestspheres = 120;
		double mystep= 0;

		for (double mdt=0.01; (mdt>0.00015 && application.GetDevice()->run()); mdt*=0.5)
		{
			GetLog() << "\n\nDT = " << mdt << "\n";

			for (int nviters = 20; (nviters <= 81 && application.GetDevice()->run()); nviters +=20)
			{
				GetLog() << "  nviters = " << nviters << "\n";

				ISceneNode* parent = application.GetSceneManager()->addEmptySceneNode();

				mystep = 0.01;
				mphysicalSystem.SetChTime(0.0);
				mphysicalSystem.SetIterLCPsharpnessLambda(0.8);
				mphysicalSystem.SetIterLCPomega(0.8);
				mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
				mphysicalSystem.SetIterLCPmaxItersSpeed(30);
				mphysicalSystem.SetMaxPenetrationRecoverySpeed(2.0); 
				
					// create spheres and walls, and insert into that scene node
				ChSetRandomSeed(13);
				create_some_falling_items(mphysicalSystem, application.GetSceneManager(), application.GetVideoDriver(), parent, ntestspheres, 1.6, 10); 

				int totframes = 0;
				int averagecnt = 0;
				double average = 0;
											// ------- begin simulation loop -----
				while(application.GetDevice()->run())
				{
					totframes++;

					application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));
					application.DrawAll();

					// start mode
					if (mphysicalSystem.GetChTime()>2.4)
					{	
						mphysicalSystem.SetIterLCPmaxItersSpeed(nviters);
						mystep=mdt;
					}

					//start average
					if (mphysicalSystem.GetChTime()>2.6)
					{	

						class _label_reporter_class : public chrono::ChReportContactCallback 
						{
						public:
							virtual bool ReportContactCallback (
											const chrono::ChVector<>& pA,				
											const chrono::ChVector<>& pB,				
											const chrono::ChMatrix33<>& plane_coord,	
											const double& distance,				
											const float& mfriction,			  	
											const chrono::ChVector<>& react_forces,		
											const chrono::ChVector<>& react_torques,	
											chrono::collision::ChCollisionModel* modA,	
											chrono::collision::ChCollisionModel* modB) 
							{
								if (distance<maxpen)
									maxpen = distance;
								return true; // to continue scanning contacts
							}
							// Data 
							double maxpen;
						};

						_label_reporter_class my_label_rep;

						my_label_rep.maxpen = 0;

						// scan all contacts
						mphysicalSystem.GetContactContainer()->ReportAllContacts(&my_label_rep);
						//GetLog() << "        p: " << my_label_rep.maxpen << "\n";
						averagecnt++;
						average += my_label_rep.maxpen;
					}

					// stop average & loop
					if(mphysicalSystem.GetChTime()>2.8)
					{
						average /= (double)averagecnt;
						GetLog() << "     avg.err: " << average << "\n";
						data_err_pos <<  average << " ";
						break;
					}

					// integrate
					mphysicalSystem.DoStepDynamics(mystep);

					application.GetVideoDriver()->endScene(); 

				}							// ------- end of simulation loop -----

				
				// remove the root scene node (contains walls & spheres but no camera no lights)
				// to reset and prepare for new scene at next for() loop.
				parent->remove();

				// Now the system should already have no bodies, because of full removal from Irrlicht scene.
				// Then, the following clearing should be unuseful...(do it anyway..)
				mphysicalSystem.Clear(); 
			}

			data_err_pos << "\n";
		}

	} // end test



	// Test pos error with Tasora stabilization
	//
	//
	if (true)
	{
		ChStreamOutAsciiFile data_err_pos("data_err_tasora.dat");
		int ntestspheres = 120;

		for (int npiters = 20; (npiters <= 81 && application.GetDevice()->run()); npiters +=20)
		{
			GetLog() << "  npiters = " << npiters << "\n";

			for (int nviters = 20; (nviters <= 81 && application.GetDevice()->run()); nviters +=20)
			{
				GetLog() << "  nviters = " << nviters << "\n";

				ISceneNode* parent = application.GetSceneManager()->addEmptySceneNode();

				mphysicalSystem.SetChTime(0.0);
				mphysicalSystem.SetIterLCPsharpnessLambda(0.8);
				mphysicalSystem.SetIterLCPomega(0.8);
				mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
				mphysicalSystem.SetIntegrationType(ChSystem::INT_TASORA);
				mphysicalSystem.SetIterLCPmaxItersSpeed(30);
				mphysicalSystem.SetIterLCPmaxItersStab(30);
				mphysicalSystem.SetMaxPenetrationRecoverySpeed(0.0); 
				
					// create spheres and walls, and insert into that scene node
				ChSetRandomSeed(13);
				create_some_falling_items(mphysicalSystem, application.GetSceneManager(), application.GetVideoDriver(), parent, ntestspheres, 1.6, 10); 

				int totframes = 0;
				int averagecnt = 0;
				double average = 0;
											// ------- begin simulation loop -----
				while(application.GetDevice()->run())
				{
					totframes++;

					application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));
					application.DrawAll();

					// start mode
					if (mphysicalSystem.GetChTime()>3)
					{	
						mphysicalSystem.SetIterLCPmaxItersSpeed(nviters);
						mphysicalSystem.SetIterLCPmaxItersStab(npiters);
					}

					//start average
					if (mphysicalSystem.GetChTime()>4.2)
					{	

						class _label_reporter_class : public chrono::ChReportContactCallback 
						{
						public:
							virtual bool ReportContactCallback (
											const chrono::ChVector<>& pA,				
											const chrono::ChVector<>& pB,				
											const chrono::ChMatrix33<>& plane_coord,	
											const double& distance,				
											const float& mfriction,			  	
											const chrono::ChVector<>& react_forces,		
											const chrono::ChVector<>& react_torques,	
											chrono::collision::ChCollisionModel* modA,	
											chrono::collision::ChCollisionModel* modB) 
							{
								if (distance<maxpen)
									maxpen = distance;
								return true; // to continue scanning contacts
							}
							// Data 
							double maxpen;
						};

						_label_reporter_class my_label_rep;

						my_label_rep.maxpen = 0;

						// scan all contacts
						mphysicalSystem.GetContactContainer()->ReportAllContacts(&my_label_rep);
						averagecnt++;
						average += my_label_rep.maxpen;
					}

					// stop average & loop
					if(mphysicalSystem.GetChTime()>4.5)
					{
						average /= (double)averagecnt;
						GetLog() << "     avg.err: " << average << "\n";
						data_err_pos <<  average << " ";
						break;
					}

					// integrate
					mphysicalSystem.DoStepDynamics(0.01);

					application.GetVideoDriver()->endScene(); 

				}							// ------- end of simulation loop -----

				
				// remove the root scene node (contains walls & spheres but no camera no lights)
				// to reset and prepare for new scene at next for() loop.
				parent->remove();

				// Now the system should already have no bodies, because of full removal from Irrlicht scene.
				// Then, the following clearing should be unuseful...(do it anyway..)
				mphysicalSystem.Clear(); 
			}

			data_err_pos << "\n";
		}

	} // end test



	getchar();



	//
	// loop of tests:each time test more spheres!
	// 
	ChStreamOutAsciiFile data_spheres_times("data_spheres_times.dat");
	ChStreamOutAsciiFile data_contacts_times("data_contact_times.dat");
	ChStreamOutAsciiFile data_ncontacts("data_ncontacts.dat");
	ChStreamOutAsciiFile data_timexconstr("data_timexconstr.dat");

	for (int nspheres = 1; (nspheres <= 1 && application.GetDevice()->run()); nspheres +=1)
	{

		ChStreamOutAsciiFile data_timing("data_timing1k.dat");
		

		// 
		// Create all the  rigid bodies!!!! 
		//

			// the root scene node containing all dynamic objects
		ISceneNode* parent = application.GetSceneManager()->addEmptySceneNode();

			// create spheres and walls, and insert into that scene node
		create_some_falling_items(mphysicalSystem, application.GetSceneManager(), application.GetVideoDriver(), parent, SPHERE_NUM,SPHERE_RAD,SPHERE_MASS); 

		// 
		// THE SOFT-REAL-TIME CYCLE
		// 

		int mdelay=0;
		int lastFPS = -1;

		ChTimer<double>mtimer; 

		ChRealtimeStepTimer m_realtime_timer;

		//ChStreamOutAsciiFile data_deltares("data_pNviolation_i80.dat");


		int totframes = 0;
		while(application.GetDevice()->run())
		{
			totframes++;

			if (totframes == 400) mtimer.start();

			// Irrlicht must prepare frame to draw
			application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));
		
			// Irrlicht draws all 3D objects and all GUI items
			application.DrawAll();


			// HERE CHRONO INTEGRATION IS PERFORMED: THE  
			// TIME OF THE SIMULATION ADVANCES FOR A SINGLE
			// STEP:
			mphysicalSystem.DoStepDynamics(0.01);


			application.GetVideoDriver()->endScene(); 

			

			if (totframes > 600)
			data_timing << mphysicalSystem.GetChTime()-3.0 << " " \
						<< mphysicalSystem.GetTimerStep() << " " \
						<< mphysicalSystem.GetTimerLcp()+mphysicalSystem.GetTimerCollisionBroad() << " " \
						<< mphysicalSystem.GetTimerLcp() << "\n";


			/*
			//***PLOT*** plot max constr.normal violation
			if ((totframes<600)&&(totframes>300))
			{
				ChLcpIterativeSolver* msolver = (ChLcpIterativeSolver*)mphysicalSystem.GetLcpSolverStab();
				if (msolver->GetViolationHistory().size()>1)
					data_deltares <<  msolver->GetViolationHistory()[msolver->GetViolationHistory().size()-1] << "\n";
			}

			//***PLOT*** plot deltalambda corrections for varying omega
			if (totframes==999)
			{
				ChLcpIterativeSymmSOR* msolver = (ChLcpIterativeSymmSOR*)mphysicalSystem.GetLcpSolverSpeed();
				msolver->SetOmega(1.0);
			}
			if (totframes==1000)
			{
				GetLog()<< "Saving plot data\n";
				ChLcpIterativeSolver* msolver = (ChLcpIterativeSolver*)mphysicalSystem.GetLcpSolverSpeed();
				ChStreamOutAsciiFile data_deltalambda("data_deltalambda_om80.dat");
				for (int k=0; k< msolver->GetDeltalambdaHistory().size(); k++)
				{
					data_deltalambda <<  msolver->GetDeltalambdaHistory()[k] << "\n";
				}
			}
			*/

			if ((totframes>200)&&(totframes<=500))
			{
				//data_timexconstr << mphysicalSystem.GetTimerLcp()/(double)mphysicalSystem.GetNcontacts() << "\n";
				//GetLog()<< "  avg time/contacts " << mphysicalSystem.GetTimerLcp()/(double)mphysicalSystem.GetNcontacts() << "\n";
				//GetLog()<< "  ncontacts " << (double)mphysicalSystem.GetNcontacts() << "\n";
			}

			if(totframes>500) break;


		}  // ------- end of simulation loop -----



		mtimer.stop();
		GetLog()<< "Time used for simulating/viewing first 500 steps: " << mtimer() << "\n";
		GetLog()<< "  num contacts " << mphysicalSystem.GetNcontacts() << "\n";
		GetLog()<< "  avg time/contacts " << mtimer()/(double)mphysicalSystem.GetNcontacts() << "\n";
		data_spheres_times << nspheres << "  " << mtimer() << "\n";
		data_contacts_times << mphysicalSystem.GetNcontacts() << "  " << mtimer() << "\n";
		

		// remove the root scene node (contains walls & spheres but no camera no lights)
		// to reset and prepare for new scene at next for() loop.
		parent->remove();

		// Now the system should already have no bodies, because of full removal from Irrlicht scene.
		// Then, the following clearing should be unuseful...(do it anyway..)
		mphysicalSystem.Clear(); 
	}



	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	getchar();

	return 0;
}


