// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Wei Hu, Radu Serban
// =============================================================================
//
// Definition of the SPH granular TERRAIN NODE (using Chrono::FSI).
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <set>

#include <mpi.h>

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_fsi/utils/ChUtilsJSON.h"
#include "chrono_fsi/utils/ChUtilsGeneratorFsi.h"

#include "chrono_vehicle/cosim/ChVehicleCosimTerrainNodeGranularSPH.h"

using namespace chrono::fsi;

using std::cout;
using std::endl;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construction of the terrain node:
// - create the Chrono system and set solver parameters
// - create the Chrono FSI system
// -----------------------------------------------------------------------------
ChVehicleCosimTerrainNodeGranularSPH::ChVehicleCosimTerrainNodeGranularSPH(bool render)
    : ChVehicleCosimTerrainNode(Type::GRANULAR_SPH, ChContactMethod::SMC, render), m_depth(0) {
    cout << "[Terrain node] GRANULAR_SPH " << endl;

    // Create systems
    m_system = new ChSystemSMC;
    m_systemFSI = new ChSystemFsi(*m_system);

    // Solver settings independent of method type
    m_system->Set_G_acc(ChVector<>(0, 0, m_gacc));

    // Set number of threads
    m_system->SetNumThreads(1);
}

ChVehicleCosimTerrainNodeGranularSPH::~ChVehicleCosimTerrainNodeGranularSPH() {
    delete m_systemFSI;
    delete m_system;
}

void ChVehicleCosimTerrainNodeGranularSPH::SetPropertiesSPH(const std::string& filename, double depth) {
    m_depth = depth;

    // Get the pointer to the system parameter and use a JSON file to fill it out with the user parameters
    m_params = m_systemFSI->GetSimParams();
    fsi::utils::ParseJSON(filename, m_params, fsi::mR3(0, 0, 0));

}

// -----------------------------------------------------------------------------
// Complete construction of the mechanical system.
// This function is invoked automatically from Initialize.
// - adjust system settings
// - create the container body
// - set m_init_height
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNodeGranularSPH::Construct() {
    // Domain size
    fsi::Real bxDim = (fsi::Real)(2 * m_hdimX);
    fsi::Real byDim = (fsi::Real)(2 * m_hdimY);
    fsi::Real bzDim = (fsi::Real)(1.25 * m_depth);

    // Set up the periodic boundary condition (if not, set relative larger values)
    fsi::Real initSpace0 = m_params->MULT_INITSPACE * m_params->HSML;
    m_params->boxDimX = bxDim;
    m_params->boxDimY = byDim;
    m_params->boxDimZ = bzDim;
    m_params->cMin = chrono::fsi::mR3(-bxDim / 2, -byDim / 2, -bzDim - 10 * initSpace0) * 10;
    m_params->cMax = chrono::fsi::mR3(bxDim / 2, byDim / 2, bzDim + 10 * initSpace0) * 10;

    // Set the time integration type and the linear solver type (only for ISPH)
    m_systemFSI->SetFluidDynamics(m_params->fluid_dynamic_type);
    m_systemFSI->SetFluidSystemLinearSolver(m_params->LinearSolver);

    // Call FinalizeDomain to setup the binning for neighbor search
    fsi::utils::FinalizeDomain(m_params);

    const std::string out_dir = GetOutDirName();//GetChronoOutputPath() + "FSI_output_data/";
    std::string demo_dir;
    fsi::utils::PrepareOutputDir(m_params, demo_dir, out_dir, "FSI_JSON");

    //// RADU TODO:  Create the fluid particles...
    //// Fluid domain:  bxDim x byDim x m_depth
    // Dimension of the fluid domain
    fsi::Real fxDim = m_params->fluidDimX;
    fsi::Real fyDim = m_params->fluidDimY;
    fsi::Real fzDim = m_params->fluidDimZ;
    // Create Fluid region and discretize with SPH particles
    ChVector<> boxCenter(0.0, 0.0, fzDim / 2);
    ChVector<> boxHalfDim(fxDim / 2, fyDim / 2, fzDim / 2);

    // Use a chrono sampler to create a bucket of points
    utils::GridSampler<> sampler(initSpace0);
    utils::Generator::PointVector points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add fluid particles from the sampler points to the FSI system
    size_t numPart = points.size();
    for (int i = 0; i < numPart; i++) {
        // Calculate the pressure of a steady state (p = rho*g*h)
        Real pre_ini = m_params->rho0 * abs(m_params->gravity.z) * (-points[i].z() + fzDim);
        Real rho_ini = m_params->rho0 + pre_ini / (m_params->Cs * m_params->Cs);
        m_systemFSI->GetDataManager()->AddSphMarker(
            fsi::mR4(points[i].x(), points[i].y(), points[i].z(), m_params->HSML),
            fsi::mR3(1e-10),
            fsi::mR4(rho_ini, pre_ini, m_params->mu0, -1));
    }
    size_t numPhases = m_systemFSI->GetDataManager()->fsiGeneralData->referenceArray.size();
    if (numPhases != 0) {
        std::cout << "Error! numPhases is wrong, thrown from main\n" << std::endl;
    } else {
        m_systemFSI->GetDataManager()->fsiGeneralData->referenceArray.push_back(fsi::mI4(0, (int)numPart, -1, -1));
        m_systemFSI->GetDataManager()->fsiGeneralData->referenceArray.push_back(fsi::mI4((int)numPart, (int)numPart, 0, 0));
    }

    //// RADU TODO - this function must set m_init_height!
    m_init_height = fzDim;


    // Create container body
    auto container = std::shared_ptr<ChBody>(m_system->NewBody());
    m_system->AddBody(container);
    container->SetIdentifier(-1);
    container->SetMass(1);
    container->SetBodyFixed(true);
    container->SetCollide(false);

    // Create the geometry of the boundaries

    // Bottom and Top wall - size and position
    ChVector<> size_XY(bxDim / 2 + 3 * initSpace0, byDim / 2 + 3 * initSpace0, 2 * initSpace0);
    ChVector<> pos_zp(0, 0, bzDim + 1 * initSpace0);
    ChVector<> pos_zn(0, 0, -3 * initSpace0);

    // Left and right Wall - size and position
    ChVector<> size_YZ(2 * initSpace0, byDim / 2 + 3 * initSpace0, bzDim / 2);
    ChVector<> pos_xp(bxDim / 2 + initSpace0, 0.0, bzDim / 2 + 0 * initSpace0);
    ChVector<> pos_xn(-bxDim / 2 - 3 * initSpace0, 0.0, bzDim / 2 + 0 * initSpace0);

    // Front and back Wall - size and position
    ChVector<> size_XZ(bxDim / 2, 2 * initSpace0, bzDim / 2);
    ChVector<> pos_yp(0, byDim / 2 + initSpace0, bzDim / 2 + 0 * initSpace0);
    ChVector<> pos_yn(0, -byDim / 2 - 3 * initSpace0, bzDim / 2 + 0 * initSpace0);

    // Add BCE particles attached on the walls into FSI system
    fsi::utils::AddBoxBce(m_systemFSI->GetDataManager(), m_params, container, pos_zp, chrono::QUNIT, size_XY, 12);
    fsi::utils::AddBoxBce(m_systemFSI->GetDataManager(), m_params, container, pos_zn, chrono::QUNIT, size_XY, 12);
    fsi::utils::AddBoxBce(m_systemFSI->GetDataManager(), m_params, container, pos_xp, chrono::QUNIT, size_YZ, 23);
    fsi::utils::AddBoxBce(m_systemFSI->GetDataManager(), m_params, container, pos_xn, chrono::QUNIT, size_YZ, 23);
    fsi::utils::AddBoxBce(m_systemFSI->GetDataManager(), m_params, container, pos_yp, chrono::QUNIT, size_XZ, 13);
    fsi::utils::AddBoxBce(m_systemFSI->GetDataManager(), m_params, container, pos_yn, chrono::QUNIT, size_XZ, 13);


    // Write file with terrain node settings
    std::ofstream outf;
    outf.open(m_node_out_dir + "/settings.dat", std::ios::out);
    outf << "System settings" << endl;
    outf << "   Integration step size = " << m_step_size << endl;
    outf << "Patch dimensions" << endl;
    outf << "   X = " << 2 * m_hdimX << "  Y = " << 2 * m_hdimY << endl;
    outf << "   depth = " << m_depth << endl;
}

void AddMeshMarkers(const std::string& mesh_filename, 
                    double spearation, std::vector<ChVector<>>& marker_positions) {
    //load mesh from obj file and create BCE particles
    chrono::geometry::ChTriangleMeshConnected mmesh;
    std::string obj_path = mesh_filename;
    double scale_ratio = 1.0;
    mmesh.LoadWavefrontMesh(obj_path, false, true);
    mmesh.Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));     // scale to a different size
    mmesh.RepairDuplicateVertexes(1e-9);                                 // if meshes are not watertight 

	ChVector<> minV = mmesh.m_vertices[0];
    ChVector<> maxV = mmesh.m_vertices[0];
	ChVector<> currV = mmesh.m_vertices[0];
	for (unsigned int i = 0; i < mmesh.m_vertices.size(); ++i){
		currV = mmesh.m_vertices[i];
		if(minV.x() > currV.x()) minV.x() = currV.x();
		if(minV.y() > currV.y()) minV.y() = currV.y();
		if(minV.z() > currV.z()) minV.z() = currV.z();
		if(maxV.x() < currV.x()) maxV.x() = currV.x();
		if(maxV.y() < currV.y()) maxV.y() = currV.y();
		if(maxV.z() < currV.z()) maxV.z() = currV.z();
	}
	printf("start coords: %f, %f, %f\n", minV.x(), minV.y(), minV.z());
	printf("end coords: %f, %f, %f\n", maxV.x(), maxV.y(), maxV.z());

	std::vector<ChVector<>> point_cloud;
    ChVector<> ray_origin = ChVector<>(0.0,0.0,0.0);
    double delta = spearation;
    double EPSI = 0.000001;
    int total_num = 0;
	for(double x = minV.x(); x < maxV.x(); x += delta){
		ray_origin.x() = x + 1e-9;
		for(double y = minV.y(); y < maxV.y(); y += delta){
			ray_origin.y() = y + 1e-9;
			for(double z = minV.z(); z < maxV.z(); z += delta){
				ray_origin.z() = z + 1e-9;
                int isInsideMesh = 0;
                {
                    ChVector<> ray_dir1 = ChVector<>(5,0.5,0.25);
                    ChVector<> ray_dir2 = ChVector<>(-3,0.7,10);
                    ChVector<int> t_face;
                    ChVector<> v1, v2, v3;
                    int t_inter1, t_inter2;
                    double out;
                    int intersectCounter1 = 0;
                    int intersectCounter2 = 0;
                    for (unsigned int i = 0; i < mmesh.m_face_v_indices.size(); ++i)
                    {
                        t_face  = mmesh.m_face_v_indices[i];
                        v1 = mmesh.m_vertices[t_face.x()];
                        v2 = mmesh.m_vertices[t_face.y()];
                        v3 = mmesh.m_vertices[t_face.z()];

                        t_inter1 = 0;
                        t_inter2 = 0;
                        for (unsigned int j = 0; j < 2; j++){
                            ChVector<> orig = ray_origin;
                            ChVector<> dir;
                            if(j==0){dir = ray_dir1;}
                            if(j==1){dir = ray_dir2;}
                
                            ChVector<> edge1, edge2;  //Edgedge1, Edgedge2
                            ChVector<> pvec, qvec, tvec;
                            double det, inv_det, uu, vv, tt;

                            //Find vectors for two edges sharing V1
                            edge1 = v2 - v1;
                            edge2 = v3 - v1;

                            //Begin calculating determinant - also used to calculate uu parameter
                            pvec = Vcross(dir, edge2);
                            //if determinant is near zero, ray lies in plane of triangle or ray is parallel to plane of triangle
                            det = Vdot(edge1, pvec);
                            //NOT CULLING
                            if(det > -EPSI && det < EPSI){
                                if(j==0){t_inter1 = 0;}
                                if(j==1){t_inter2 = 0;}
                                continue;
                            } 
                            inv_det = 1.0 / det;

                            //calculate distance from V1 to ray origin
                            tvec =  orig - v1;

                            //Calculate uu parameter and test bound
                            uu = Vdot(tvec, pvec) * inv_det;
                            //The intersection lies outside of the triangle
                            if(uu < 0.0 || uu > 1.0) {
                                if(j==0){t_inter1 = 0;}
                                if(j==1){t_inter2 = 0;}
                                continue;
                            }

                            //Prepare to test vv parameter
                            qvec = Vcross(tvec, edge1);

                            //Calculate vv parameter and test bound
                            vv = Vdot(dir, qvec) * inv_det;
                            //The intersection lies outside of the triangle
                            if(vv < 0.0 || ((uu + vv)  > 1.0)) {
                                if(j==0){t_inter1 = 0;}
                                if(j==1){t_inter2 = 0;}
                                continue;	
                            } 

                            tt = Vdot(edge2, qvec) * inv_det;
                            if(tt > EPSI) { //ray intersection
                                if(j==0){t_inter1 = 1;}
                                if(j==1){t_inter2 = 1;}
                                continue;
                            }

                            // No hit, no win
                            if(j==0){t_inter1 = 0;}
                            if(j==1){t_inter2 = 0;}
                        }

                        if(t_inter1 == 1) {intersectCounter1 += 1;}
                        if(t_inter2 == 1) {intersectCounter2 += 1;}
                    }
                    if( ((intersectCounter1 % 2) == 1) && ((intersectCounter2 % 2) == 1) ) {
                        isInsideMesh = 1;
                    }
                    else {
                        isInsideMesh =  0;
                    }
                }
                if(isInsideMesh == 1) {
                    point_cloud.push_back(ChVector<>(x,y,z));
                    total_num++;
                    // printf("current number of particles is: %d \n", total_num);
                }
			}
		}
	}
    marker_positions = point_cloud;
    printf("total number of particles on the mesh is: %d \n", total_num);
}

void ChVehicleCosimTerrainNodeGranularSPH::CreateWheelProxy() {
    // Create wheel proxy body
    auto body = std::shared_ptr<ChBody>(m_system->NewBody());
    body->SetIdentifier(0);
    body->SetMass(m_rig_mass);
    body->SetBodyFixed(true);  // proxy body always fixed
    body->SetCollide(false);

    // Create collision mesh
    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    trimesh->getCoordsVertices() = m_mesh_data.vpos;
    trimesh->getCoordsNormals() = m_mesh_data.vnrm;
    trimesh->getIndicesVertexes() = m_mesh_data.tri;

    m_system->AddBody(body);
    m_proxies.push_back(ProxyBody(body, 0));

    // Add this body to the FSI system
    m_systemFSI->AddFsiBody(body);

    //// RADU TODO - Create BCE markers associated with trimesh!
    // test with a cylinder wheel now
    // Size of the cylinder wheel
    double wheel_length = 0.2;
    double wheel_radius = 0.1;
    fsi::Real initSpace0 = m_params->MULT_INITSPACE * m_params->HSML;
    fsi::utils::AddCylinderBce(m_systemFSI->GetDataManager(), m_params, body, ChVector<>(0, 0, 0),
                               ChQuaternion<>(1, 0, 0, 0), wheel_radius, wheel_length + initSpace0, 
                               m_params->HSML, false);

    //// WEI TODO - will implemente the creation of BCE markers from mesh later
    // now, we can only create BCE from obj file and save the BCE to another file
    // and use AddBCE_FromFile function to load the BCE particles into FSI system
    /*
    std::string obj_path = "./wheel_XXX.obj";
    std::vector<ChVector<>> point_cloud;
    double particle_spacing = initSpace0;
    AddMeshMarkers(obj_path, particle_spacing, point_cloud);

    // write the BCE particles' position into file
    std::ofstream myFileSPH;
    myFileSPH.open("./BCEparticles.txt", std::ios::trunc);
    myFileSPH.close();
    myFileSPH.open("./BCEparticles.txt", std::ios::app);
	myFileSPH << "x" << "," << "y" << "," << "z" << "\n";
	for (int i = 0; i < point_cloud.size(); ++i)
	{
		myFileSPH << point_cloud[i].x() << "," << "\t" 
			      << point_cloud[i].y() << "," << "\t" 
			      << point_cloud[i].z() << "," << "\n";
	}
	myFileSPH.close();
    // load  the BCE particles' position from file
    std::string BCE_path = "./BCEparticles.txt";
    double scale_ratio = 1.0;
    fsi::utils::AddBCE_FromFile(m_systemFSI->GetDataManager(), m_params, body, BCE_path, 
                                ChVector<double>(0), QUNIT, scale_ratio);
    */

    // Construction of the FSI system must be finalized before running
    m_systemFSI->Finalize();
}

// Set state of wheel proxy body.
void ChVehicleCosimTerrainNodeGranularSPH::UpdateWheelProxy() {
    m_proxies[0].m_body->SetPos(m_wheel_state.pos);
    m_proxies[0].m_body->SetPos_dt(m_wheel_state.lin_vel);
    m_proxies[0].m_body->SetRot(m_wheel_state.rot);
    m_proxies[0].m_body->SetWvel_par(m_wheel_state.ang_vel);
    // m_proxies[0].m_body->SetWacc_par(ChVector<>(0.0, 0.0, 0.0));
}

// Collect resultant contact force and torque on wheel proxy body.
void ChVehicleCosimTerrainNodeGranularSPH::GetForceWheelProxy() {
    m_wheel_contact.point = ChVector<>(0, 0, 0);
    m_wheel_contact.force = m_proxies[0].m_body->Get_accumulated_force();
    m_wheel_contact.moment = m_proxies[0].m_body->Get_accumulated_torque();
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularSPH::Advance(double step_size) {
    //// RADU TODO:  correlate m_step_size with m_params->dT

    m_timer.reset();
    m_timer.start();
    double t = 0;
    while (t < step_size) {
        double h = std::min<>(m_params->dT, step_size - t);
        m_systemFSI->DoStepDynamics_FSI();
        t += h;
    }
    m_timer.stop();
    m_cum_sim_time += m_timer();

    PrintWheelProxyContactData();
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularSPH::OutputTerrainData(int frame) {
    //// TODO
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularSPH::PrintWheelProxyUpdateData() {
    //// TODO
}

void ChVehicleCosimTerrainNodeGranularSPH::PrintWheelProxyContactData() {
    //// TODO
}

}  // end namespace vehicle
}  // end namespace chrono
