// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#include <iostream>
#include <string>
#include <cmath>
#include <ctime>
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_granular/api/ChApiGranularChrono.h"
#include "chrono_granular/physics/ChGranular.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono_granular/utils/ChCudaMathUtils.cuh"
#include "demos/granular/ChGranularDemoUtils.hpp"
#include "chrono/core/ChStream.h"
#include "chrono/core/ChVector.h"

using namespace chrono;
using namespace chrono::granular;

// Show command line usage
void ShowUsage(std::string name) {
    std::cout << "usage: " + name + " <kn mg/R> " + "< gamma_n >" << std::endl;
}

float getMass(float rad, float density){
    float volume = 4.0f/3.0f * CH_C_PI * std::pow(rad, 3);
    float mass = volume * density;
    return mass;
}

// calculate kinetic energy of the system
float getSystemKE(float rad, float density, ChGranularSMC_API &apiSMC, int numSpheres){
    float sysKE = 0.0f;
    float sphere_KE;
    ChVector<float> angularVelo;
    ChVector<float> velo;
    float mass = getMass(rad, density);
    float inertia = 0.4f * mass * std::pow(rad,2);

    for (int i = 0; i < numSpheres; i++){
        angularVelo = apiSMC.getAngularVelo(i);
        velo = apiSMC.getVelo(i);
        sphere_KE = 0.5f * mass * velo.Length2() + 0.5f * inertia * angularVelo.Length2();
        sysKE = sysKE + sphere_KE;
    }
    return sysKE;
}

void calculateBoundaryForces(ChSystemGranularSMC &gran_sys, 
                             int numSpheres, 
                             float rad,
                             float kn,
                             float gn,
                             float mass,
                             float bottom_plate_position,
                             std::vector<ChVector<float>> &normalForces,
                             std::vector<int> &particlesInContact){
    float3 velo;
    float penetration;
    float force_multiplier;
    float3 contact_normal = make_float3(0.0f, 0.0f, 1.0f);

    for (int i = 0; i < numSpheres; i++){
        float3 pos  = gran_sys.getPosition(i);

        // check if it's in contact with the bottom boundary
        if (pos.z - rad < bottom_plate_position){
            penetration = std::abs(pos.z - rad - bottom_plate_position);
            force_multiplier = sqrt(penetration/rad);
            float3 Fn = kn * penetration * contact_normal;

            velo = gran_sys.getVelocity(i);
            float3 rel_vel = velo;

            float projection = Dot(rel_vel, contact_normal);

            // add damping
            Fn = Fn + -1. * gn * projection * contact_normal * mass;
            Fn = Fn * force_multiplier;
            normalForces.push_back(ChVector<float>(Fn.x, Fn.y, Fn.z));
            particlesInContact.push_back(i);
        }

    }
}


// initialize velocity, dimenstion of the slab: x_dim_num * radius by y_dim_num * radius
std::vector<ChVector<float>> initializePositions(int x_dim_num, int z_dim_num, float radius){
    std::vector<ChVector<float>> pos;
    float z = (-(float)z_dim_num/2.0f + 1.0f) * radius;
    float y = 0;
    float z_diff = std::sqrt(3.0f) * radius;
    float x;
    int layers = 0;
    int total_layers = 15;
    while (z <= ((float)z_dim_num/2.0f - 3.0f) * radius - z_diff){
        x = (-(float)x_dim_num/2.0f + 1.0f) * radius;
        while (x <= ((float)x_dim_num/2.0f - 1.0f) * radius){
            ChVector<float> position(x, y, z);
            pos.push_back(position);
            x = x + 2 * radius;
        }
        layers = layers+1;

        if (layers == total_layers){
            break;
        }

        x = (-(float)x_dim_num/2.0f + 2.0f) * radius;
        z = z + z_diff;
        while (x <= ((float)x_dim_num/2.0f - 1.0f) * radius){
            ChVector<float> position(x, y, z);
            pos.push_back(position);
            x = x + 2 * radius;
        }
        layers = layers + 1;
        if (layers == total_layers){
            break;
        }

        z = z + z_diff;
    }
    return pos;

}

// find top center sphere ID
int findTopCenterSphereID(ChSystemGranularSMC &gran_sys, int numSpheres){
    double max_z = gran_sys.get_max_z();
    float3 particlePos;
    for (int i = 0; i < numSpheres; i++){
        particlePos = gran_sys.getPosition(i);
        if (std::abs(particlePos.x) < 0.001 && std::abs(particlePos.z - max_z) < 0.01){
            std::cout << "top sphere ID: " << i << ", pos: " << particlePos.x << ", " << particlePos.z << std::endl;
            return i;
            break;
        }
    }

    printf("ERROR! can not find the particle, check if the setup is correct.\n");
    return -1;
}

int main(int argc, char* argv[]) {


    if (argc != 3) {
        ShowUsage(argv[0]);
        return 1;
    }


    // sphere parameters
    float sphere_radius = 5;

    int x_dim_num = 122;
    int z_dim_num = 30;

    // box dim
    float box_X = x_dim_num * sphere_radius;
    float box_Y = box_X;
    float box_Z = z_dim_num * sphere_radius;
    
    // wave propagation test related parameters
    float force_ratio = 1.0f;


    // material based parameter
    float sphere_density = 1.15;
    float sphere_volume = 4.0f/3.0f * CH_C_PI * sphere_radius * sphere_radius * sphere_radius;
    float sphere_mass = sphere_density * sphere_volume;

    float gravity = 980;

    float kn = std::stof(argv[1]) * sphere_mass * gravity/sphere_radius;
    float kt = 0.0f;
    // damping parameters
    float gamma_n = std::stof(argv[2]);
    float gamma_t = 0.0f;

    printf("run test kn: %e, gn: %e\n", kn, gamma_n);

    // friction coefficient
    float mu_s_s2s = 0.0f;
    float mu_s_s2w = 0.0f;

    // set gravity
    float grav_X = 0.0f;
    float grav_Y = 0.0f;
    float grav_Z = -980.0f;

    // time integrator
    float step_size = 1e-5;
    float minimum_end_time = 40.0f;
    float maximum_end_time = 80.0f;

    // setup simulation gran_sys
    ChSystemGranularSMC gran_sys(sphere_radius, sphere_density,
                                 make_float3(box_X, box_Y, box_Z));

    ChGranularSMC_API apiSMC;
    apiSMC.setGranSystem(&gran_sys);

    std::vector<ChVector<float>> initialPos = initializePositions(x_dim_num, z_dim_num, sphere_radius);

    int numSpheres = initialPos.size();


    // initialPos.push_back(ChVector<float>(0.0f, 0.0f, initialPos.at(numSpheres-1).z()+2*sphere_radius));

    std::cout << "number of spheres: " << initialPos.size();

    apiSMC.setElemsPositions(initialPos);

    float psi_T = 32.0f;
    float psi_L = 256.0f;
    gran_sys.setPsiFactors(psi_T, psi_L);


    // normal force model
    gran_sys.set_K_n_SPH2SPH(kn);
    gran_sys.set_K_n_SPH2WALL(2*kn);
    gran_sys.set_Gamma_n_SPH2SPH(gamma_n);
    gran_sys.set_Gamma_n_SPH2WALL(gamma_n);
    
    //tangential force model
    gran_sys.set_friction_mode(GRAN_FRICTION_MODE::MULTI_STEP);
    gran_sys.set_K_t_SPH2SPH(2.0f/7.0f * kn);
    gran_sys.set_K_t_SPH2WALL(2.0f/7.0f * kn);
    gran_sys.set_Gamma_t_SPH2SPH(gamma_t);
    gran_sys.set_Gamma_t_SPH2WALL(gamma_t);
    gran_sys.set_static_friction_coeff_SPH2SPH(mu_s_s2s);
    gran_sys.set_static_friction_coeff_SPH2WALL(mu_s_s2w);

    gran_sys.set_gravitational_acceleration(grav_X, grav_Y, grav_Z);

    

    // Set the position of the BD
    gran_sys.set_BD_Fixed(true);
    gran_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::FORWARD_EULER);
    gran_sys.set_fixed_stepSize(step_size);



    gran_sys.initialize();
    
    int top_center_sphereID = findTopCenterSphereID(gran_sys, numSpheres);
    if (top_center_sphereID == -1){
        std::cout << "ERROR! can not find top center sphere! check setup\n";
        return -1;
    }
    // start with no external force from top
    gran_sys.setWavePropagationParameters(top_center_sphereID, 0.0f, gravity);

    float curr_time = 0;

    // initialize values that I want to keep track of
    float sysKE;
    // print out stepsize
    float print_time_step = 5000.0f * step_size;

    // std::string output_dir = "out_6mg_kn" + std::to;
    char output_dir[100];
    sprintf(output_dir, "longerT_kn_%06d_gn_%06d", (int)(kn), (int)gamma_n);

    filesystem::create_directory(filesystem::path(output_dir));

    char filename[100];
    int currframe = 0;
    clock_t start = std::clock();
    float coeff_old = 100; // for determine whether there still any changes in KE

    printf("time, avgKE (mgR), relative error of reaction force\n");
	while (curr_time < 20) {
        gran_sys.advance_simulation(print_time_step);
        curr_time += print_time_step;

        sysKE = getSystemKE(sphere_radius, sphere_density, apiSMC, numSpheres);
        
        float avgSysKE = sysKE/numSpheres;
        float coeff = avgSysKE/(sphere_mass*gravity*sphere_radius);  
        float diff = std::abs(coeff - coeff_old);
        if (curr_time > 5 && diff/coeff_old < 1E-4){
            sprintf(filename, "%s/step%06d", output_dir, currframe++);
            gran_sys.writeFile(std::string(filename));

            break;
        }

        coeff_old = coeff;

        float bottom_wall_force[3];
        int bottomWallID = 4;
        gran_sys.getBCReactionForces(bottomWallID, bottom_wall_force);
        float totalWeight = numSpheres * sphere_mass * gravity;
        float relative_error = std::abs(std::abs(bottom_wall_force[2]) - totalWeight)/totalWeight;
        printf("%e, %e\n", curr_time, coeff);
        //        write position info
        sprintf(filename, "%s/step%06d", output_dir, currframe++);
        gran_sys.writeFile(std::string(filename));

    }

    float settlingTime = curr_time;

    // if (curr_time + step_size > 20){
    //         printf("small changes NOT ACHIEVED, end at avgSysKE: %e\n", coeff_old);
    //         return -1;
    // }


    std::vector<ChVector<float>> normalForces;
    std::vector<int> particlesInContact;
    calculateBoundaryForces(gran_sys, numSpheres, sphere_radius, 2*kn, gamma_n, sphere_mass, -box_Z/2.0f, normalForces, particlesInContact);


    // set all velocity to zero 
    gran_sys.setVelocityZero();


    // start with no external force from top
    float F_ext_ratio = 6.0f;
    float F_duration = 2.0f;

    int F_ext_ratio_array_size = std::floor(F_duration/step_size)+1;
    float F_ext_ratio_array[F_ext_ratio_array_size];
    float slope = F_ext_ratio/F_duration;
    for (int i = 0; i < F_ext_ratio_array_size; i++){
        F_ext_ratio_array[i] = slope * step_size * i;
    }
    float time_end_force = curr_time + F_duration;

    // apply force gradually
    int force_counter = 0;

    while (curr_time < time_end_force && force_counter < F_ext_ratio_array_size){
        // TODO: need to check this 

        gran_sys.advance_simulation(step_size);
        curr_time += step_size;

        gran_sys.setWavePropagationParameters(top_center_sphereID, F_ext_ratio_array[force_counter], gravity);
        force_counter++;

    }

    gran_sys.setWavePropagationParameters(top_center_sphereID, F_ext_ratio, gravity);


	while (true) {

        gran_sys.advance_simulation(print_time_step);
        curr_time += print_time_step;

        sysKE = getSystemKE(sphere_radius, sphere_density, apiSMC, numSpheres);
        
        float avgSysKE = sysKE/numSpheres;
        float coeff = avgSysKE/(sphere_mass*gravity*sphere_radius);        

        float bottom_wall_force[3];
        int bottomWallID = 4;

        // modify this
        gran_sys.getBCReactionForces(bottomWallID, bottom_wall_force);
        float totalWeight = numSpheres * sphere_mass * gravity + F_ext_ratio * sphere_mass * gravity;
        float relative_error = std::abs(std::abs(bottom_wall_force[2]) - totalWeight)/totalWeight;
        
        avgSysKE = sysKE/numSpheres;
        coeff = avgSysKE/(sphere_mass*gravity*sphere_radius);  
        float diff = std::abs(coeff - coeff_old);
        // 5e-3 is for radius = 3mm, can't achieve; 1e-6 for large radius, able to achieve
        if (curr_time > settlingTime + 7 && diff/coeff_old < 1E-4){
            sprintf(filename, "%s/step%06d", output_dir, currframe++);
            gran_sys.writeFile(std::string(filename));

            break;
        }

        coeff_old = coeff;
        printf("%e, %e\n", curr_time, coeff);


        if (curr_time > maximum_end_time){
            break;
        }
        // //        write position info
        sprintf(filename, "%s/step%06d", output_dir, currframe++);
        gran_sys.writeFile(std::string(filename));

    }

    // might want to start a new array for normalForces 
    calculateBoundaryForces(gran_sys, numSpheres, sphere_radius, 2*kn, gamma_n, sphere_mass, -box_Z/2.0f, normalForces, particlesInContact);

    printf("applying external force %f mg, bottom layer reaction force : x, reaction force\n", F_ext_ratio);
    for (int i = 0; i < normalForces.size(); i ++){
        printf("%e, %e\n", gran_sys.getPosition(particlesInContact.at(i)).x, normalForces.at(i).z());
    }

	clock_t end_time = std::clock();
	double computation_time = ((double)(end_time - start)) / CLOCKS_PER_SEC;
	std::cout << "Time: " << computation_time << " seconds" << std::endl;
    return 0;
}

