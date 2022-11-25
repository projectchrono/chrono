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
// Author: Radu Serban
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <ctime>

#include "chrono/physics/ChSystemSMC.h"

#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/utils/ChUtilsTypeConvert.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fsi;

const std::string out_dir = GetChronoOutputPath() + "FSI_BCE/";

int main(int argc, char* argv[]) {
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    ChSystemSMC sysMBS;
    ChSystemFsi sysFSI(&sysMBS);

    sysFSI.SetInitialSpacing(0.025);
    sysFSI.SetKernelLength(0.025);

    // Dummy body

    auto body = chrono_types::make_shared<ChBody>();
    body->SetPos(ChVector<>(-1, -2, -3));
    body->SetPos_dt(ChVector<>(0.2, 0.3, 0.4));
    body->SetRot(Q_from_AngAxis(CH_C_PI / 4, ChVector<>(0, 1, 0)));
    body->SetWvel_loc(ChVector<>(0.1, -0.1, 0.2));

    ChFrame<> frame(ChVector<>(1, 2, 3), Q_from_AngX(CH_C_PI / 8));
    thrust::host_vector<Real4> bce;
    std::ofstream fbce;

    // Box

    ChVector<> box_size(0.2, 0.4, 0.6);

    bce.clear();
    sysFSI.CreateBCE_box(fsi::utils::ToReal3(box_size), true, bce);
    std::cout << "box solid nBCE = " << bce.size() << std::endl;
    fbce.open((out_dir + "/box_solid.txt"), std::ios::trunc);
    for (int i = 0; i < bce.size(); i++) {
        fbce << bce[i].x << " " << bce[i].y << " " << bce[i].z << std::endl;
    }
    fbce.close();
    sysFSI.AddBoxBCE(body, frame, box_size, true);

    bce.clear();
    sysFSI.CreateBCE_box(fsi::utils::ToReal3(box_size), false, bce);
    std::cout << "box bndry nBCE = " << bce.size() << std::endl;
    fbce.open((out_dir + "/box_bndry.txt"), std::ios::trunc);
    for (int i = 0; i < bce.size(); i++) {
        fbce << bce[i].x << " " << bce[i].y << " " << bce[i].z << std::endl;
    }
    fbce.close();
    sysFSI.AddBoxBCE(body, frame, box_size, false);

    // Sphere

    Real sph_radius = 0.25;

    bce.clear();
    sysFSI.CreateBCE_sphere(sph_radius, true, true, bce);
    std::cout << "sphere solid polar nBCE = " << bce.size() << std::endl;
    fbce.open((out_dir + "/sphere_solid_polar.txt"), std::ios::trunc);
    for (int i = 0; i < bce.size(); i++) {
        fbce << bce[i].x << " " << bce[i].y << " " << bce[i].z << std::endl;
    }
    fbce.close();
    sysFSI.AddSphereBCE(body, frame, sph_radius, true, true);

    bce.clear();
    sysFSI.CreateBCE_sphere(sph_radius, false, true, bce);
    std::cout << "sphere bndry polar nBCE = " << bce.size() << std::endl;
    fbce.open((out_dir + "/sphere_bndry_polar.txt"), std::ios::trunc);
    for (int i = 0; i < bce.size(); i++) {
        fbce << bce[i].x << " " << bce[i].y << " " << bce[i].z << std::endl;
    }
    fbce.close();
    sysFSI.AddSphereBCE(body, frame, sph_radius, false, true);

    bce.clear();
    sysFSI.CreateBCE_sphere(sph_radius, true, false, bce);
    std::cout << "sphere solid cartesian nBCE = " << bce.size() << std::endl;
    fbce.open((out_dir + "/sphere_solid_cartesian.txt"), std::ios::trunc);
    for (int i = 0; i < bce.size(); i++) {
        fbce << bce[i].x << " " << bce[i].y << " " << bce[i].z << std::endl;
    }
    fbce.close();
    sysFSI.AddSphereBCE(body, frame, sph_radius, true, false);

    bce.clear();
    sysFSI.CreateBCE_sphere(sph_radius, false, false, bce);
    std::cout << "sphere bndry cartesian nBCE = " << bce.size() << std::endl;
    fbce.open((out_dir + "/sphere_bndry_cartesian.txt"), std::ios::trunc);
    for (int i = 0; i < bce.size(); i++) {
        fbce << bce[i].x << " " << bce[i].y << " " << bce[i].z << std::endl;
    }
    fbce.close();
    sysFSI.AddSphereBCE(body, frame, sph_radius, false, false);

    // Cylinder

    Real cyl_radius = 0.25;
    Real cyl_height = 0.4;
    bool cyl_capped = true;

    bce.clear();
    sysFSI.CreateBCE_cylinder(cyl_radius, cyl_height, true, cyl_capped, true, bce);
    std::cout << "cylinder solid polar nBCE = " << bce.size() << std::endl;
    fbce.open((out_dir + "/cyl_solid_polar.txt"), std::ios::trunc);
    for (int i = 0; i < bce.size(); i++) {
        fbce << bce[i].x << " " << bce[i].y << " " << bce[i].z << std::endl;
    }
    fbce.close();
    sysFSI.AddCylinderBCE(body, frame, cyl_radius, cyl_height, true, cyl_capped, true);

    bce.clear();
    sysFSI.CreateBCE_cylinder(cyl_radius, cyl_height, false, cyl_capped, true, bce);
    std::cout << "cylinder bndry polar nBCE = " << bce.size() << std::endl;
    fbce.open((out_dir + "/cyl_bndry_polar.txt"), std::ios::trunc);
    for (int i = 0; i < bce.size(); i++) {
        fbce << bce[i].x << " " << bce[i].y << " " << bce[i].z << std::endl;
    }
    fbce.close();
    sysFSI.AddCylinderBCE(body, frame, cyl_radius, cyl_height, false, cyl_capped, true);

    bce.clear();
    sysFSI.CreateBCE_cylinder(cyl_radius, cyl_height, true, cyl_capped, false, bce);
    std::cout << "cylinder solid cartesian nBCE = " << bce.size() << std::endl;
    fbce.open((out_dir + "/cyl_solid_cartesian.txt"), std::ios::trunc);
    for (int i = 0; i < bce.size(); i++) {
        fbce << bce[i].x << " " << bce[i].y << " " << bce[i].z << std::endl;
    }
    fbce.close();
    sysFSI.AddCylinderBCE(body, frame, cyl_radius, cyl_height, true, cyl_capped, false);

    bce.clear();
    sysFSI.CreateBCE_cylinder(cyl_radius, cyl_height, false, cyl_capped, false, bce);
    std::cout << "cylinder bndry cartesian nBCE = " << bce.size() << std::endl;
    fbce.open((out_dir + "/cyl_bndry_cartesian.txt"), std::ios::trunc);
    for (int i = 0; i < bce.size(); i++) {
        fbce << bce[i].x << " " << bce[i].y << " " << bce[i].z << std::endl;
    }
    fbce.close();
    sysFSI.AddCylinderBCE(body, frame, cyl_radius, cyl_height, false, cyl_capped, false);

    // Cylindrical annulus

    Real ca_radius_inner = 0.2;
    Real ca_radius_outer = 0.4;
    Real ca_height = 0.2;

    bce.clear();
    sysFSI.CreateBCE_cylinder_annulus(ca_radius_inner, ca_radius_outer, ca_height, true, bce);
    std::cout << "cylinder annulus polar nBCE = " << bce.size() << std::endl;
    fbce.open((out_dir + "/cyl_annulus_polar.txt"), std::ios::trunc);
    for (int i = 0; i < bce.size(); i++) {
        fbce << bce[i].x << " " << bce[i].y << " " << bce[i].z << std::endl;
    }
    fbce.close();
    sysFSI.AddCylinderAnnulusBCE(body, frame, ca_radius_inner, ca_radius_outer, ca_height, true);

    bce.clear();
    sysFSI.CreateBCE_cylinder_annulus(ca_radius_inner, ca_radius_outer, ca_height, false, bce);
    std::cout << "cylinder annulus cartesian nBCE = " << bce.size() << std::endl;
    fbce.open((out_dir + "/cyl_annulus_cartesian.txt"), std::ios::trunc);
    for (int i = 0; i < bce.size(); i++) {
        fbce << bce[i].x << " " << bce[i].y << " " << bce[i].z << std::endl;
    }
    fbce.close();
    sysFSI.AddCylinderAnnulusBCE(body, frame, ca_radius_inner, ca_radius_outer, ca_height, false);

    // Cone

    Real cone_radius = 0.25;
    Real cone_height = 0.2;
    bool cone_capped = false;

    bce.clear();
    sysFSI.CreateBCE_cone(cone_radius, cone_height, true, cone_capped, true, bce);
    std::cout << "cone solid polar nBCE = " << bce.size() << std::endl;
    fbce.open((out_dir + "/cone_solid_polar.txt"), std::ios::trunc);
    for (int i = 0; i < bce.size(); i++) {
        fbce << bce[i].x << " " << bce[i].y << " " << bce[i].z << std::endl;
    }
    fbce.close();
    sysFSI.AddConeBCE(body, frame, cone_radius, cone_height, true, cone_capped, true);

    bce.clear();
    sysFSI.CreateBCE_cone(cone_radius, cone_height, false, cone_capped, true, bce);
    std::cout << "cone bndry polar nBCE = " << bce.size() << std::endl;
    fbce.open((out_dir + "/cone_bndry_polar.txt"), std::ios::trunc);
    for (int i = 0; i < bce.size(); i++) {
        fbce << bce[i].x << " " << bce[i].y << " " << bce[i].z << std::endl;
    }
    fbce.close();
    sysFSI.AddConeBCE(body, frame, cone_radius, cone_height, false, cone_capped, true);

    bce.clear();
    sysFSI.CreateBCE_cone(cone_radius, cone_height, true, cone_capped, false, bce);
    std::cout << "cone solid cartesian nBCE = " << bce.size() << std::endl;
    fbce.open((out_dir + "/cone_solid_cartesian.txt"), std::ios::trunc);
    for (int i = 0; i < bce.size(); i++) {
        fbce << bce[i].x << " " << bce[i].y << " " << bce[i].z << std::endl;
    }
    fbce.close();
    sysFSI.AddConeBCE(body, frame, cone_radius, cone_height, true, cone_capped, false);

    bce.clear();
    sysFSI.CreateBCE_cone(cone_radius, cone_height, false, cone_capped, false, bce);
    std::cout << "cone bndry cartesian nBCE = " << bce.size() << std::endl;
    fbce.open((out_dir + "/cone_bndry_cartesian.txt"), std::ios::trunc);
    for (int i = 0; i < bce.size(); i++) {
        fbce << bce[i].x << " " << bce[i].y << " " << bce[i].z << std::endl;
    }
    fbce.close();
    sysFSI.AddConeBCE(body, frame, cone_radius, cone_height, false, cone_capped, false);

    return 0;
}
