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
// Authors: Radu Serban
// =============================================================================
//
// Test of operations with 3d vectors
//
// =============================================================================

#include "gtest/gtest.h"
#include "chrono/core/ChVector3.h"

#include <random>

using namespace chrono;

const double ABS_ERR_D = 1e-15;
const float ABS_ERR_F = 1e-6f;

TEST(ChVectorTest, normalize) {
    ChVector3d ad(1.1, -2.2, 3.3);
    ASSERT_NEAR(ad.GetNormalized().Length(), 1.0, ABS_ERR_D);
    ASSERT_TRUE(ad.Normalize());
    ASSERT_NEAR(ad.Length(), 1.0, ABS_ERR_D);

    ChVector3d bd(0.0);
    ASSERT_FALSE(bd.Normalize());

    ChVector3f af(1.1f, -2.2f, 3.3f);
    ASSERT_NEAR(af.GetNormalized().Length(), 1.0f, ABS_ERR_F);
    ASSERT_TRUE(af.Normalize());
    ASSERT_NEAR(af.Length(), 1.0f, ABS_ERR_F);

    ChVector3f bf(0.0f);
    ASSERT_FALSE(bf.Normalize());
}

TEST(ChVectorTest, dot) {
    ChVector3d ad(1.1, -2.2, 3.3);
    ASSERT_NEAR(ad.Dot(ad), ad.Length2(), ABS_ERR_D);
    ASSERT_NEAR(ad.Dot(-ad), -ad.Length2(), ABS_ERR_D);
    ASSERT_NEAR(ad.Dot(ad.GetOrthogonalVector()), 0.0, ABS_ERR_D);

    ChVector3f af(1.1f, -2.2f, 3.3f);
    ASSERT_NEAR(af.Dot(af), af.Length2(), ABS_ERR_F);
    ASSERT_NEAR(af.Dot(-af), -af.Length2(), ABS_ERR_F);
    ASSERT_NEAR(af.Dot(af.GetOrthogonalVector()), 0.0f, ABS_ERR_F);
}

TEST(ChVectorTest, cross) {
    ChVector3d ad(1.1, -2.2, 3.3);
    ChVector3d bd(-0.5, 0.6, 0.7);
    auto cd = ad.Cross(bd);
    ASSERT_NEAR(cd.Dot(ad), 0.0, ABS_ERR_D);
    ASSERT_NEAR(cd.Dot(bd), 0.0, ABS_ERR_D);

    auto zd1 = ad.Cross(ad);
    ASSERT_NEAR(zd1.x(), 0.0, ABS_ERR_D);
    ASSERT_NEAR(zd1.y(), 0.0, ABS_ERR_D);
    ASSERT_NEAR(zd1.z(), 0.0, ABS_ERR_D);

    auto zd2 = ad.Cross(-ad);
    ASSERT_NEAR(zd2.x(), 0.0, ABS_ERR_D);
    ASSERT_NEAR(zd2.y(), 0.0, ABS_ERR_D);
    ASSERT_NEAR(zd2.z(), 0.0, ABS_ERR_D);

    auto pd = ad.GetOrthogonalVector();
    ASSERT_NEAR(ad.Cross(pd).Length(), ad.Length() * pd.Length(), ABS_ERR_D);

    ChVector3f af(1.1f, -2.2f, 3.3f);
    ChVector3f bf(-0.5f, 0.6f, 0.7f);
    auto cf = af.Cross(bf);
    ASSERT_NEAR(cf.Dot(af), 0.0f, ABS_ERR_F);
    ASSERT_NEAR(cf.Dot(bf), 0.0f, ABS_ERR_F);

    auto zf1 = af.Cross(af);
    ASSERT_NEAR(zf1.x(), 0.0f, ABS_ERR_F);
    ASSERT_NEAR(zf1.y(), 0.0f, ABS_ERR_F);
    ASSERT_NEAR(zf1.z(), 0.0f, ABS_ERR_F);

    auto zf2 = af.Cross(-af);
    ASSERT_NEAR(zf2.x(), 0.0f, ABS_ERR_F);
    ASSERT_NEAR(zf2.y(), 0.0f, ABS_ERR_F);
    ASSERT_NEAR(zf2.z(), 0.0f, ABS_ERR_F);

    auto pf = af.GetOrthogonalVector();
    ASSERT_NEAR(af.Cross(pf).Length(), af.Length() * pf.Length(), ABS_ERR_F);
}

TEST(ChVectorTest, gram_schmidt) {
    // Temporary verification test

    // Generate Random vectors
    long loops = 1000;
    double tol = 1e-10; // Numerical errors of complex process needs larger tolerance than ABS_ERR_D
    std::random_device rd;
    std::default_random_engine re(rd());
    std::uniform_real_distribution<double> dist(-10.0, 10.0);
    // Ensures results from ChVector3::GetDirectionAxesAsX, ChVector3::GetDirectionAxesAsY, ChVector3::GetDirectionAxesAsZ
    ChVector3<> Vx_old;
    ChVector3<> Vy_old;
    ChVector3<> Vz_old;
    // Are the same as refactored ChVector3::GetDirectionAxes
    ChVector3<> Vx_new;
    ChVector3<> Vy_new;
    ChVector3<> Vz_new;




    //////////////////////////////
    // 1. ---- x_dir.GetDirectionAxesAsX(Vx, Vy, Vz, y_sugg) == xdir.GetDirectionAxes(Vx, Vy, Vz, y_sugg)
    //////////////////////////////
    ChVector3<> x_dir;
    ChVector3<> y_sugg;

    // 1.1 -- Corner cases:
    x_dir.Set(0.0, -1.5, 0.0);

    // Second vector is zero
    y_sugg.SetNull();
    x_dir.GetDirectionAxesAsX(Vx_old, Vy_old, Vz_old, y_sugg);
    x_dir.GetDirectionAxes(Vx_new, Vy_new, Vz_new, y_sugg);
    std::cout<<"New heuristic expected to have different behavior than old one so no formal test"<<std::endl;
    std::cout<<"This is okay: no simulation should rely on one given arbitrary heuristic rollback of the bad input to function correctly"<<std::endl;
    std::cout<<Vx_old<<" | "<<Vy_old<<" | "<<Vz_old<<std::endl;
    std::cout<<Vx_new<<" | "<<Vy_new<<" | "<<Vz_new<<std::endl<<std::endl;

    // Second vector is collinear to first vector
    y_sugg = x_dir * 3.0;
    x_dir.GetDirectionAxesAsX(Vx_old, Vy_old, Vz_old, y_sugg);
    x_dir.GetDirectionAxes(Vx_new, Vy_new, Vz_new, y_sugg);
    std::cout<<Vx_old<<" | "<<Vy_old<<" | "<<Vz_old<<std::endl;
    std::cout<<Vx_new<<" | "<<Vy_new<<" | "<<Vz_new<<std::endl<<std::endl;
    

    // 1.2 -- Good (random) input
    for (long i =0 ; i < loops ; i++) {
        x_dir.Set(dist(re), dist(re), dist(re));
        y_sugg.Set(dist(re), dist(re), dist(re));

        x_dir.GetDirectionAxesAsX(Vx_old, Vy_old, Vz_old, y_sugg);
        x_dir.GetDirectionAxes(Vx_new, Vy_new, Vz_new, y_sugg);
        
        for (int i = 0; i < 3; i++) {
            ASSERT_NEAR(Vx_old[i], Vx_new[i], tol);
            ASSERT_NEAR(Vy_old[i], Vy_new[i], tol);
            ASSERT_NEAR(Vz_old[i], Vz_new[i], tol);
        }
    }

    //////////////////////////////
    // 2. ---- ydir.GetDirectionAxesAsY(Vx, Vy, Vz, x_sugg) == ydir.GetDirectionAxes(Vy, Vz, Vx, z_sugg)
    //////////////////////////////
    ChVector3<> y_dir;
    ChVector3<> z_sugg;

    // 2.1 -- Corner cases:
    y_dir.Set(1.2, -1.5, 0.0);

    // Second vector is zero
    z_sugg.SetNull();
    y_dir.GetDirectionAxesAsY(Vx_old, Vy_old, Vz_old, z_sugg);
    y_dir.GetDirectionAxes(Vy_new, Vz_new, Vx_new, z_sugg);
    std::cout<<"New heuristic expected to have different behavior than old one so no formal test"<<std::endl;
    std::cout<<"This is okay: no simulation should rely on one given arbitrary heuristic rollback of the bad input to function correctly"<<std::endl;
    std::cout<<Vx_old<<" | "<<Vy_old<<" | "<<Vz_old<<std::endl;
    std::cout<<Vx_new<<" | "<<Vy_new<<" | "<<Vz_new<<std::endl<<std::endl;

    // Second vector is collinear to first vector
    z_sugg = y_dir * 16.78;
    y_dir.GetDirectionAxesAsY(Vx_old, Vy_old, Vz_old, z_sugg);
    y_dir.GetDirectionAxes(Vy_new, Vz_new, Vx_new, z_sugg);
    std::cout<<"UTEST WARNING: THIS SHOULD TRIGGER A COLLINEAR WARNING"<<std::endl;
    std::cout<<"THE FACT THAT IT DOES NOT SUGGESTS THE NUMERIC LIMITS IN Normalize() MIGHT BE TOO SMALL (see TODO JBC)"<<std::endl;
    std::cout<<Vx_old<<" | "<<Vy_old<<" | "<<Vz_old<<std::endl;
    std::cout<<Vx_new<<" | "<<Vy_new<<" | "<<Vz_new<<std::endl<<std::endl;    

    // 2.2 -- Good (random) input
    for (long i =0 ; i < loops ; i++) {
        y_dir.Set(dist(re), dist(re), dist(re));
        z_sugg.Set(dist(re), dist(re), dist(re));

        y_dir.GetDirectionAxesAsY(Vx_old, Vy_old, Vz_old, z_sugg);
        y_dir.GetDirectionAxes(Vy_new, Vz_new, Vx_new, z_sugg);
        
        for (int i = 0; i < 3; i++) {            
            ASSERT_NEAR(Vx_old[i], Vx_new[i], tol);
            ASSERT_NEAR(Vy_old[i], Vy_new[i], tol);
            ASSERT_NEAR(Vz_old[i], Vz_new[i], tol);
        }
    }

    //////////////////////////////
    // 3. ---- zdir.GetDirectionAxesAsZ(Vx, Vy, Vz, x_sugg) == zdir.GetDirectionAxes(Vz, Vx, Vy, x_sugg)
    //////////////////////////////
    ChVector3<> z_dir;
    ChVector3<> x_sugg;

    // 3.1 -- Corner cases:
    z_dir.Set(1.2, -1.5, 13.157);

    // Second vector is zero
    x_sugg.SetNull();
    z_dir.GetDirectionAxesAsZ(Vx_old, Vy_old, Vz_old, x_sugg);
    z_dir.GetDirectionAxes(Vz_new, Vx_new, Vy_new, x_sugg);
    std::cout<<"New heuristic expected to have different behavior than old one so no formal test"<<std::endl;
    std::cout<<"This is okay: no simulation should rely on one given arbitrary heuristic rollback of the bad input to function correctly"<<std::endl;
    std::cout<<Vx_old<<" | "<<Vy_old<<" | "<<Vz_old<<std::endl;
    std::cout<<Vx_new<<" | "<<Vy_new<<" | "<<Vz_new<<std::endl<<std::endl;

    // Second vector is collinear to first vector
    x_sugg = z_dir * (-6.59);
    z_dir.GetDirectionAxesAsZ(Vx_old, Vy_old, Vz_old, x_sugg);
    z_dir.GetDirectionAxes(Vz_new, Vx_new, Vy_new, x_sugg);
    std::cout<<"UTEST WARNING: THIS SHOULD TRIGGER A COLLINEAR WARNING"<<std::endl;
    std::cout<<"THE FACT THAT IT DOES NOT SUGGESTS THE NUMERIC LIMITS IN Normalize() MIGHT BE TOO SMALL (see TODO JBC)"<<std::endl;
    std::cout<<Vx_old<<" | "<<Vy_old<<" | "<<Vz_old<<std::endl;
    std::cout<<Vx_new<<" | "<<Vy_new<<" | "<<Vz_new<<std::endl<<std::endl;
    

    // 2.2 -- Good (random) input
    for (long i =0 ; i < loops ; i++) {
        z_dir.Set(dist(re), dist(re), dist(re));
        x_sugg.Set(dist(re), dist(re), dist(re));

        z_dir.GetDirectionAxesAsZ(Vx_old, Vy_old, Vz_old, x_sugg);
        z_dir.GetDirectionAxes(Vz_new, Vx_new, Vy_new, x_sugg);
        
        for (int i = 0; i < 3; i++) {
            ASSERT_NEAR(Vx_old[i], Vx_new[i], tol);
            ASSERT_NEAR(Vy_old[i], Vy_new[i], tol);
            ASSERT_NEAR(Vz_old[i], Vz_new[i], tol);
        }
    }
}
