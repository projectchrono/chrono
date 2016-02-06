//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChForce.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <math.h>

#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include <float.h>
#include <memory.h>

#include "physics/ChForce.h"
#include "physics/ChBody.h"

namespace chrono {

//////////////////////////////////////
//////////////////////////////////////

// CLASS FOR FORCES

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChForce> a_registration_ChForce;

///////////////////////////////////////

ChForce::ChForce() {
    Body = NULL;

    Qf = new ChMatrixDynamic<double>(BODY_QDOF, 1);
    vpoint = VNULL;
    vrelpoint = VNULL;
    force = VNULL;
    relforce = VNULL;
    vdir = VECT_X;
    vreldir = VECT_X;
    restpos = VNULL;
    mforce = 0;
    modula = std::make_shared<ChFunction_Const>(1);

    align = FDIR_BODY;
    frame = FPOS_BODY;
    mode = FTYPE_FORCE;

    move_x = std::make_shared<ChFunction_Const>(0);
    move_y = std::make_shared<ChFunction_Const>(0);
    move_z = std::make_shared<ChFunction_Const>(0);
    f_x = std::make_shared<ChFunction_Const>(0);
    f_y = std::make_shared<ChFunction_Const>(0);
    f_z = std::make_shared<ChFunction_Const>(0);

    ChTime = 0;
}

ChForce::~ChForce() {
    delete Qf;
}

void ChForce::Copy(ChForce* source) {
    // first copy the parent class data...
    ChObj::Copy(source);

    Body = source->Body;

    mforce = source->mforce;
    force = source->force;
    relforce = source->relforce;
    vdir = source->vdir;
    vreldir = source->vreldir;
    vpoint = source->vpoint;
    vrelpoint = source->vrelpoint;
    restpos = source->restpos;
    align = source->align;
    frame = source->frame;
    mode = source->mode;

    ChTime = source->ChTime;

    Qf->CopyFromMatrix(*source->Qf);

 
    modula = std::shared_ptr<ChFunction>(source->modula->new_Duplicate());
    
    move_x = std::shared_ptr<ChFunction>(source->move_x->new_Duplicate());
    move_y = std::shared_ptr<ChFunction>(source->move_y->new_Duplicate());
    move_z = std::shared_ptr<ChFunction>(source->move_z->new_Duplicate());
    
    f_x = std::shared_ptr<ChFunction>(source->f_x->new_Duplicate());
    f_y = std::shared_ptr<ChFunction>(source->f_y->new_Duplicate());
    f_z = std::shared_ptr<ChFunction>(source->f_z->new_Duplicate());
}

////// Impose absolute or relative positions, also
//     setting the correct "rest position".

void ChForce::SetVpoint(Vector mypoint) {
    // abs pos
    vpoint = mypoint;
    // rel pos
    vrelpoint = GetBody()->Point_World2Body(vpoint);
    // computes initial rest position.
    Vector displace = VNULL;
    if (move_x)
        displace.x = move_x->Get_y(ChTime);
    if (move_y)
        displace.y = move_y->Get_y(ChTime);
    if (move_z)
        displace.z = move_z->Get_y(ChTime);
    if (frame == FPOS_WORLD)
        restpos = Vsub(vpoint, displace);
    if (frame == FPOS_BODY)
        restpos = Vsub(vrelpoint, displace);
}

void ChForce::SetVrelpoint(Vector myrelpoint) {
    // rel pos
    vrelpoint = myrelpoint;
    // abs pos
    vpoint = GetBody()->Point_Body2World(vrelpoint);
    // computes initial rest position.
    Vector displace = VNULL;
    if (move_x)
        displace.x = move_x->Get_y(ChTime);
    if (move_y)
        displace.y = move_y->Get_y(ChTime);
    if (move_z)
        displace.z = move_z->Get_y(ChTime);
    if (frame == FPOS_WORLD)
        restpos = Vsub(vpoint, displace);
    if (frame == FPOS_BODY)
        restpos = Vsub(vrelpoint, displace);
}

////// Impose absolute or relative force directions

void ChForce::SetDir(Vector newf) {
    vdir = Vnorm(newf);
    vreldir = GetBody()->TransformDirectionParentToLocal(vdir);
    UpdateState();  // update also F
}

void ChForce::SetRelDir(Vector newf) {
    vreldir = Vnorm(newf);
    vdir = GetBody()->TransformDirectionLocalToParent(vreldir);
    UpdateState();  // update also F
}

////// Impose module

void ChForce::SetMforce(double newf) {
    mforce = newf;
    UpdateState();  // update also F
}

////// force as applied to body
void ChForce::GetBodyForceTorque(Vector* body_force, Vector* body_torque) {
    ChMatrix33<> Xpos;

    switch (mode) {
        case FTYPE_FORCE:
            *body_force = this->force;  // Fb = F.w

            Xpos.Set_X_matrix(this->vrelpoint);
            *body_torque = Xpos.MatrT_x_Vect(this->relforce);
            *body_torque = Vmul(*body_torque, -1.0);  // Mb = - [u]'[A]'F,w   = - [u]'F,l
            break;

        case FTYPE_TORQUE:
            *body_force = VNULL;      // Fb = 0;
            *body_torque = relforce;  // Mb = [A]'F,w   = F,l
            break;

        default:
            break;
    }
}

//////
////// Updating
//////

void ChForce::UpdateTime(double mytime) {
    ChTime = mytime;

    //... put time-sensitive stuff here..
}

void ChForce::UpdateState() {
    ChBody* my_body;
    double modforce;
    Vector vectforce;
    Vector vmotion;
    Vector xyzforce;
    ChMatrixNM<double, 3, 1> mat_force;
    ChMatrix33<> Xpos;
    ChMatrixNM<double, 4, 1> Qfrot;

    my_body = GetBody();

    // ====== Update the position of point of application

    vmotion = VNULL;
    if (move_x)
        vmotion.x = move_x->Get_y(ChTime);
    if (move_y)
        vmotion.y = move_y->Get_y(ChTime);
    if (move_z)
        vmotion.z = move_z->Get_y(ChTime);

    switch (frame) {
        case FPOS_WORLD:
            vpoint = Vadd(restpos, vmotion);                // Uw
            vrelpoint = my_body->Point_World2Body(vpoint);  // Uo1 = [A]'(Uw-Xo1)
            break;
        case FPOS_BODY:
            vrelpoint = Vadd(restpos, vmotion);             // Uo1
            vpoint = my_body->Point_Body2World(vrelpoint);  // Uw = Xo1+[A]Uo1
            break;
    }

    // ====== Update the fm force vector and add fv

    modforce = mforce * modula->Get_y(ChTime);

    vectforce = VNULL;
    xyzforce = VNULL;
    if (f_x)
        xyzforce.x = f_x->Get_y(ChTime);
    if (f_y)
        xyzforce.y = f_y->Get_y(ChTime);
    if (f_z)
        xyzforce.z = f_z->Get_y(ChTime);

    switch (align) {
        case FDIR_WORLD:
            vreldir = my_body->TransformDirectionParentToLocal(vdir);
            vectforce = Vmul(vdir, modforce);
            vectforce = Vadd(vectforce, xyzforce);
            break;
        case FDIR_BODY:
            vdir = my_body->TransformDirectionLocalToParent(vreldir);
            vectforce = Vmul(vdir, modforce);
            xyzforce = my_body->TransformDirectionLocalToParent(xyzforce);
            vectforce = Vadd(vectforce, xyzforce);
            break;
    }

    force = vectforce;                                           // Fw
    relforce = my_body->TransformDirectionParentToLocal(force);  // Fo1 = [A]'Fw

    // ====== Update the Qc lagrangian!

    switch (mode) {
        case FTYPE_FORCE: {
            Qf->SetElement(0, 0, force.x);  // pos.lagrangian Qfx
            Qf->SetElement(1, 0, force.y);
            Qf->SetElement(2, 0, force.z);
            //   Qfrot= (-[A][u][G])'f
            Vector VQtemp;

            Xpos.Set_X_matrix(vrelpoint);

            VQtemp = Xpos.MatrT_x_Vect(relforce);  // = [u]'[A]'F,w

            mat_force.PasteVector(VQtemp, 0, 0);

            ChMatrixNM<double, 3, 4> mGl;
            ChFrame<>::SetMatrix_Gl(mGl, my_body->GetCoord().rot);

            Qfrot.MatrTMultiply(mGl, mat_force);
            Qfrot.MatrNeg();  // Q = - [Gl]'[u]'[A]'F,w
            Qf->PasteMatrix(&Qfrot, 3, 0);
            break;
        }

        case FTYPE_TORQUE:
            Qf->SetElement(0, 0, 0);  // pos.lagrangian Qfx
            Qf->SetElement(1, 0, 0);
            Qf->SetElement(2, 0, 0);

            // rot.lagangian
            mat_force.PasteVector(relforce, 0, 0);

            ChMatrixNM<double, 3, 4> mGl;
            ChFrame<>::SetMatrix_Gl(mGl, my_body->GetCoord().rot);

            Qfrot.MatrTMultiply(mGl, mat_force);
            Qf->PasteMatrix(&Qfrot, 3, 0);

            break;
    }
}

void ChForce::Update(double mytime) {
    UpdateTime(mytime);
    UpdateState();
}

////// File  I/O



void ChForce::ArchiveOUT(ChArchiveOut& marchive) {

    // class version number
    marchive.VersionWrite(1);

    // serialize parent class too
    ChObj::ArchiveOUT(marchive);

    // stream out all member data
    marchive << CHNVP(mode);
    marchive << CHNVP(frame);
    marchive << CHNVP(align);
    marchive << CHNVP(vrelpoint);
    marchive << CHNVP(vpoint);
    marchive << CHNVP(move_x);
    marchive << CHNVP(move_y);
    marchive << CHNVP(move_z);
    marchive << CHNVP(restpos);
    marchive << CHNVP(f_x);
    marchive << CHNVP(f_y);
    marchive << CHNVP(f_z);
    marchive << CHNVP(mforce);
    marchive << CHNVP(modula,"f_time");
    marchive << CHNVP(vreldir);
    marchive << CHNVP(vdir);
}

void ChForce::ArchiveIN(ChArchiveIn& marchive) {
    // class version number
    int version = marchive.VersionRead();
    // deserialize parent class too
    ChObj::ArchiveIN(marchive);

    // stream in all member data
    marchive >> CHNVP(mode);
    marchive >> CHNVP(frame);
    marchive >> CHNVP(align);
    marchive >> CHNVP(vrelpoint);
    marchive >> CHNVP(vpoint);
    marchive >> CHNVP(move_x);
    marchive >> CHNVP(move_y);
    marchive >> CHNVP(move_z);
    marchive >> CHNVP(restpos);
    marchive >> CHNVP(f_x);
    marchive >> CHNVP(f_y);
    marchive >> CHNVP(f_z);
    marchive >> CHNVP(mforce);
    marchive >> CHNVP(modula,"f_time");
    marchive >> CHNVP(vreldir);
    marchive >> CHNVP(vdir);
}



}  // END_OF_NAMESPACE____

///// eof
