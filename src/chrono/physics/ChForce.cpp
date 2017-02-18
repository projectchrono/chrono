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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChForce.h"
#include "chrono/physics/ChGlobal.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChForce)

ChForce::ChForce()
    : Body(NULL),
      vpoint(VNULL),
      vrelpoint(VNULL),
      force(VNULL),
      relforce(VNULL),
      vdir(VECT_X),
      vreldir(VECT_X),
      restpos(VNULL),
      mforce(0),
      align(BODY_DIR),
      frame(BODY),
      mode(FORCE) {
    Qf = new ChMatrixDynamic<double>(7, 1);

    modula = std::make_shared<ChFunction_Const>(1);
    move_x = std::make_shared<ChFunction_Const>(0);
    move_y = std::make_shared<ChFunction_Const>(0);
    move_z = std::make_shared<ChFunction_Const>(0);
    f_x = std::make_shared<ChFunction_Const>(0);
    f_y = std::make_shared<ChFunction_Const>(0);
    f_z = std::make_shared<ChFunction_Const>(0);
}

ChForce::ChForce(const ChForce& other) : ChObj(other) {
    Body = other.Body;

    mforce = other.mforce;
    force = other.force;
    relforce = other.relforce;
    vdir = other.vdir;
    vreldir = other.vreldir;
    vpoint = other.vpoint;
    vrelpoint = other.vrelpoint;
    restpos = other.restpos;
    align = other.align;
    frame = other.frame;
    mode = other.mode;

    ChTime = other.ChTime;

    Qf = new ChMatrixDynamic<double>(7, 1);
    Qf->CopyFromMatrix(*other.Qf);

    modula = std::shared_ptr<ChFunction>(other.modula->Clone());

    move_x = std::shared_ptr<ChFunction>(other.move_x->Clone());
    move_y = std::shared_ptr<ChFunction>(other.move_y->Clone());
    move_z = std::shared_ptr<ChFunction>(other.move_z->Clone());

    f_x = std::shared_ptr<ChFunction>(other.f_x->Clone());
    f_y = std::shared_ptr<ChFunction>(other.f_y->Clone());
    f_z = std::shared_ptr<ChFunction>(other.f_z->Clone());
}

ChForce::~ChForce() {
    delete Qf;
}

// Impose absolute or relative positions, also setting the correct "rest position".
void ChForce::SetVpoint(ChVector<> mypoint) {
    // abs pos
    vpoint = mypoint;
    // rel pos
    vrelpoint = GetBody()->Point_World2Body(vpoint);

    // computes initial rest position.
    ChVector<> displace = VNULL;
    if (move_x)
        displace.x() = move_x->Get_y(ChTime);
    if (move_y)
        displace.y() = move_y->Get_y(ChTime);
    if (move_z)
        displace.z() = move_z->Get_y(ChTime);

    switch (frame) {
        case WORLD:
            restpos = Vsub(vpoint, displace);
            break;
        case BODY:
            restpos = Vsub(vrelpoint, displace);
            break;
    }
}

void ChForce::SetVrelpoint(ChVector<> myrelpoint) {
    // rel pos
    vrelpoint = myrelpoint;
    // abs pos
    vpoint = GetBody()->Point_Body2World(vrelpoint);

    // computes initial rest position.
    ChVector<> displace = VNULL;
    if (move_x)
        displace.x() = move_x->Get_y(ChTime);
    if (move_y)
        displace.y() = move_y->Get_y(ChTime);
    if (move_z)
        displace.z() = move_z->Get_y(ChTime);

    switch (frame) {
        case WORLD:
            restpos = Vsub(vpoint, displace);
            break;
        case BODY:
            restpos = Vsub(vrelpoint, displace);
            break;
    }
}

// Impose absolute force directions
void ChForce::SetDir(ChVector<> newf) {
    vdir = Vnorm(newf);
    vreldir = GetBody()->TransformDirectionParentToLocal(vdir);
    UpdateState();  // update also F
}

// Impose relative force directions
void ChForce::SetRelDir(ChVector<> newf) {
    vreldir = Vnorm(newf);
    vdir = GetBody()->TransformDirectionLocalToParent(vreldir);
    UpdateState();  // update also F
}

// Impose module
void ChForce::SetMforce(double newf) {
    mforce = newf;
    UpdateState();  // update also F
}

// Force as applied to body
void ChForce::GetBodyForceTorque(ChVector<>& body_force, ChVector<>& body_torque) const {
    ChMatrix33<> Xpos;

    switch (mode) {
        case FORCE:
            body_force = this->force;  // Fb = F.w

            Xpos.Set_X_matrix(this->vrelpoint);
            body_torque = Xpos.MatrT_x_Vect(this->relforce);
            body_torque = Vmul(body_torque, -1.0);  // Mb = - [u]'[A]'F,w   = - [u]'F,l
            break;

        case TORQUE:
            body_force = VNULL;      // Fb = 0;
            body_torque = relforce;  // Mb = [A]'F,w   = F,l
            break;

        default:
            break;
    }
}

// Updating

void ChForce::UpdateTime(double mytime) {
    ChTime = mytime;

    //... put time-dependent stuff here..
}

void ChForce::UpdateState() {
    ChBody* my_body;
    double modforce;
    ChVector<> vectforce;
    ChVector<> vmotion;
    ChVector<> xyzforce;
    ChMatrixNM<double, 3, 1> mat_force;
    ChMatrix33<> Xpos;
    ChMatrixNM<double, 4, 1> Qfrot;

    my_body = GetBody();

    // ====== Update the position of point of application

    vmotion = VNULL;
    if (move_x)
        vmotion.x() = move_x->Get_y(ChTime);
    if (move_y)
        vmotion.y() = move_y->Get_y(ChTime);
    if (move_z)
        vmotion.z() = move_z->Get_y(ChTime);

    switch (frame) {
        case WORLD:
            vpoint = Vadd(restpos, vmotion);                // Uw
            vrelpoint = my_body->Point_World2Body(vpoint);  // Uo1 = [A]'(Uw-Xo1)
            break;
        case BODY:
            vrelpoint = Vadd(restpos, vmotion);             // Uo1
            vpoint = my_body->Point_Body2World(vrelpoint);  // Uw = Xo1+[A]Uo1
            break;
    }

    // ====== Update the fm force vector and add fv

    modforce = mforce * modula->Get_y(ChTime);

    vectforce = VNULL;
    xyzforce = VNULL;
    if (f_x)
        xyzforce.x() = f_x->Get_y(ChTime);
    if (f_y)
        xyzforce.y() = f_y->Get_y(ChTime);
    if (f_z)
        xyzforce.z() = f_z->Get_y(ChTime);

    switch (align) {
        case WORLD_DIR:
            vreldir = my_body->TransformDirectionParentToLocal(vdir);
            vectforce = Vmul(vdir, modforce);
            vectforce = Vadd(vectforce, xyzforce);
            break;
        case BODY_DIR:
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
        case FORCE: {
            Qf->SetElement(0, 0, force.x());  // pos.lagrangian Qfx
            Qf->SetElement(1, 0, force.y());
            Qf->SetElement(2, 0, force.z());
            //   Qfrot= (-[A][u][G])'f
            ChVector<> VQtemp;

            Xpos.Set_X_matrix(vrelpoint);

            VQtemp = Xpos.MatrT_x_Vect(relforce);  // = [u]'[A]'F,w

            mat_force.PasteVector(VQtemp, 0, 0);

            ChMatrixNM<double, 3, 4> mGl;
            ChFrame<>::SetMatrix_Gl(mGl, my_body->GetCoord().rot);

            Qfrot.MatrTMultiply(mGl, mat_force);
            Qfrot.MatrNeg();  // Q = - [Gl]'[u]'[A]'F,w
            Qf->PasteMatrix(Qfrot, 3, 0);
            break;
        }

        case TORQUE:
            Qf->SetElement(0, 0, 0);  // pos.lagrangian Qfx
            Qf->SetElement(1, 0, 0);
            Qf->SetElement(2, 0, 0);

            // rot.lagangian
            mat_force.PasteVector(relforce, 0, 0);

            ChMatrixNM<double, 3, 4> mGl;
            ChFrame<>::SetMatrix_Gl(mGl, my_body->GetCoord().rot);

            Qfrot.MatrTMultiply(mGl, mat_force);
            Qf->PasteMatrix(Qfrot, 3, 0);

            break;
    }
}

void ChForce::Update(double mytime) {
    UpdateTime(mytime);
    UpdateState();
}

// File  I/O

void ChForce::ArchiveOUT(ChArchiveOut& marchive) {
    // class version number
    marchive.VersionWrite<ChForce>();

    // serialize parent class too
    ChObj::ArchiveOUT(marchive);

    // stream out all member data

    ForceType_mapper ftypemapper;
    marchive << CHNVP(ftypemapper(mode), "force_type");
    ReferenceFrame_mapper refmapper;
    marchive << CHNVP(refmapper(frame), "reference_frame_type");
    AlignmentFrame_mapper alignmapper;
    marchive << CHNVP(alignmapper(align), "alignment_frame_type");

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
    marchive << CHNVP(modula, "f_time");
    marchive << CHNVP(vreldir);
    marchive << CHNVP(vdir);
}

void ChForce::ArchiveIN(ChArchiveIn& marchive) {
    // class version number
    int version = marchive.VersionRead<ChForce>();

    // deserialize parent class too
    ChObj::ArchiveIN(marchive);

    // stream in all member data

    ForceType_mapper ftypemapper;
    marchive >> CHNVP(ftypemapper(mode), "force_type");
    ReferenceFrame_mapper refmapper;
    marchive >> CHNVP(refmapper(frame), "reference_frame_type");
    AlignmentFrame_mapper alignmapper;
    marchive >> CHNVP(alignmapper(align), "alignment_frame_type");

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
    marchive >> CHNVP(modula, "f_time");
    marchive >> CHNVP(vreldir);
    marchive >> CHNVP(vdir);
}

}  // end namespace chrono