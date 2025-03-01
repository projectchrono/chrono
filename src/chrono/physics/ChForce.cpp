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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/core/ChGlobal.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChForce.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChForce)

ChForce::ChForce()
    : Body(nullptr),
      mode(FORCE),
      frame(BODY),
      align(BODY_DIR),
      vpoint(VNULL),
      vrelpoint(VNULL),
      restpos(VNULL),
      mforce(0),
      vdir(VECT_X),
      vreldir(VECT_X),
      force(VNULL),
      relforce(VNULL) {
    modula = chrono_types::make_shared<ChFunctionConst>(1);
    move_x = chrono_types::make_shared<ChFunctionConst>(0);
    move_y = chrono_types::make_shared<ChFunctionConst>(0);
    move_z = chrono_types::make_shared<ChFunctionConst>(0);
    f_x = chrono_types::make_shared<ChFunctionConst>(0);
    f_y = chrono_types::make_shared<ChFunctionConst>(0);
    f_z = chrono_types::make_shared<ChFunctionConst>(0);
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

    Qf = other.Qf;

    modula = std::shared_ptr<ChFunction>(other.modula->Clone());

    move_x = std::shared_ptr<ChFunction>(other.move_x->Clone());
    move_y = std::shared_ptr<ChFunction>(other.move_y->Clone());
    move_z = std::shared_ptr<ChFunction>(other.move_z->Clone());

    f_x = std::shared_ptr<ChFunction>(other.f_x->Clone());
    f_y = std::shared_ptr<ChFunction>(other.f_y->Clone());
    f_z = std::shared_ptr<ChFunction>(other.f_z->Clone());
}

// Impose absolute or relative positions, also setting the correct "rest position".
void ChForce::SetVpoint(ChVector3d mypoint) {
    // abs pos
    vpoint = mypoint;
    // rel pos
    vrelpoint = GetBody()->TransformPointParentToLocal(vpoint);

    // computes initial rest position.
    ChVector3d displace = VNULL;
    if (move_x)
        displace.x() = move_x->GetVal(ChTime);
    if (move_y)
        displace.y() = move_y->GetVal(ChTime);
    if (move_z)
        displace.z() = move_z->GetVal(ChTime);

    switch (frame) {
        case WORLD:
            restpos = Vsub(vpoint, displace);
            break;
        case BODY:
            restpos = Vsub(vrelpoint, displace);
            break;
    }
}

void ChForce::SetVrelpoint(ChVector3d myrelpoint) {
    // rel pos
    vrelpoint = myrelpoint;
    // abs pos
    vpoint = GetBody()->TransformPointLocalToParent(vrelpoint);

    // computes initial rest position.
    ChVector3d displace = VNULL;
    if (move_x)
        displace.x() = move_x->GetVal(ChTime);
    if (move_y)
        displace.y() = move_y->GetVal(ChTime);
    if (move_z)
        displace.z() = move_z->GetVal(ChTime);

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
void ChForce::SetDir(ChVector3d newf) {
    vdir = Vnorm(newf);
    vreldir = GetBody()->TransformDirectionParentToLocal(vdir);
    UpdateState();  // update also F
}

// Impose relative force directions
void ChForce::SetRelDir(ChVector3d newf) {
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
void ChForce::GetBodyForceTorque(ChVector3d& body_force, ChVector3d& body_torque) const {
    switch (mode) {
        case FORCE: {
            body_force = force;  // Fb = F.w
            ChStarMatrix33<> Xpos(vrelpoint);
            body_torque = -(Xpos.transpose() * relforce);  // Mb = - [u]'[A]'F,w   = - [u]'F,l
            break;
        }
        case TORQUE:
            body_force = VNULL;      // Fb = 0;
            body_torque = relforce;  // Mb = [A]'F,w   = F,l
            break;

        default:
            break;
    }
}

// Updating

void ChForce::UpdateState() {
    ChBody* my_body;
    double modforce;
    ChVector3d vectforce;
    ChVector3d vmotion;
    ChVector3d xyzforce;

    my_body = GetBody();

    // ====== Update the position of point of application

    vmotion = VNULL;
    if (move_x)
        vmotion.x() = move_x->GetVal(ChTime);
    if (move_y)
        vmotion.y() = move_y->GetVal(ChTime);
    if (move_z)
        vmotion.z() = move_z->GetVal(ChTime);

    switch (frame) {
        case WORLD:
            vpoint = Vadd(restpos, vmotion);                           // Uw
            vrelpoint = my_body->TransformPointParentToLocal(vpoint);  // Uo1 = [A]'(Uw-Xo1)
            break;
        case BODY:
            vrelpoint = Vadd(restpos, vmotion);                        // Uo1
            vpoint = my_body->TransformPointLocalToParent(vrelpoint);  // Uw = Xo1+[A]Uo1
            break;
    }

    // ====== Update the fm force vector and add fv

    modforce = mforce * modula->GetVal(ChTime);

    vectforce = VNULL;
    xyzforce = VNULL;
    if (f_x)
        xyzforce.x() = f_x->GetVal(ChTime);
    if (f_y)
        xyzforce.y() = f_y->GetVal(ChTime);
    if (f_z)
        xyzforce.z() = f_z->GetVal(ChTime);

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
            Qf(0) = force.x();  // pos.lagrangian Qfx
            Qf(1) = force.y();
            Qf(2) = force.z();

            //   Qfrot= (-[A][u][G])'f

            ChStarMatrix33<> Xpos(vrelpoint);
            ChVector3d VQtemp = Xpos.transpose() * relforce;  // = [u]'[A]'F,w

            ChGlMatrix34<> mGl(my_body->GetCoordsys().rot);
            ChVectorN<double, 4> Qfrot = -mGl.transpose() * VQtemp.eigen();  // Q = - [Gl]'[u]'[A]'F,w

            Qf.segment(3, 4) = Qfrot;

            break;
        }

        case TORQUE:
            Qf(0) = 0;  // pos.lagrangian Qfx
            Qf(1) = 0;
            Qf(2) = 0;

            // rot.lagangian
            ChGlMatrix34<> mGl(my_body->GetCoordsys().rot);
            ChVectorN<double, 4> Qfrot = mGl.transpose() * relforce.eigen();

            Qf.segment(3, 4) = Qfrot;

            break;
    }
}

void ChForce::Update(double time, bool update_assets) {
    ChObj::Update(time, update_assets);
    UpdateState();
}

// File  I/O

void ChForce::ArchiveOut(ChArchiveOut& archive_out) {
    // class version number
    archive_out.VersionWrite<ChForce>();

    // serialize parent class too
    ChObj::ArchiveOut(archive_out);

    // stream out all member data

    ForceType_mapper ftypemapper;
    archive_out << CHNVP(ftypemapper(mode), "force_type");
    ReferenceFrame_mapper refmapper;
    archive_out << CHNVP(refmapper(frame), "reference_frame_type");
    AlignmentFrame_mapper alignmapper;
    archive_out << CHNVP(alignmapper(align), "alignment_frame_type");

    archive_out << CHNVP(vrelpoint);
    archive_out << CHNVP(vpoint);
    archive_out << CHNVP(move_x);
    archive_out << CHNVP(move_y);
    archive_out << CHNVP(move_z);
    archive_out << CHNVP(restpos);
    archive_out << CHNVP(f_x);
    archive_out << CHNVP(f_y);
    archive_out << CHNVP(f_z);
    archive_out << CHNVP(mforce);
    archive_out << CHNVP(modula, "f_time");
    archive_out << CHNVP(vreldir);
    archive_out << CHNVP(vdir);
}

void ChForce::ArchiveIn(ChArchiveIn& archive_in) {
    // class version number
    /*int version =*/archive_in.VersionRead<ChForce>();

    // deserialize parent class too
    ChObj::ArchiveIn(archive_in);

    // stream in all member data

    ForceType_mapper ftypemapper;
    archive_in >> CHNVP(ftypemapper(mode), "force_type");
    ReferenceFrame_mapper refmapper;
    archive_in >> CHNVP(refmapper(frame), "reference_frame_type");
    AlignmentFrame_mapper alignmapper;
    archive_in >> CHNVP(alignmapper(align), "alignment_frame_type");

    archive_in >> CHNVP(vrelpoint);
    archive_in >> CHNVP(vpoint);
    archive_in >> CHNVP(move_x);
    archive_in >> CHNVP(move_y);
    archive_in >> CHNVP(move_z);
    archive_in >> CHNVP(restpos);
    archive_in >> CHNVP(f_x);
    archive_in >> CHNVP(f_y);
    archive_in >> CHNVP(f_z);
    archive_in >> CHNVP(mforce);
    archive_in >> CHNVP(modula, "f_time");
    archive_in >> CHNVP(vreldir);
    archive_in >> CHNVP(vdir);
}

}  // end namespace chrono