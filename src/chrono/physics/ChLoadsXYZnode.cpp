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
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono/physics/ChLoadsXYZnode.h"

namespace chrono {

// -----------------------------------------------------------------------------
// ChLoadXYZnodeForce
// -----------------------------------------------------------------------------

ChLoadXYZnodeForce::ChLoadXYZnodeForce(std::shared_ptr<ChNodeXYZ> body)
    : ChLoadCustom(body)
{
	computed_abs_force = VNULL;
}

void ChLoadXYZnodeForce::ComputeQ(ChState* state_x, ChStateDelta* state_w) {

	auto mnode = std::dynamic_pointer_cast<ChNodeXYZ>(this->loadable);

	ChVector<> nodeApos;
	ChVector<> nodeApos_dt;

	if (state_x) {
        // the numerical jacobian algo might change state_x
        nodeApos = (state_x->ClipVector(0, 0));
    } else {
		nodeApos = mnode->GetPos();
    }

    if (state_w) {
        // the numerical jacobian algo might change state_w
		nodeApos_dt = (state_w->ClipVector(0, 0));
    } else {
        nodeApos_dt = mnode->GetPos_dt();
    }

	ComputeForce(nodeApos, nodeApos_dt, computed_abs_force);

    // Compute Q 
	this->load_Q.PasteVector(computed_abs_force, 0, 0);
}

void ChLoadXYZnodeForce::Update(double time) {
    ChLoadCustom::Update(time);
}



// -----------------------------------------------------------------------------
// ChLoadXYZnodeForceAbsolute
// -----------------------------------------------------------------------------

ChLoadXYZnodeForceAbsolute::ChLoadXYZnodeForceAbsolute(std::shared_ptr<ChNodeXYZ> body,
                                 const ChVector<>& force)
    : ChLoadXYZnodeForce(body),
      m_force_base(force),
      m_scale(1) {
    m_modulation = std::make_shared<ChFunction_Const>(1.0);
}

/// Compute the force on the node, in absolute coordsystem,
/// given position of node as abs_pos.
void ChLoadXYZnodeForceAbsolute::ComputeForce(const ChVector<>& abs_pos,
													 const ChVector<>& abs_vel,
													 ChVector<>& abs_force) {
	abs_force = GetForce();
}

void ChLoadXYZnodeForceAbsolute::Update(double time) {
    m_modulation->Update(time);
    m_scale = m_modulation->Get_y(time);
    ChLoadXYZnodeForce::Update(time);
}

void ChLoadXYZnodeForceAbsolute::SetForceBase(const ChVector<>& force) {
    m_force_base = force;
}

ChVector<> ChLoadXYZnodeForceAbsolute::GetForce() const {
    return m_force_base * m_scale;
}



// -----------------------------------------------------------------------------
// ChLoadXYZnodeXYZnode
// -----------------------------------------------------------------------------

ChLoadXYZnodeXYZnode::ChLoadXYZnodeXYZnode(std::shared_ptr<ChNodeXYZ> mnodeA,  
										   std::shared_ptr<ChNodeXYZ> mnodeB)
    : ChLoadCustomMultiple(mnodeA, mnodeB)
{
	computed_abs_force = VNULL;
}

void ChLoadXYZnodeXYZnode::ComputeQ(ChState* state_x, ChStateDelta* state_w) {

	auto mnodeA = std::dynamic_pointer_cast<ChNodeXYZ>(this->loadables[0]);
	auto mnodeB = std::dynamic_pointer_cast<ChNodeXYZ>(this->loadables[1]);

	ChVector<> nodeApos;
	ChVector<> nodeApos_dt;
	ChVector<> nodeBpos;
	ChVector<> nodeBpos_dt;

	if (state_x) {
        // the numerical jacobian algo might change state_x
        nodeApos = (state_x->ClipVector(0, 0));
		nodeBpos = (state_x->ClipVector(3, 0));
    } else {
		nodeApos = mnodeA->GetPos();
		nodeBpos = mnodeB->GetPos();
    }

    if (state_w) {
        // the numerical jacobian algo might change state_w
		nodeApos_dt = (state_w->ClipVector(0, 0));
		nodeBpos_dt = (state_w->ClipVector(3, 0));
    } else {
        nodeApos_dt = mnodeA->GetPos_dt();
		nodeBpos_dt = mnodeB->GetPos_dt();
    }

	ComputeForce((nodeApos-nodeBpos), (nodeApos_dt-nodeBpos_dt), computed_abs_force);

    // Compute Q 
	this->load_Q.PasteVector( computed_abs_force, 0, 0);
	this->load_Q.PasteVector(-computed_abs_force, 3, 0);
}

void ChLoadXYZnodeXYZnode::Update(double time) {
    ChLoadCustomMultiple::Update(time);
}


// -----------------------------------------------------------------------------
// ChLoadXYZnodeXYZnodeSpring
// -----------------------------------------------------------------------------

ChLoadXYZnodeXYZnodeSpring::ChLoadXYZnodeXYZnodeSpring(std::shared_ptr<ChNodeXYZ> mnodeA,  ///< node to apply load to
								  std::shared_ptr<ChNodeXYZ> mnodeB,   ///< node to apply load to as reaction
								  double mK,	///< stiffness,
								  double mR,	///< damping,
								  double md_0)	///< initial rest length
    : ChLoadXYZnodeXYZnode(mnodeA, mnodeB),
     K(mK),R(mR),d_0(md_0)
     {
	is_stiff = false;
}

/// Compute the force on the node, in absolute coordsystem,
/// given position of node as abs_pos.
void ChLoadXYZnodeXYZnodeSpring::ComputeForce(const ChVector<>& rel_pos,
													 const ChVector<>& rel_vel,
													 ChVector<>& abs_force) {
		ChVector<> BA = rel_pos.GetNormalized();
		double d = rel_pos.Length() - d_0;
		double d_dt = Vdot(rel_vel, BA);
		abs_force = (-K * d - R * d_dt) * BA;
}



// -----------------------------------------------------------------------------
// ChLoadXYZnodeXYZnodeBushing
// -----------------------------------------------------------------------------

ChLoadXYZnodeXYZnodeBushing::ChLoadXYZnodeXYZnodeBushing(std::shared_ptr<ChNodeXYZ> mnodeA,  ///< node to apply load to
								  std::shared_ptr<ChNodeXYZ> mnodeB   ///< node to apply load to as reaction
								  )	
    : ChLoadXYZnodeXYZnode(mnodeA, mnodeB)
     {
	force_dX = std::make_shared<ChFunction_Const>(0.0);
	force_dY = std::make_shared<ChFunction_Const>(0.0);
	force_dZ = std::make_shared<ChFunction_Const>(0.0);
	R = VNULL;
	is_stiff = false;
}

/// Compute the force on the node, in absolute coordsystem,
/// given position of node as abs_pos.
void ChLoadXYZnodeXYZnodeBushing::ComputeForce(const ChVector<>& rel_pos,
													 const ChVector<>& rel_vel,
													 ChVector<>& abs_force) {
	abs_force = ChVector<>(
		force_dX->Get_y(rel_pos.x()) - R.x() * rel_vel.x(),
		force_dY->Get_y(rel_pos.y()) - R.y() * rel_vel.y(),
		force_dZ->Get_y(rel_pos.z()) - R.z() * rel_vel.z());
}



// -----------------------------------------------------------------------------
// ChLoadBodyBody
// -----------------------------------------------------------------------------

ChLoadXYZnodeBody::ChLoadXYZnodeBody(std::shared_ptr<ChNodeXYZ> mnodeA,
									 std::shared_ptr<ChBody> mbodyB)
					: ChLoadCustomMultiple(mnodeA, mbodyB) {
	ChFrame<> abs_application(mnodeA->GetPos());
    mbodyB->ChFrame::TransformParentToLocal(abs_application, loc_application_B);
	computed_loc_force = VNULL;
}

void ChLoadXYZnodeBody::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
    auto mnodeA = std::dynamic_pointer_cast<ChNodeXYZ>(this->loadables[0]);
    auto mbodyB = std::dynamic_pointer_cast<ChBody>(this->loadables[1]);

    ChFrameMoving<> bodycoordA, bodycoordB;
    if (state_x) {
        // the numerical jacobian algo might change state_x
        bodycoordA.SetPos(state_x->ClipVector(0, 0));
        bodycoordB.SetCoord(state_x->ClipCoordsys(3, 0));
    } else {
        bodycoordA.SetPos(mnodeA->pos);
        bodycoordB.SetCoord(mbodyB->coord);
    }

    if (state_w) {
        // the numerical jacobian algo might change state_w
        bodycoordA.SetPos_dt(state_w->ClipVector(0, 0));
        bodycoordB.SetPos_dt(state_w->ClipVector(3, 0));
        bodycoordB.SetWvel_loc(state_w->ClipVector(6, 0));
    } else {
        bodycoordA.SetPos_dt(mnodeA->GetPos_dt());
        bodycoordB.SetCoord_dt(mbodyB->GetCoord_dt());
    }

    frame_Aw = bodycoordA;
    frame_Bw = ChFrameMoving<>(loc_application_B) >> bodycoordB;
    ChFrameMoving<> rel_AB = frame_Aw >> frame_Bw.GetInverse();

    // COMPUTE THE FORCE

    ComputeForce(rel_AB, computed_loc_force);

    ChVector<> abs_force = frame_Bw.TransformDirectionLocalToParent(computed_loc_force);

    // Compute Q
    this->load_Q.PasteVector(abs_force, 0, 0);

    ChVector<> loc_ftorque = bodycoordB.GetRot().RotateBack(((frame_Bw.GetPos() - bodycoordB.GetPos()) % abs_force));
    this->load_Q.PasteVector(-abs_force, 3, 0);
    this->load_Q.PasteVector(-loc_ftorque, 6, 0);
}

std::shared_ptr<ChNodeXYZ> ChLoadXYZnodeBody::GetNodeA() const {
    return std::dynamic_pointer_cast<ChNodeXYZ>(this->loadables[0]);
}

std::shared_ptr<ChBody> ChLoadXYZnodeBody::GetBodyB() const {
    return std::dynamic_pointer_cast<ChBody>(this->loadables[1]);
}



// -----------------------------------------------------------------------------
// ChLoadXYZnodeBodySpring
// -----------------------------------------------------------------------------

ChLoadXYZnodeBodySpring::ChLoadXYZnodeBodySpring(std::shared_ptr<ChNodeXYZ> mnodeA,  ///< node to apply load to
								  std::shared_ptr<ChBody> mbodyB,   ///< node to apply load to as reaction
								  double mK,	///< stiffness,
								  double mR,	///< damping,
								  double md_0)	///< initial rest length
    : ChLoadXYZnodeBody(mnodeA, mbodyB),
     K(mK),R(mR),d_0(md_0)
     {
	is_stiff = false;
}

/// Compute the force on the node, in absolute coordsystem,
/// given position of node as abs_pos.
void ChLoadXYZnodeBodySpring:: ComputeForce(const ChFrameMoving<>& rel_AB,
                               ChVector<>& loc_force) {
		ChVector<> BA = rel_AB.GetPos().GetNormalized();
		double d = rel_AB.GetPos().Length() - d_0;
		double d_dt = Vdot(rel_AB.GetPos_dt(), BA);
		loc_force = (-K * d - R * d_dt) * BA;
}



// -----------------------------------------------------------------------------
// ChLoadXYZnodeBodyBushing
// -----------------------------------------------------------------------------

ChLoadXYZnodeBodyBushing::ChLoadXYZnodeBodyBushing(std::shared_ptr<ChNodeXYZ> mnodeA,  ///< node to apply load to
								  std::shared_ptr<ChBody> mbodyB   ///< node to apply load to as reaction
								  )	
    : ChLoadXYZnodeBody(mnodeA, mbodyB)
     {
	force_dX = std::make_shared<ChFunction_Const>(0.0);
	force_dY = std::make_shared<ChFunction_Const>(0.0);
	force_dZ = std::make_shared<ChFunction_Const>(0.0);
	R = VNULL;
	is_stiff = false;
}

/// Compute the force on the node, in absolute coordsystem,
/// given position of node as abs_pos.
void ChLoadXYZnodeBodyBushing::ComputeForce(
								const ChFrameMoving<>& rel_AB,
								ChVector<>& loc_force) {
	loc_force = ChVector<>(
		force_dX->Get_y(rel_AB.GetPos().x()) - R.x()*rel_AB.GetPos_dt().x(),
		force_dY->Get_y(rel_AB.GetPos().y()) - R.y()*rel_AB.GetPos_dt().y(),
		force_dZ->Get_y(rel_AB.GetPos().z()) - R.z()*rel_AB.GetPos_dt().z() );
}




}  // end namespace chrono
