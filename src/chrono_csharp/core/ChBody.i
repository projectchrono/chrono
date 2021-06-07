// ============================================================
// MULTIPLE INHERITANCE WORKAROUND

// (A) Methods inherited from base classes that SWIG discards
//     (i.e. methods that *are not* overriden in ChBody)

// First, ensure that these functions are not marked as 'overrides' in the generated C# code.

%csmethodmodifiers chrono::ChBody::SetPos "public"
%csmethodmodifiers chrono::ChBody::SetRot "public"
%csmethodmodifiers chrono::ChBody::GetPos "public"
%csmethodmodifiers chrono::ChBody::GetRot "public"
%csmethodmodifiers chrono::ChBody::GetA "public"

%csmethodmodifiers chrono::ChBody::SetPos_dt "public"
%csmethodmodifiers chrono::ChBody::SetRot_dt "public"
%csmethodmodifiers chrono::ChBody::SetWvel_loc "public"
%csmethodmodifiers chrono::ChBody::SetWvel_par "public"
%csmethodmodifiers chrono::ChBody::GetPos_dt "public"
%csmethodmodifiers chrono::ChBody::GetRot_dt "public"
%csmethodmodifiers chrono::ChBody::GetWvel_loc "public"
%csmethodmodifiers chrono::ChBody::GetWvel_par "public"
%csmethodmodifiers chrono::ChBody::GetPos_dtdt "public"
%csmethodmodifiers chrono::ChBody::GetRot_dtdt "public"
%csmethodmodifiers chrono::ChBody::GetWacc_loc "public"
%csmethodmodifiers chrono::ChBody::GetWacc_par "public"

// Second, extend ChBody with implementations of these functions

%extend chrono::ChBody 
{
// Methods inherited from ChFrame
void SetPos(const ChVector<double>& p)      {$self->SetPos(p);}
void SetRot(const ChQuaternion<double>& q)  {$self->SetRot(q);}
void SetRot(const ChMatrix33<double>& A)    {$self->SetRot(A);}
const ChVector<double>& GetPos() const      {return $self->GetPos();}
const ChQuaternion<double>& GetRot() const  {return $self->GetRot();}
const ChMatrix33<double>& GetA() const      {return $self->GetA();}

// Methods inherited from ChFrameMoving
void SetPos_dt(const ChVector<double>& pd)      {$self->SetPos_dt(pd);}
void SetRot_dt(const ChQuaternion<double>& qd)  {$self->SetRot_dt(qd);}
void SetWvel_loc(const ChVector<double>& wl)    {$self->SetWvel_loc(wl);}
void SetWvel_par(const ChVector<double>& wp)    {$self->SetWvel_par(wp);}
const ChVector<double>& GetPos_dt() const       {return $self->GetPos_dt();}
const ChQuaternion<double>& GetRot_dt() const   {return $self->GetRot_dt();}
ChVector<double> GetWvel_loc() const            {return $self->GetWvel_loc();}
ChVector<double> GetWvel_par() const            {return $self->GetWvel_par();}
const ChVector<double>& GetPos_dtdt() const     {return $self->GetPos_dtdt();}
const ChQuaternion<double>& GetRot_dtdt() const {return $self->GetRot_dtdt();}
ChVector<double> GetWacc_loc() const            {return $self->GetWacc_loc();}
ChVector<double> GetWacc_par() const            {return $self->GetWacc_par();}
};

// (B) Methods of a base class that SWIG discards that *are* overriden in ChBody

// Ensure that these functions are not marked as 'overrides' in the generated C# code.

%csmethodmodifiers chrono::ChBody::Variables "public"
%csmethodmodifiers chrono::ChBody::LoadableGetVariables "public"
%csmethodmodifiers chrono::ChBody::LoadableStateIncrement "public"
%csmethodmodifiers chrono::ChBody::LoadableGetStateBlock_x "public"
%csmethodmodifiers chrono::ChBody::LoadableGetStateBlock_w "public"
%csmethodmodifiers chrono::ChBody::ComputeNF "public"

//// RADU:  Do we actually want to wrap methods of ChLoadable?
////        If not, we should probably just use %ignore

// ============================================================

// Include the C++ header(s)
%{
#include "chrono/physics/ChBody.h"
%}

%shared_ptr(chrono::ChBody)

// Forward ref
//%import "ChPhysicsItem.i"   // (parent class does not need %import if all .i are included in proper order
%import "chrono_csharp/core/ChMaterialSurface.i"
%import "chrono_csharp/core/ChCollisionModel.i"
%import "chrono_csharp/core/ChMarker.i"

/* Parse the header file to generate wrappers */
%include "../../chrono/physics/ChBody.h"  




