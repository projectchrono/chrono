#ifdef SWIGCSHARP  // --------------------------------------------------------------------- CSHARP

// MULTIPLE INHERITANCE WORKAROUND

// (A) Methods inherited from base classes that SWIG discards
//     (i.e. methods that *are not* overriden in ChBody)

// First, ensure that these functions are not marked as 'overrides' in the generated C# code.

%csmethodmodifiers chrono::ChBody::SetPos "public"
%csmethodmodifiers chrono::ChBody::SetRot "public"
%csmethodmodifiers chrono::ChBody::SetPosDer "public"
%csmethodmodifiers chrono::ChBody::SetLinVel "public"
%csmethodmodifiers chrono::ChBody::SetRotDer "public"
%csmethodmodifiers chrono::ChBody::SetAngVelLocal "public"
%csmethodmodifiers chrono::ChBody::SetAngVelParent "public"

%csmethodmodifiers chrono::ChBody::GetPhysicsItem "public"
%csmethodmodifiers chrono::ChBody::GetPos "public"
%csmethodmodifiers chrono::ChBody::GetRot "public"
%csmethodmodifiers chrono::ChBody::GetRotMat "public"
%csmethodmodifiers chrono::ChBody::GetPosDer "public"
%csmethodmodifiers chrono::ChBody::GetLinVel "public"
%csmethodmodifiers chrono::ChBody::GetRotDer "public"
%csmethodmodifiers chrono::ChBody::GetAngVelLocal "public"
%csmethodmodifiers chrono::ChBody::GetAngVelParent "public"
%csmethodmodifiers chrono::ChBody::GetPosDer2 "public"
%csmethodmodifiers chrono::ChBody::GetLinAcc "public"
%csmethodmodifiers chrono::ChBody::GetRotDer2 "public"
%csmethodmodifiers chrono::ChBody::GetAngAccLocal "public"
%csmethodmodifiers chrono::ChBody::GetAngAccParent "public"

// avoid adding new keyword
%csmethodmodifiers chrono::ChBody::AddCollisionModel "public"
%csmethodmodifiers chrono::ChBody::AddCollisionShape "public"

// Second, extend ChBody with implementations of these functions

%extend chrono::ChBody 
{
// Methods inherited from ChFrame
void SetPos(const ChVector3<double>& p)     {$self->SetPos(p);}
void SetRot(const ChQuaternion<double>& q)  {$self->SetRot(q);}
void SetRot(const ChMatrix33<double>& A)    {$self->SetRot(A);}
const ChVector3<double>& GetPos() const     {return $self->GetPos();}
const ChQuaternion<double>& GetRot() const  {return $self->GetRot();}
const ChMatrix33<double>& GetRotMat() const {return $self->GetRotMat();}

// Methods inherited from ChFrameMoving
void SetPosDer(const ChVector3<double>& pd)        {$self->SetPosDer(pd);}
void SetLinVel(const ChVector3<double>& pd)        {$self->SetLinVel(pd);}
void SetRotDer(const ChQuaternion<double>& qd)     {$self->SetRotDer(qd);}
void SetAngVelLocal(const ChVector3<double>& wl)   {$self->SetAngVelLocal(wl);}
void SetAngVelParent(const ChVector3<double>& wp)  {$self->SetAngVelParent(wp);}
const ChVector3<double>& GetPosDer() const         {return $self->GetPosDer();}
const ChVector3<double>& GetLinVel() const         {return $self->GetLinVel();}
const ChQuaternion<double>& GetRotDer() const      {return $self->GetRotDer();}
ChVector3<double> GetAngVelLocal() const           {return $self->GetAngVelLocal();}
ChVector3<double> GetAngVelParent() const          {return $self->GetAngVelParent();}
const ChVector3<double>& GetPosDer2() const        {return $self->GetPosDer2();}
const ChVector3<double>& GetLinAcc() const         {return $self->GetLinAcc();}
const ChQuaternion<double>& GetRotDer2() const     {return $self->GetRotDer2();}
ChVector3<double> GetAngAccLocal() const           {return $self->GetAngAccLocal();}
ChVector3<double> GetAngAccParent() const          {return $self->GetAngAccParent();}

// Methods inherited from ChContactable
void AddCollisionModel(std::shared_ptr<ChCollisionModel> model)                         {$self->AddCollisionModel(model);}
void AddCollisionShape(std::shared_ptr<ChCollisionShape> shape, const ChFrame<>& frame) {$self->AddCollisionShape(shape, frame);}
std::shared_ptr<ChCollisionModel> GetCollisionModel()                                   {return $self->GetCollisionModel();}
};

// (B) Methods of a base class that SWIG discards that *are* overriden in ChBody

// Ensure that these functions are not marked as 'overrides' in the generated C# code.

%csmethodmodifiers chrono::ChBody::Variables "public"
%csmethodmodifiers chrono::ChBody::LoadableGetVariables "public"
%csmethodmodifiers chrono::ChBody::LoadableStateIncrement "public"
%csmethodmodifiers chrono::ChBody::LoadableGetStateBlockPosLevel "public"
%csmethodmodifiers chrono::ChBody::LoadableGetStateBlockVelLevel "public"
%csmethodmodifiers chrono::ChBody::ComputeNF "public"

//// RADU:  Do we actually want to wrap methods of ChLoadable?
////        If not, we should probably just use %ignore

#endif             // --------------------------------------------------------------------- CSHARP

// Include the C++ header(s)
%{
#include "chrono/physics/ChContactable.h"
#include "chrono/physics/ChBody.h"
%}

%template(ChForceList) std::vector< std::shared_ptr<chrono::ChForce> >;
%template(ChMarkerList) std::vector< std::shared_ptr<chrono::ChMarker> >;
 
%shared_ptr(chrono::ChBody)

// Forward ref
//%import "ChPhysicsItem.i"   // (parent class does not need %import if all .i are included in proper order
%import "chrono_swig/interface/core/ChContactMaterial.i"
%import "chrono_swig/interface/core/ChCollisionModel.i"
%import "chrono_swig/interface/core/ChMarker.i"

// Parse the header file to generate wrappers
%include "../../../chrono/physics/ChContactable.h"  
%template(ChContactable1vars6) chrono::ChContactable_1vars<6>;
%include "../../../chrono/physics/ChBody.h"  




