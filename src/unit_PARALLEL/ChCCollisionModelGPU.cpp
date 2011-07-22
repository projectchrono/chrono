//////////////////////////////////////////////////
//  
//   ChCModelGPU.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com 
// ------------------------------------------------
///////////////////////////////////////////////////

#ifndef CH_NOCUDA 


#include "ChCCollisionModelGPU.h" 
#include "physics/ChBody.h"
#include "physics/ChSystem.h"

namespace chrono {
	namespace collision {

		ChCollisionModelGPU::ChCollisionModelGPU(){
			nObjects = 0;
			colFam = -1;
			noCollWith = -2;
		}

		ChCollisionModelGPU::~ChCollisionModelGPU(){
			//ClearModel(); not possible, would call GetPhysicsItem() that is pure virtual, enough to use instead..
			mData.clear();
		}
		int ChCollisionModelGPU::ClearModel(){
			if (GetPhysicsItem()->GetSystem()){
				if (GetPhysicsItem()->GetCollide())
				{
					GetPhysicsItem()->GetSystem()->GetCollisionSystem()->Remove(this);
				}
			}

			mData.clear();
			nObjects=0;
			colFam = -1;
			noCollWith = -2;
			return 1;
		} 


		int ChCollisionModelGPU::BuildModel(){
			if (GetPhysicsItem()->GetSystem())
				if (GetPhysicsItem()->GetCollide())
					GetPhysicsItem()->GetSystem()->GetCollisionSystem()->Add(this);

			return 1;
		}

		bool ChCollisionModelGPU::AddCompoundBody(int numSpheres, vector<float4> &data){
			model_type=COMPOUNDSPHERE;
			if(numSpheres<=0){
				return false;
			}
			nObjects = numSpheres;
			bData tData;
			for(int ii=0; ii<numSpheres; ii++){
				tData.B=make_float4(0);
				tData.C=make_float4(0);
				tData.A=data[ii];
				mData.push_back(tData);
			}
			return true;
		}

		bool ChCollisionModelGPU::AddSphere(double radius,  ChVector<>* pos){
			double mass=this->GetBody()->GetMass();
			this->GetBody()->SetInertiaXX(ChVector<>(2/5.0*mass*radius*radius,2/5.0*mass*radius*radius,2/5.0*mass*radius*radius));
			model_type=SPHERE;
			nObjects = 1;
			bData tData;
			tData.A=make_float4(0,0,0,radius);
			tData.B=make_float4(0,0,0,0);
			tData.C=make_float4(0,0,0,0);
			mData.push_back(tData);
			return true;
		}
		bool ChCollisionModelGPU::AddEllipsoid(double rx,  double ry,  double rz, ChVector<>* pos, ChMatrix33<>* rot){
			double mass=this->GetBody()->GetMass();
			this->GetBody()->SetInertiaXX(ChVector<>(1/5.0*mass*(ry*ry+rz*rz),1/5.0*mass*(rx*rx+rz*rz),1/5.0*mass*(rx*rx+ry*ry)));
			model_type=ELLIPSOID;
			nObjects = 1;
			bData tData;
			tData.A=make_float4(0,0,0,0);
			tData.B=make_float4(rx,ry,rz,0);
			tData.C=make_float4(0,0,0,0);
			mData.push_back(tData);
			return true;
		}
		bool ChCollisionModelGPU::AddBox(double hx, double hy, double hz, ChVector<>* pos, ChMatrix33<>* rot){
			double mass=this->GetBody()->GetMass();
			this->GetBody()->SetInertiaXX(ChVector<>(1/12.0*mass*(hy*hy+hz*hz),1/12.0*mass*(hx*hx+hz*hz),1/12.0*mass*(hx*hx+hy*hy)));
			model_type=BOX;
			nObjects = 1;
			bData tData;
			tData.A=make_float4(0,0,0,0);
			tData.B=make_float4(hx,hy,hz,0);
			tData.C=make_float4(0,0,0,0);
			mData.push_back(tData);
			return true;
		}
		bool ChCollisionModelGPU::AddTriangle(ChVector<> A, ChVector<> B, ChVector<> C, ChVector<>* pos, ChMatrix33<>* rot){
			model_type=TRIANGLEMESH;
			nObjects = 1;
			bData tData;
			tData.A=make_float4(A.x,A.y,A.z,-2);
			tData.B=make_float4(B.x,B.y,B.z,0);
			tData.C=make_float4(C.x,C.y,C.z,0);
			mData.push_back(tData);
			return true;
		}
		/// Add a cylinder to this model (default axis on Y direction), for collision purposes
		bool ChCollisionModelGPU::AddCylinder (double rx, double ry, double rz, ChVector<>* pos, ChMatrix33<>* rot){
			double mass=this->GetBody()->GetMass();
			this->GetBody()->SetInertiaXX(ChVector<>(1/12.0*mass*(3*rx*rx+ry*ry),1/2.0*mass*(rx*rx),1/12.0*mass*(3*rx*rx+ry*ry)));
			model_type=CYLINDER;
			nObjects = 1;
			bData tData;
			tData.A=make_float4(0,0,0,0);
			tData.B=make_float4(rx,ry,rz,0);
			tData.C=make_float4(0,0,0,0);
			mData.push_back(tData);
			return true;
		}

		bool ChCollisionModelGPU::AddConvexHull (std::vector<ChVector<double> >& pointlist, ChVector<>* pos, ChMatrix33<>* rot)
		{
			//NOT SUPPORTED
			return false;
		}
		bool ChCollisionModelGPU::AddBarrel (double Y_low, double Y_high, double R_vert, double R_hor, double R_offset, ChVector<>* pos, ChMatrix33<>* rot){
			//NOT SUPPORTED
			return false;
		}
		bool ChCollisionModelGPU::AddRectangle	(double rx, double ry){
			model_type=RECT;
			nObjects = 1;
			bData tData;
			tData.A=make_float4(0,0,0,0);
			tData.B=make_float4(rx,ry,0,0);
			tData.C=make_float4(0,0,0,0);
			mData.push_back(tData);
			return true;

		}
		bool ChCollisionModelGPU::AddDisc		(double rad){
			model_type=DISC;
			nObjects = 1;
			bData tData;
			tData.A=make_float4(0,0,0,0);
			tData.B=make_float4(rad,0,0,0);
			tData.C=make_float4(0,0,0,0);
			mData.push_back(tData);
			return true;
		}
		bool ChCollisionModelGPU::AddEllipse	(double rx, double ry){
			model_type=ELLIPSE;
			nObjects = 1;
			bData tData;
			tData.A=make_float4(0,0,0,0);
			tData.B=make_float4(rx,ry,0,0);
			tData.C=make_float4(0,0,0,0);
			mData.push_back(tData);
			return true;
		}
		bool ChCollisionModelGPU::AddCapsule	(double len, double rad){
			model_type=CAPSULE;
			nObjects = 1;
			bData tData;
			tData.A=make_float4(0,0,0,0);
			tData.B=make_float4(rad,len,0,0);
			tData.C=make_float4(0,0,0,0);
			mData.push_back(tData);
			return true;
		}
		bool ChCollisionModelGPU::AddCone		(double rad, double h){
			double mass=this->GetBody()->GetMass();
			this->GetBody()->SetInertiaXX(ChVector<>(3/5.0*mass*h*h+3/20.0*mass*rad*rad,3/5.0*mass*h*h+3/20.0*mass*rad*rad,3/10.0*mass*rad*rad));
			model_type=CONE;
			nObjects = 1;
			bData tData;
			tData.A=make_float4(0,0,0,0);
			tData.B=make_float4(rad,h,0,0);
			tData.C=make_float4(0,0,0,0);
			mData.push_back(tData);
			return true;
		}

		/// Add a triangle mesh to this model
		bool ChCollisionModelGPU::AddTriangleMesh (const  geometry::ChTriangleMesh& trimesh,	bool is_static, bool is_convex,  ChVector<>* pos, ChMatrix33<>* rot){
			model_type=TRIANGLEMESH;
			nObjects = trimesh.getNumTriangles();
			bData tData;
			for(int i=0; i<nObjects; i++){
				ChTriangle temptri=trimesh.getTriangle(i);
				tData.A=make_float4(temptri.p1.x,temptri.p1.y,temptri.p1.z,-2);
				tData.B=make_float4(temptri.p2.x,temptri.p2.y,temptri.p2.z,0);
				tData.C=make_float4(temptri.p3.x,temptri.p3.y,temptri.p3.z,0);
				mData.push_back(tData);
			}
			return true;
		}
		bool ChCollisionModelGPU::AddCopyOfAnotherModel (ChCollisionModel* another){
			//NOT SUPPORTED
			return false;
		}
		ChVector<> ChCollisionModelGPU::GetSpherePos(int ID){
			//ChVector<> pos(colSpheres[4*n+0],colSpheres[4*n+1],colSpheres[4*n+2]);
			ChVector<> pos;
			pos.x = mData[ID].A.x;
			pos.y = mData[ID].A.y;
			pos.z = mData[ID].A.z;
			return pos;
		}
		void ChCollisionModelGPU::GetAABB(ChVector<>& bbmin, ChVector<>& bbmax) const
		{
		}

		float ChCollisionModelGPU::GetSphereR(int ID){
			if (ID>nObjects)
				return -1.f;
			return mData[ID].A.w;
		}

		void  ChCollisionModelGPU::SetFamily(int mfamily){
			colFam = mfamily;
		}

		int   ChCollisionModelGPU::GetFamily(){
			return colFam;
		}

		void  ChCollisionModelGPU::SetFamilyMaskNoCollisionWithFamily(int mfamily){
			noCollWith = mfamily;
		} 

		void  ChCollisionModelGPU::SetFamilyMaskDoCollisionWithFamily(int mfamily){
			if(noCollWith = mfamily){
				noCollWith = -1;
			}
		}
		bool ChCollisionModelGPU::GetFamilyMaskDoesCollisionWithFamily(int mfamily){
			return (noCollWith!=mfamily);
		}

		int ChCollisionModelGPU::GetNoCollFamily(){
			return noCollWith;
		}
		void ChCollisionModelGPU::SyncPosition()
		{
			ChBody* bpointer = GetBody();
			assert(bpointer);
			//assert(bpointer->GetSystem());
		}

		ChPhysicsItem* ChCollisionModelGPU::GetPhysicsItem() {
			return (ChPhysicsItem*)GetBody();
		};

	} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

#endif  // end of ! CH_NOCUDA
