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

		ChModelGPU::ChModelGPU(){
			this->mPos=make_float3(0,0,0);


			nObjects = 0;
			colFam = 0;
			noCollWith = -1;
			mType=0;
		}

		ChModelGPU::~ChModelGPU(){
			//ClearModel(); not possible, would call GetPhysicsItem() that is pure virtual, enough to use instead..
			mData.clear();
		}



		int ChModelGPU::ClearModel(){
			//cout<<"CLEAR"<<endl;
			if (GetPhysicsItem()->GetSystem()){
				if (GetPhysicsItem()->GetCollide())
				{
					GetPhysicsItem()->GetSystem()->GetCollisionSystem()->Remove(this);
				}
			}

			mData.clear();

			//posX = 0.f;
			//posY = 0.f;
			//posZ = 0.f;
			mPos=make_float3(0,0,0);
			nObjects=0;
			colFam = 0;
			noCollWith = -1;
			mType=0;
			return 1;
		} 


		int ChModelGPU::BuildModel(){
			//assert((ChBody*)GetPhysicsItem());
			if (GetPhysicsItem()->GetSystem())
				if (GetPhysicsItem()->GetCollide())
					GetPhysicsItem()->GetSystem()->GetCollisionSystem()->Add(this);

			return 1;
		}

		bool ChModelGPU::AddCompoundBody(int numSpheres, vector<float4> &data){
			mType=1;

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

		bool ChModelGPU::AddSphere(double radius,  ChVector<>* pos){
			mType=0;
			//cout<<"MAKE Sphere"<<endl;
			//add a sphere body - this collision geometry has one collision sphere at the body coordinate origin
			nObjects = 1;
			bData tData;
			tData.B=make_float4(0,0,0,0);
			tData.C=make_float4(0,0,0,0);
			if(pos!=0){
				tData.A=make_float4((*pos).x,(*pos).x,(*pos).x,radius);
				mPos=make_float3((*pos).x,(*pos).y,(*pos).z);
			}
			else{
				tData.A=make_float4(0,0,0,radius);
				mPos=make_float3(0,0,0);
			}
			mData.push_back(tData);
			return true;
		}
		bool ChModelGPU::AddEllipsoid(double rx,  double ry,  double rz, ChVector<>* pos, ChMatrix33<>* rot){
			//NOT SUPPORTED
			return false;
		}
		bool ChModelGPU::AddBox(double hx, double hy, double hz, ChVector<>* pos, ChMatrix33<>* rot){
			//cout<<"MAKE Boxx"<<endl;
			mType=2;
			nObjects = 1;
			bData tData;
			ChQuaternion<double> quat=mbody->GetRot();
			tData.A=make_float4(pos->x,pos->y,pos->z,-2);
			tData.B=make_float4(hx,hz,hy,0);
			tData.C=make_float4(quat.e0,quat.e1,quat.e2,quat.e3);

			mPos=make_float3(pos->x,pos->y,pos->z);
			mData.push_back(tData);

			return true;
		}
		bool ChModelGPU::AddTriangle(ChVector<> A, ChVector<> B, ChVector<> C, ChVector<>* pos, ChMatrix33<>* rot){
			//cout<<"MAKE Triangle"<<endl;
			mType=3;
			nObjects = 1;
			bData tData;
			//ChQuaternion<double> quat=mbody->GetRot();

			tData.A=make_float4(A.x,A.y,A.z,-2);
			tData.B=make_float4(B.x,B.y,B.z,0);
			tData.C=make_float4(C.x,C.y,C.z,0);

			mData.push_back(tData);

			return true;
		}
		/// Add a cylinder to this model (default axis on Y direction), for collision purposes
		bool ChModelGPU::AddCylinder (double rx, double rz, double hy, ChVector<>* pos, ChMatrix33<>* rot){
			//NOT SUPPORTED
			return false;
		}

		bool ChModelGPU::AddConvexHull (std::vector<ChVector<double> >& pointlist, ChVector<>* pos, ChMatrix33<>* rot)
		{
			//NOT SUPPORTED
			return false;
		}
		bool ChModelGPU::AddBarrel (double Y_low, double Y_high, double R_vert, double R_hor, double R_offset, ChVector<>* pos, ChMatrix33<>* rot){
			//NOT SUPPORTED
			return false;
		}
		/// Add a triangle mesh to this model
		bool ChModelGPU::AddTriangleMesh (const  geometry::ChTriangleMesh& trimesh,	bool is_static, bool is_convex,  ChVector<>* pos, ChMatrix33<>* rot){
			//NOT SUPPORTED
			return false;
		}

		bool ChModelGPU::AddCopyOfAnotherModel (ChCollisionModel* another){
			//NOT SUPPORTED
			return false;
		}
		ChVector<> ChModelGPU::GetSpherePos(int ID){
			//ChVector<> pos(colSpheres[4*n+0],colSpheres[4*n+1],colSpheres[4*n+2]);
			ChVector<> pos;
			pos.x = mData[ID].A.x;
			pos.y = mData[ID].A.y;
			pos.z = mData[ID].A.z;
			return pos;
		}

		void ChModelGPU::GetAABB(ChVector<>& bbmin, ChVector<>& bbmax) const
		{

		}

		float ChModelGPU::GetSphereR(int ID){
			if (ID>nObjects)
				return -1.f;
			return mData[ID].A.w;
		}

		int ChModelGPU::GetType(){
			return mType;
		}
		void  ChModelGPU::SetFamily(int mfamily){
			colFam = mfamily;
		}

		int   ChModelGPU::GetFamily(){
			return colFam;
		}

		void  ChModelGPU::SetFamilyMaskNoCollisionWithFamily(int mfamily){
			noCollWith = mfamily;
		} 

		void  ChModelGPU::SetFamilyMaskDoCollisionWithFamily(int mfamily){
			if(noCollWith = mfamily){
				noCollWith = -1;
			}
		}
		bool ChModelGPU::GetFamilyMaskDoesCollisionWithFamily(int mfamily){
			return (noCollWith!=mfamily);
		}

		int ChModelGPU::GetNoCollFamily(){
			return noCollWith;
		}
		void ChModelGPU::SyncPosition()
		{
			ChBody* bpointer = GetBody();
			assert(bpointer);
			//assert(bpointer->GetSystem());
			mPos=make_float3(bpointer->GetPos().x,bpointer->GetPos().y,bpointer->GetPos().z);
		}

		ChPhysicsItem* ChModelGPU::GetPhysicsItem() {
			return (ChPhysicsItem*)GetBody();
		};

	} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

#endif  // end of ! CH_NOCUDA
