//////////////////////////////////////////////////
//  
//   ChCCollisionSystemGPU.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChCCollisionSystemGPU.h"

namespace chrono {
	namespace collision {
		ChCollisionSystemGPU::ChCollisionSystemGPU(float mEnvelope, float mBinSize, uint mMaxContact){
			mGPU.mEnvelope=mEnvelope;
			mGPU.mBinSize=mBinSize;
			mGPU.mMaxRad=0;

			mGPU.mNSpheres=0;
			mGPU.mNBoxes=0;
			mGPU.mNTriangles=0;
			mGPU.mMaxContact=mMaxContact;
			mGPU.InitCudaCollision();
		}
		ChCollisionSystemGPU::~ChCollisionSystemGPU(){
			mGPU.mDataSpheres.clear();
			mGPU.mDataBoxes.clear();
			mGPU.mDataTriangles.clear();
			mGPU.mAuxData.clear();
			for(int i=0; i<mGPU.mNBodies; i++){delete colModels[i];}
			colModels.clear();
		}
		void ChCollisionSystemGPU::Clear(void){}   

		void ChCollisionSystemGPU::SetSystemDescriptor(ChLcpSystemDescriptorGPU* mdescriptor){
			mSystemDescriptor=mdescriptor;
			mSystemDescriptor->maxContacts=mGPU.mMaxContact;
			mGPU.mContactsGPU=mSystemDescriptor->vContactsGPU;
			//mGPU.mContactBodyID=mSystemDescriptor->d_contact_bodyID;
		}

		void ChCollisionSystemGPU::Add(ChCollisionModel* model){
			chrono::collision::ChCollisionModelGPU* modelGPU=(chrono::collision::ChCollisionModelGPU*)model;
			colModels.push_back(modelGPU);
			if(modelGPU->GetType()==0){
				mGPU.mNSpheres+=modelGPU->GetNObjects();
			}else if(modelGPU->GetType()==1){
				mGPU.mNSpheres+=modelGPU->GetNObjects();
			}else if(modelGPU->GetType()==2){
				mGPU.mNBoxes+=modelGPU->GetNObjects();
			}else if(modelGPU->GetType()==3){
				mGPU.mNTriangles+=modelGPU->GetNObjects();
			}
			mGPU.mNBodies++;
			
		}

		void ChCollisionSystemGPU::Remove(ChCollisionModel* model){}

		void ChCollisionSystemGPU::Run(){
			updateDataStructures();
			if(mGPU.mNSpheres>0){
			mGPU.CudaCollision();
			}
			mSystemDescriptor->nContactsGPU=mGPU.mNumContacts;
		}
		void ChCollisionSystemGPU::ReportContacts(ChContactContainerBase* mcontactcontainer){}
		void ChCollisionSystemGPU::ReportProximities(ChProximityContainerBase* mproximitycontainer){
			mproximitycontainer->BeginAddProximities();
			mproximitycontainer->EndAddProximities();
		}

		bool ChCollisionSystemGPU::RayHit(const ChVector<>& from, const ChVector<>& to, ChRayhitResult& mresult){return false;}

		void ChCollisionSystemGPU::updateDataStructures(){
			chrono::collision::ChCollisionModelGPU* body;
				mGPU.mAuxData.clear();
				//mGPU.mColFam.clear();
				//mGPU.mNoCollWith.clear();
				mGPU.mDataSpheres.clear();
				
			if(mGPU.mDataBoxes.size()!=mGPU.mNBoxes){
				mGPU.mDataBoxes.resize(mGPU.mNBoxes);
			}
			if(mGPU.mDataTriangles.size()!=mGPU.mNTriangles){
				mGPU.mDataTriangles.resize(mGPU.mNTriangles);
			}
			indexB=0;
			indexT=0;
			if(mGPU.mNSpheres>0){
				mGPU.cMax=make_float3(-FLT_MAX  ,-FLT_MAX  ,-FLT_MAX  );
				mGPU.cMin=make_float3(FLT_MAX  ,FLT_MAX  ,FLT_MAX  );
			}
			else{
				mGPU.cMax=make_float3(-1  ,-1  ,-1  );
				mGPU.cMin=make_float3(1  ,1  ,1  );

			}
			mGPU.mMaxRad=0;

			for(int i=0; i<mGPU.mNBodies; i++){
				body = colModels[i];
				
				int type=body->GetType();
				//cout<<"TYPE "<<type<<endl;;
				if(type==0){
						mGPU.mAuxData.push_back(F4(i,body->GetFamily(),body->GetNoCollFamily(),body->GetBody()->GetKfriction()));
						gPos = body->GetBody()->GetCoord().TrasformLocalToParent(body->GetSpherePos(0));
						mGPU.mDataSpheres.push_back(make_float4(gPos.x,gPos.y,gPos.z,body->GetSphereR(0)));
						mGPU.mMaxRad=max(mGPU.mMaxRad,body->GetSphereR(0));
						mGPU.cMax.x=max(mGPU.cMax.x,(float)gPos.x+body->GetSphereR(0));
						mGPU.cMax.y=max(mGPU.cMax.y,(float)gPos.y+body->GetSphereR(0));
						mGPU.cMax.z=max(mGPU.cMax.z,(float)gPos.z+body->GetSphereR(0));
						mGPU.cMin.x=min(mGPU.cMin.x,(float)gPos.x-body->GetSphereR(0));
						mGPU.cMin.y=min(mGPU.cMin.y,(float)gPos.y-body->GetSphereR(0));
						mGPU.cMin.z=min(mGPU.cMin.z,(float)gPos.z-body->GetSphereR(0));
				}
				else if(type==1){
					for(int j=0; j<body->GetNObjects(); j++){
						mGPU.mAuxData.push_back(F4(i,body->GetFamily(),body->GetNoCollFamily(),body->GetBody()->GetKfriction()));
						gPos = body->GetBody()->GetCoord().TrasformLocalToParent(body->GetSpherePos(j));
						mGPU.mDataSpheres.push_back(make_float4(gPos.x,gPos.y,gPos.z,body->GetSphereR(j)));	
						mGPU.mMaxRad=max(mGPU.mMaxRad,body->GetSphereR(j));
						mGPU.cMax.x=max(mGPU.cMax.x,(float)gPos.x);
						mGPU.cMax.y=max(mGPU.cMax.y,(float)gPos.y);
						mGPU.cMax.z=max(mGPU.cMax.z,(float)gPos.z);
						mGPU.cMin.x=min(mGPU.cMin.x,(float)gPos.x);
						mGPU.cMin.y=min(mGPU.cMin.y,(float)gPos.y);
						mGPU.cMin.z=min(mGPU.cMin.z,(float)gPos.z);
					}
				}
				else if(type==2){
					localPos=body->GetBody()->GetPos();
					mGPU.mDataBoxes[indexB].A = body->mData[0].A+make_float4(localPos.x,localPos.y,localPos.z,0);
					mGPU.mDataBoxes[indexB].B = body->mData[0].B;
					mGPU.mDataBoxes[indexB].C = body->mData[0].C;
					mGPU.mDataBoxes[indexB].B.w=i;
					mGPU.mDataBoxes[indexB].A.w=body->GetBody()->GetKfriction();
					//cout<<"Body: "<<i<<" "<<localPos.x<<" "<<localPos.y<<" "<<localPos.z<<endl;
					indexB++;
				}
				else if(type==3){
					localPos=body->GetBody()->GetPos();
					ChMatrix33<>* rotA=body->GetBody()->GetA();
					ChVector<> pA=localPos+rotA->Matr_x_Vect(ChVector<>(body->mData[0].A.x,body->mData[0].A.y,body->mData[0].A.z));
					ChVector<> pB=localPos+rotA->Matr_x_Vect(ChVector<>(body->mData[0].B.x,body->mData[0].B.y,body->mData[0].B.z));
					ChVector<> pC=localPos+rotA->Matr_x_Vect(ChVector<>(body->mData[0].C.x,body->mData[0].C.y,body->mData[0].C.z));

					mGPU.mDataTriangles[indexT].A = make_float4(pA.x,pA.y,pA.z,body->GetBody()->GetKfriction());
					mGPU.mDataTriangles[indexT].B = make_float4(pB.x,pB.y,pB.z,i);
					mGPU.mDataTriangles[indexT].C = make_float4(pC.x,pC.y,pC.z,0);
					indexT++;
				}
			}
		}
	} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____
