//   ChBody.cpp

#include "ChBodyGPU.h"
namespace chrono {
    using namespace collision;

    // Register into the object factory, to enable run-time
    // dynamic creation and persistence
    ChClassRegister<ChBodyGPU> a_registration_ChBodyGPU;


    ChBodyGPU::ChBodyGPU() {
        marklist.clear();
        forcelist.clear();
        BFlagsSetAllOFF();      // no flags
        Xforce = VNULL;
        Xtorque = VNULL;
        Force_acc = VNULL;
        Torque_acc = VNULL;
        Scr_force = VNULL;
        Scr_torque = VNULL;
        cdim = VNULL;
        collision_model = InstanceCollisionModel();
        matsurface = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);
        density = 1000.0f;
        conductivity = 0.2f;
        last_coll_pos = CSYSNORM;
        //SetIdentifier(CHGLOBALS().GetUniqueIntID()); // mark with unique ID
        max_speed = 0.5f;
        max_wvel  = 2.0f * float(CH_C_PI);
        sleep_time = 0.6f;
        sleep_starttime = 0;
        sleep_minspeed = 0.1f;
        sleep_minwvel = 0.04f;
        SetUseSleeping(true);
        variables.SetUserData((void*)this);
        id = 0;
    }

    ChBodyGPU::~ChBodyGPU() {
    }

    ChCollisionModel* ChBodyGPU::InstanceCollisionModel() {
        ChCollisionModel* collision_model_t = (ChCollisionModelGPU*) new ChCollisionModelGPU();
        ((ChCollisionModelGPU*) collision_model_t)->SetBody(this);
        return collision_model_t;
    }
} // END_OF_NAMESPACE____




