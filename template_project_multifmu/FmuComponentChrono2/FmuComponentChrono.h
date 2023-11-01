
#pragma once
//#define FMI2_FUNCTION_PREFIX MyModel_
#include <FmuToolsExport.h>
#include <vector>
#include <array>

#include <chrono/physics/ChSystemNSC.h>
#include <chrono/physics/ChBodyEasy.h>
#include "chrono/core/ChRealtimeStep.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#endif



using namespace chrono;


class FmuComponent: public FmuComponentBase{
public:
    FmuComponent(fmi2String _instanceName, fmi2Type _fmuType, fmi2String _fmuGUID);

    virtual ~FmuComponent(){}

    /// FMU_ACTION: override _doStep of the base class with the problem-specific implementation
    virtual fmi2Status _doStep(fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize, fmi2Boolean noSetFMUStatePriorToCurrentPoint) override;



protected:

    virtual void _exitInitializationMode() override;

    virtual void _preModelDescriptionExport() override;
    virtual void _postModelDescriptionExport() override;

    virtual bool is_cosimulation_available() const override { return true;}
    virtual bool is_modelexchange_available() const override { return false;}

    // Problem-specific data members   
    ChSystemNSC sys;
    
    ChRealtimeStepTimer realtime_timer;

#ifdef CHRONO_IRRLICHT
    std::shared_ptr<chrono::irrlicht::ChVisualSystemIrrlicht> vis;
#endif





    double x_tt;
    double x_t;
    double x;
    double theta_tt;
    double theta_t;
    double theta;
    double pendulum_length = 0.5;
    double pendulum_mass = 1.0;
    double cart_mass = 1.0;


};


// FMU_ACTION:: implement the following functions
FmuComponentBase* fmi2Instantiate_getPointer(fmi2String instanceName, fmi2Type fmuType, fmi2String fmuGUID){
    return new FmuComponent(instanceName, fmuType, fmuGUID);
}


