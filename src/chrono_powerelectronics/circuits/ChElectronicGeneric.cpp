// ==================================================================================================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 Federico Maria Reato, Matteo Santelia, Filippo Morlacchi, Claudio Ricci, Maurizio Zama, Iseo Serrature S.p.a, projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// ==================================================================================================================================================
// Authors: Federico Maria Reato, Matteo Santelia, Filippo Morlacchi, Claudio Ricci
// ==================================================================================================================================================

#include "ChElectronicGeneric.h"

namespace chrono {
namespace powerelectronics {

ChElectronicGeneric::ChElectronicGeneric(std::string Netlist_location, double t_step)
    : ChElectronicCircuit(Netlist_location, t_step) {
}

void ChElectronicGeneric::InputDefinition(std::map<std::string, double> PWLIn, std::map<std::string, double> FlowIn){
    PWLInVar = PWLIn;
    FlowInVar = FlowIn;
}

void ChElectronicGeneric::PreInitialize() { 
 
    std::cout << "Initializing Circuit" << std::endl;
    
    this->SetInitialPWLIn(PWLInVar);

    this->SetInitialFlowInICs(FlowInVar);

    /*
    this->SetBranchTracking({
        "LaC"
    });*/
} 

void ChElectronicGeneric::PostInitialize() {
    // Additional post-initialization logic can be added here if necessary.
}

void ChElectronicGeneric::PreAdvance(double dt_mbs) { 
    for (const auto& entry : PWLInVar) {
        this->pwl_in[entry.first] = entry.second;
    }

    for (const auto& entry : FlowInVar) {
        this->flow_in[entry.first] = entry.second;
    }
} 

void ChElectronicGeneric::PostAdvance(double dt_mbs) {
}

}
}
