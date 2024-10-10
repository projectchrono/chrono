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

#ifndef CHELECTRONICSEXECUTOR_H
#define CHELECTRONICSEXECUTOR_H

// =========================
// ======== Headers ========
// =========================
#include "ChElectronicsNetlist.h"
#include "ChElectronicsCosimResult.h"
//#include "ChElectronics.h"

#include <filesystem>

// =======================
// ======== Class ========
// =======================

namespace chrono {
namespace powerelectronics {

class ChElectronicsCosimulation {  
public:

    // =============================
    // ======== Constructor ========
    // =============================
    ChElectronicsCosimulation(std::string netlist) {
        this->Initialize(netlist);
    }

    ChElectronicsCosimulation() {}


    void Initialize(std::string circuit_file) 
    {
        netlist.InitNetlist(circuit_file, 1e-6, 2e-4);
        this->Initialize();
    }

    void Initialize() {
    }

    typedef std::map<std::string,std::vector<double>> CircStateMap;

    CircStateMap RunSpice(Netlist_V netlist, double t_step, double t_end, bool initial);

    void Cosimulate(CosimResults results, FlowInMap flow_in, PWLInMap pwl_in, double t_step, double t_end);
    
    CosimResults GetResult_V();


    double GetCurrStep() {
        return sim_step;
    }

    void SetInitialPWLIn (PWLInMap map) {
        this->netlist.SetInitialPWLIn(map);
    }

    void SetInitialFlowInICs(FlowInMap map) {
        this->netlist.SetInitialFlowInICs(map);
    }

    void SetBranchTracking(Branch_V branches) {
        this->netlist.SetBranchTracking(branches);
    }

    void IncrementStep() {
        sim_step++;
    }
    
    ChElectronicsNetlist GetNetlist() {
        return netlist;
    }
    
private:

    int sim_step = 1;                               // Initialize the simulation step counter to 1
    double t_clock = 0.0;                            // Global clock of the simulation
    
    ChElectronicsNetlist netlist;

    std::string data_dir;
    
    CosimResults results;


};
}
}

#endif // CHELECTRONICSEXECUTOR_H