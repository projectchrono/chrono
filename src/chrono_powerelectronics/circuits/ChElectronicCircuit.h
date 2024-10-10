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

#ifndef CHELECTRONICSCIRCUITS_H
#define CHELECTRONICSCIRCUITS_H

// =========================
// ======== Headers ========
// =========================

#include "../ChElectronicsCosimResult.h"
#include "../ChElectronicsCosimulation.h"
#include <chrono>
// =======================
// ======== Class ========
// =======================

// using namespace std;

namespace chrono {
namespace powerelectronics {

class ChElectronicCircuit {  
public:


    ChElectronicCircuit(std::string netlist_file, double t_step) {
        this->netlist_file = netlist_file;
        this->t_step = t_step;
    }


    virtual void PreInitialize() {};
    virtual void PostInitialize() {};

    virtual void PreAdvance () {};

    
    virtual void PostAdvance () {};

    virtual void Initialize() final {
        this->PreInitialize();
        cosim.Initialize(netlist_file);
        netlist = cosim.GetNetlist().netlist_file;
        this->PostInitialize();
    }

    virtual void Advance(double dt_mbs) final {
        this->PreAdvance();

        // std::cout << "#########################################################" << std::endl;

        t_sim_electronics += dt_mbs;

        auto runSpiceStart = std::chrono::high_resolution_clock::now();
        this->result = cosim.RunSpice(this->netlist, this->t_step, dt_mbs);
        auto runSpiceEnd = std::chrono::high_resolution_clock::now();
        auto runSpiceTime = std::chrono::duration_cast<std::chrono::microseconds>(runSpiceEnd - runSpiceStart).count();
        
        auto cosimStart = std::chrono::high_resolution_clock::now();
        cosim.Cosimulate(cosim.GetResult_V(), this->flow_in, this->pwl_in, this->t_step, dt_mbs);
        auto cosimEnd = std::chrono::high_resolution_clock::now();
        auto cosimTime = std::chrono::duration_cast<std::chrono::microseconds>(cosimEnd - cosimStart).count();
        
        this->netlist = cosim.GetNetlist().netlist_file;

        this->PostAdvance();

        // std::cout << "RunSpice time: " << runSpiceTime << " microseconds" << std::endl;
        // std::cout << "Cosimulate time: " << cosimTime << " microseconds" << std::endl;
    }

    void SetInitialPWLIn(PWLInMap map) {
        this->cosim.SetInitialPWLIn(map);
    }

    void SetInitialFlowInICs(FlowInMap map) {
        this->cosim.SetInitialFlowInICs(map);
    }

    void SetBranchTracking(Branch_V branches) {
        this->cosim.SetBranchTracking(branches);
    }

    std::map<std::string,std::vector<double>> GetResult() {
        return this->result;
    }

    FlowInMap GetFlowIn() {
        return flow_in;
    }
    PWLInMap GetPWLIn() {
        return pwl_in;
    }

private:

    Netlist_V netlist;
    std::string netlist_file;

    ChElectronicsCosimulation cosim;

    double t_step;


    double t_sim_electronics = 0;
    std::map<std::string,std::vector<double>> result;

protected:
    FlowInMap flow_in;
    PWLInMap pwl_in;
    
};

}
}

#endif // CHELECTRONICSCIRCUITS_H