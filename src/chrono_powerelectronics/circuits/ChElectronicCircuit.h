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
#include "../ChElectronicsNetlist.h"
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

    virtual void PreAdvance (double dt_mbs) {};

    
    virtual void PostAdvance (double dt_mbs) {};

    virtual void Initialize(double dt_mbs) final {
        // Pre initialize the model
        this->PreInitialize();
        cosim.Initialize(netlist_file);
        netlist = cosim.GetNetlist().netlist_file;
        this->PostInitialize();

        // Generate a preliminary result map to obtain the branches related to the inductance inside the Netlist
        this->Advance(dt_mbs);
        auto res = this->GetResult();
        /*
        // Plot the results
        std::cout << "Results:\n" << std::endl;
        for (const auto& [key, values] : res) { 
            std::cout << key << ": ";
            }
            std::cout << std::endl;*/

        // Generate the list of inductance inside the Netlist
        std::vector<std::string> InductanceMap = this->InductanceListDef(res);
        this->SetBranchTracking(InductanceMap);

        // Officially initialize the model
        this->PreInitialize();
        cosim.Initialize(netlist_file);
        netlist = cosim.GetNetlist().netlist_file;
        this->PostInitialize();

        // Set the Netlist to the real firs Initial Condition
        this->Advance(dt_mbs);
        
    }

    virtual void Advance(double dt_mbs) final {
        this->PreAdvance(dt_mbs);

        // std::cout << "#########################################################" << std::endl;

        t_sim_electronics += dt_mbs;

        auto runSpiceStart = std::chrono::high_resolution_clock::now();
        /*std::cout << "#########################################################" << std::endl;
        std::cout << "Netlist file: " << std::endl;
            for (const auto& line : this->netlist) {
                std::cout << line << std::endl;
            }
        std::cout << "#########################################################" << std::endl;*/
        
        this->result = cosim.RunSpice(this->netlist, this->t_step, dt_mbs, initial);
        auto runSpiceEnd = std::chrono::high_resolution_clock::now();
        auto runSpiceTime = std::chrono::duration_cast<std::chrono::microseconds>(runSpiceEnd - runSpiceStart).count();
        
        auto cosimStart = std::chrono::high_resolution_clock::now();
        cosim.Cosimulate(cosim.GetResult_V(), this->flow_in, this->pwl_in, this->t_step, dt_mbs);
        auto cosimEnd = std::chrono::high_resolution_clock::now();
        auto cosimTime = std::chrono::duration_cast<std::chrono::microseconds>(cosimEnd - cosimStart).count();
        
        this->netlist = cosim.GetNetlist().netlist_file;

        this->PostAdvance(dt_mbs);

        initial = false;

        // std::cout << "RunSpice time: " << runSpiceTime << " microseconds" << std::endl;
        // std::cout << "Cosimulate time: " << cosimTime << " microseconds" << std::endl;
    }

    void SetInitialPWLIn(PWLInMap map) {
        this->cosim.SetInitialPWLIn(map);
    }

    void SetInitialFlowInICs(FlowInMap map) {
        this->cosim.SetInitialFlowInICs(map);
    }

    std::vector<std::string> InductanceListDef(std::map<std::string,std::vector<double>> resmap){
        std::vector<std::string> InductanceMap;
        for (const auto& [key, vec] : resmap) {
            if (netlistObj.toLowerCase(key)[0] == 'l'){
                //std::cout << key << std::endl;
                InductanceMap.push_back(key);
            }
        }
        return InductanceMap;
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
    // Added this method since the netlist member is private, it is not directly accessible from the outside
    Netlist_V GetNetlist() const {
        return this->netlist;
    }


private:

    //
    std::string netlist_file;

    ChElectronicsCosimulation cosim;

    double t_step;
    bool initial = true;

    double t_sim_electronics = 0;
    std::map<std::string,std::vector<double>> result;

protected:
    Netlist_V netlist;
    FlowInMap flow_in;
    PWLInMap pwl_in;
    ChElectronicsNetlist netlistObj; 
    
};

}
}

#endif // CHELECTRONICSCIRCUITS_H