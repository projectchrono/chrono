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

#ifndef CHELECTRONICSNETLIST_H
#define CHELECTRONICSNETLIST_H

// =========================
// ======== Headers ========
// =========================
#include "ChElectronicsCosimResult.h"
#include <map>
#include <iostream>
#include <fstream> 
#include <sstream> 
#include <string>   
#include <algorithm>    // For std::find

typedef std::vector<std::string> Netlist_V;
typedef std::map<std::string,std::pair<double,double>> PWLSourceMap;
typedef std::map<std::string,double> PWLInMap;
typedef std::map<std::string,double> FlowInMap;
typedef std::map<std::string,double> VoltageMap;
typedef std::map<std::string,double> BranchCurrentMap;
typedef std::vector<std::string> Branch_V;

namespace chrono {
namespace powerelectronics {

class ChElectronicsNetlist {
public:

    Netlist_V netlist_file;
    CosimResults last_results;
    
    FlowInMap initial_flowin_ics;

    PWLInMap  initial_pwl_in;
    PWLInMap last_pwl_in{};

    Branch_V tracked_branches;

    /* Initialization */    
    void InitNetlist(std::string file, double t_step, double t_end);

    Netlist_V ReadNetlistFile(std::string file);

    // TODO: Get initial PWL source states (assumed constant from t_0->t_f)

    void SetInitialPWLIn(PWLInMap map) {
        this->initial_pwl_in = map;
    }

    void SetInitialFlowInICs(FlowInMap map) {
        this->initial_flowin_ics = map;
    }

    void SetBranchTracking(Branch_V branches) {

        this->tracked_branches = branches;
    }

    /*void SetBranchTracking(Branch_V branches) {
        this->tracked_branches = branches;
    }*/

    // For each key in pwl_in, and from last,
    // set V_0 and V_f in pwl_sources accordingly
    PWLSourceMap GetPWLConds(PWLInMap pwl_in);

    /* For all nodes, set voltage map according to results.node_val
    *  and results.node_name vectors
    */
    VoltageMap GetVoltageConds(const CosimResults& results);

    /* using keys from this->tracked_branches and cosim results, construct a new 
        BranchInMap with updated currents from results
    */
    BranchCurrentMap GetBranchConds(const CosimResults& results);

    /* Cosimulation / Pre-warming */
    void UpdateNetlist(CosimResults results, FlowInMap map, PWLInMap pwl_in, double t_step, double t_end);

    /* For every key in FlowInMap, initialize or update a .param par{...} string in the netlist */
    Netlist_V UpdateFlowInParams(Netlist_V netlist, FlowInMap map);           

    /* For every key in PWLSourceMap, initialize a PWL(...) string in the netlist */
    Netlist_V UpdatePWLSources(Netlist_V netlist, PWLSourceMap map, double t_step, double t_end);  // PWL(...)

    /* For every key in VoltageMap, initialize or update the corresponding V({key})=value string on the .ic line */
    Netlist_V UpdateVoltageICs(Netlist_V netlist, VoltageMap map); // .ic V(...)

    /* For every key in BranchInMap, initialize or update the corresponding .param ic{key} string in the netlist*/
    Netlist_V UpdateBranchCurrents(Netlist_V netlist, BranchCurrentMap branch_currents); // .param ic{key}

    /* Netlist state as string */
    std::string AsString();

    /*
    *   Create a string of form 
    *   PWL(t_0 V_0 t_1 V_1, ..., t_f V_f) 
    *   V_i is a linearly interpolated value between V_0,V_f 
    */
    std::string GeneratePWLSequence(
        std::pair<double,double> V,  // Order: [V_0, V_f]
        double t_start, double t_end);

    // typedef std::vector<std::string> Netlist_V;
    void WriteNetlist(const std::string& file) {
        // Open the file in write mode
        std::ofstream outFile(file);

        // Check if the file is successfully opened
        if (!outFile.is_open()) {
            std::cerr << "Error: Could not open the file " << file << " for writing." << std::endl;
            return;
        }

        // Write each line of the netlist to the file
        for (const auto& line : this->netlist_file) {
            outFile << line << std::endl;
        }

        // Close the file
        outFile.close();

        // Confirm the write operation
        std::cout << "Netlist successfully written to " << file << std::endl;
    }
    

    std::string toLowerCase(const std::string& str) {
        std::string lower_str = str;
        std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(),
                    [](unsigned char c){ return std::tolower(c); });
        return lower_str;
    }

};

}
}

#endif