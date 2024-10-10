#include "ChElectronicsNetlist.h"

namespace chrono {
namespace powerelectronics {

std::string ChElectronicsNetlist::AsString() {
    std::string str;

    for(std::string line : netlist_file) {
        str += line + "\n";
    }

    return str;
}

void ChElectronicsNetlist::InitNetlist(std::string file, double t_step, double t_end) {

    this->netlist_file = this->ReadNetlistFile(file);

    /* Initialize PWM strings
    */
    auto pwl_sources = GetPWLConds(this->initial_pwl_in);
    this->netlist_file = UpdatePWLSources(this->netlist_file, pwl_sources, t_step, t_end);

    /* Initialize inductance params of Flow-In Vars
    */
    auto init_flowin = this->initial_flowin_ics;
    this->netlist_file = this->UpdateFlowInParams(this->netlist_file, init_flowin);

}

Netlist_V ChElectronicsNetlist::ReadNetlistFile(std::string file) {
    /* Open Netlist*/
    std::vector<std::string> file_contents;
    std::ifstream infile(file);

    // Check if the file is open
    if (infile.is_open()) {
        std::string line;
        
        // Read the file line by line and store each line in the vector
        while (std::getline(infile, line)) {
            file_contents.push_back(line);
        }
        
        infile.close();
    } else {
        std::cerr << "Unable to open file: " << file << std::endl;
    }
    return file_contents;
}

void ChElectronicsNetlist::UpdateNetlist(CosimResults results, FlowInMap flowin_map, PWLInMap pwl_in, double t_step, double t_end) {
    
    auto pwl_sources = GetPWLConds(pwl_in);
    this->netlist_file = UpdatePWLSources(this->netlist_file, pwl_sources, t_step, t_end);

    this->netlist_file = this->UpdateFlowInParams(this->netlist_file, flowin_map);

    auto node_v_states = GetVoltageConds(results);
    this->netlist_file = this->UpdateVoltageICs(this->netlist_file, node_v_states);

    auto branch_states = GetBranchConds(results);
    this->netlist_file = this->UpdateBranchCurrents(this->netlist_file, branch_states);
       
    // std::cout << this->AsString() << std::endl;

}


Netlist_V ChElectronicsNetlist::UpdatePWLSources(Netlist_V file, PWLSourceMap map, double t_step, double t_end) {
    // Loop through each element in the map
    for (const auto& pwl_source : map) {
        const std::string& key = pwl_source.first;
        const std::pair<double, double>& values = pwl_source.second;
        
        std::string pwl_seq = GeneratePWLSequence(values, t_step, t_end);
        std::string pwl_string = "PWL(" + pwl_seq + ")";

        // Iterate through the netlist vector to find the line containing the key
        for (auto& line : file) {
            // Check if the line contains the key
            if (line.find(key) != std::string::npos) {
                // Check if PWL() already exists in the line
                std::size_t pwl_pos = line.find("PWL(");
                if (pwl_pos != std::string::npos) {
                    // Overwrite the existing PWL() by replacing the substring starting from PWL
                    line = line.substr(0, pwl_pos) + pwl_string;
                } else {
                    // Append the PWL() to the end of the line
                    line += pwl_string;
                }
                break;  // Break since we only want to modify one occurrence
            }
        }
    }
    
    return file;
}

std::string ChElectronicsNetlist::GeneratePWLSequence(std::pair<double, double> V, double t_step, double t_end) {
    // Extract initial and final voltages
    double V_0 = V.first;
    double V_f = V.second;

    // Prepare the result string
    std::ostringstream pwl_sequence;
    pwl_sequence.precision(6);  // Set precision for voltage values

    // Calculate the number of steps
    int num_steps = static_cast<int>(t_end / t_step);

    // Generate the sequence
    for (int i = 0; i <= num_steps; ++i) {
        double t_i = i * t_step;
        double V_i = V_0 + (V_f - V_0) * (t_i / t_end);

        // Append to the sequence string (format: time voltage)
        pwl_sequence << t_i << " " << V_i;

        //Add a comma if it's not the last element
        if (i < num_steps) {
            pwl_sequence << " ";
        }
    }

    return pwl_sequence.str();
}

Netlist_V ChElectronicsNetlist::UpdateFlowInParams(Netlist_V netlist, FlowInMap map) {
    // Iterate over the FlowInMap
    for (const auto& flowIn : map) {
        // Extract the flow in variable name
        std::string flowInVarName = flowIn.first;  // key of the map
        std::string paramString = ".param par" + flowInVarName + " = " + std::to_string(flowIn.second); // creating param string
        
        bool paramExists = false;

        // Check if the parameter already exists in the netlist and update it if found
        for (auto& line : netlist) {
            if (line.find(".param par" + flowInVarName) != std::string::npos) {
                line = paramString; // Update line
                paramExists = true;
                break;
            }
        }
        if (!paramExists) {
            netlist.push_back(paramString);
        }
    }

    return netlist; 
}

/* For every key in VoltageMap, initialize or update the corresponding V({key})=value string on the .ic line */

Netlist_V ChElectronicsNetlist::UpdateVoltageICs(Netlist_V netlist, VoltageMap map) {
    // Find the line starting with ".ic"
    std::string ic_line_prefix = ".ic";
    int ic_line_index = -1;
    for (size_t i = 0; i < netlist.size(); i++) {
        if (netlist[i].find(ic_line_prefix) == 0) {
            ic_line_index = i;
            break;
        }
    }

    std::string updated_ic_line = ic_line_prefix;

    // If there is an existing .ic line, parse it
    std::unordered_map<std::string, double> existing_voltages;
    if (ic_line_index != -1) {
        std::istringstream ss(netlist[ic_line_index].substr(ic_line_prefix.length()));
        std::string voltage_entry;

        while (ss >> voltage_entry) {
            // Each entry should be in the form V(node)=value
            auto equals_pos = voltage_entry.find('=');
            if (equals_pos != std::string::npos && voltage_entry[0] == 'V' && voltage_entry[1] == '(') {
                std::string node = voltage_entry.substr(2, equals_pos - 3); // Extract node
                double value = std::stod(voltage_entry.substr(equals_pos + 1)); // Extract value
                existing_voltages[node] = value;
            }
        }
    }

    // Merge VoltageMap into the existing voltages
    for (const auto& pair : map) {
        existing_voltages[pair.first] = pair.second;
    }

    // Rebuild the .ic line
    for (const auto& pair : existing_voltages) {
        updated_ic_line += " V(" + pair.first + ")=" + std::to_string(pair.second);
    }

    // Update or insert the new .ic line into the netlist
    if (ic_line_index != -1) {
        netlist[ic_line_index] = updated_ic_line;
    } else {
        netlist.push_back(updated_ic_line);
    }

    return netlist;
}


VoltageMap ChElectronicsNetlist::GetVoltageConds(const CosimResults& results) {
    VoltageMap voltageMap;

    // Ensure that node_name and node_val sizes match
    if (results.node_name.size() != results.node_val.size()) {
        throw std::runtime_error("Mismatch between node names and voltage values per node");
    }

    // Check if there are any time steps to consider
    if (results.node_val.empty() || results.node_val.back().empty()) {
        throw std::runtime_error("No voltage data available");
    }

    // Populate the map with the node names and their corresponding voltage values for the last time step
    for (size_t i = 0; i < results.node_name.size(); ++i) {
        if(results.node_name[i] == "time" ) { //|| results.node_name[i] == "n5") {
            continue;
        }

        double last_time_step_voltage = results.node_val[i].back();
        voltageMap[results.node_name[i]] = last_time_step_voltage;
    }

    return voltageMap;
}

PWLSourceMap ChElectronicsNetlist::GetPWLConds(PWLInMap pwl_in) {
    PWLSourceMap pwl_sources;

    // Check if last_pwl_in is empty
    bool last_pwl_in_empty = this->last_pwl_in.empty();

    // Iterate through pwl_in from the last to the first
    for (auto it = pwl_in.rbegin(); it != pwl_in.rend(); ++it) {
        const std::string& key = it->first;
        double V_f = it->second;
        
        // Set V_0 based on last_pwl_in, or initialize with V_f if last_pwl_in is empty
        double V_0;
        if (last_pwl_in_empty) {
            V_0 = V_f; // If last_pwl_in is empty, initialize V_0 with V_f
        } else {
            V_0 = this->last_pwl_in[key]; // Otherwise, use the value from last_pwl_in
        }

        // Add the pair (V_0, V_f) to the pwl_sources map
        pwl_sources[key] = std::make_pair(V_0, V_f);
    }

    // Store the last pwl_in for future use
    this->last_pwl_in = pwl_in;

    return pwl_sources;
}


/* using keys from this->tracked_branches (a vector) and cosim results, construct a new 
    BranchInMap with updated currents from results
*/
BranchCurrentMap ChElectronicsNetlist::GetBranchConds(const CosimResults& results) {
    BranchCurrentMap branchCurrentMap;

    // Iterate through the tracked branches
    for (const auto& branch : this->tracked_branches) {
        
        // Find the index of the tracked branch in the results branch_name vector
        auto it = std::find(results.branch_name.begin(), results.branch_name.end(), toLowerCase(branch));
        
        // If the branch is found in the results
        if (it != results.branch_name.end()) {
            // Get the index of the branch
            size_t index = std::distance(results.branch_name.begin(), it);

            // Ensure that the index is valid for the branch_val vector
            if (index < results.branch_val.size()) {
                // Get the latest current value for the branch
                double current_value = results.branch_val.back()[index];

                // Insert the branch name and its corresponding current into the map
                branchCurrentMap[branch] = current_value;
            }
        }
    }

    return branchCurrentMap;
}

/* For every key in BranchInMap, initialize or update the corresponding .param ic{key} string in the netlist*/

Netlist_V ChElectronicsNetlist::UpdateBranchCurrents(Netlist_V netlist, BranchCurrentMap map) {
    // Loop through each branch in the BranchCurrentMap
    for (const auto& [branch, current] : map) {
        // Construct the .param string for this branch
        std::string param_str = ".param ic" + branch + " = " + std::to_string(current);

        // Flag to indicate if the parameter was found and updated
        bool found = false;

        // Loop through the netlist to find if an existing entry for this branch exists
        for (auto& line : netlist) {
            // Check if the line contains .param ic{branch}
            std::string search_str = ".param ic" + branch;

            if (line.find(search_str) != std::string::npos) {
                // Update the existing line with the new current value
                line = param_str;
                found = true;
                break;
            }
        }

        // If the parameter was not found, add a new entry to the netlist
        if (!found) {
            netlist.push_back(param_str);
        }
    }

    return netlist;
}

}
}