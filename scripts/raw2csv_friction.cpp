#include <fstream>
#include <iostream>

using namespace std;

int main(int argc, char** argv) {
    if (argc != 4) {
        cerr << "Usage: raw2csv raw_data_file csv_data_file velcomponents" << std::endl;
        return 1;
    }
    
    bool write_vel_components = stoi(argv[3]);

    ifstream raw_data_file(argv[1], ifstream::in | ifstream::binary);
    ofstream csv_data_file(argv[2], ofstream::out);
    csv_data_file << "x,y,z,";
    if (write_vel_components) {
        csv_data_file << "vx,vy,vz,";
    }

    csv_data_file << "absv\n";

    while (raw_data_file.good()) {
        int n_vals = (write_vel_components) ? 10 : 7;
        int* vals = new int[n_vals];
        raw_data_file.read((char*)vals, n_vals * sizeof(float));
        // if we didn't actually get a whole line
        if (raw_data_file.gcount() < n_vals * sizeof(float)) {
            break;
        }
        csv_data_file << *((float*)&vals[0]) << "," << *((float*)&vals[1]) << "," << *((float*)&vals[2]) << ","
                      << *((float*)&vals[3]);
        if (write_vel_components) {
            csv_data_file << "," << *((float*)&vals[4]) << "," << *((float*)&vals[5]) << "," << *((float*)&vals[6]);
        }

        csv_data_file << "\n";
    }
}
