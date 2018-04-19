#include <fstream>
#include <iostream>

using namespace std;

int main(int argc, char** argv) {
    if (argc != 3) {
        cerr << "Usage: raw2csv raw_data_file csv_data_file" << std::endl;
    }
    ifstream raw_data_file(argv[1], ifstream::in | ifstream::binary);
    ofstream csv_data_file(argv[2], ofstream::out);
    csv_data_file << "x,y,z,vx,vy,vz,absv,nTouched\n";
    while (raw_data_file.good()) {
        int vals[8];
        raw_data_file.read((char*)vals, 8 * sizeof(int));
        csv_data_file << vals[0] << "," << vals[1] << "," << vals[2] << "," << *((float*)&vals[3]) << ","
                      << *((float*)&vals[4]) << "," << *((float*)&vals[5]) << "," << *((float*)&vals[6]) << ","
                      << (unsigned int)vals[7] << "\n";
    }
}
