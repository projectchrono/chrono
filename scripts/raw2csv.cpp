#include <fstream>
#include <iostream>

using namespace std;

int main(int argc, char** argv) {
    if (argc != 3) {
        cerr << "Usage: raw2csv raw_data_file csv_data_file" << std::endl;
    }
    ifstream raw_data_file(argv[1], ifstream::in | ifstream::binary);
    ofstream csv_data_file(argv[2], ofstream::out);
    csv_data_file << "x,y,z,absv\n";
    while (raw_data_file.good()) {
        int vals[4];
        raw_data_file.read((char*)vals, 4 * sizeof(float));
        csv_data_file << *((float*)&vals[0]) << "," << *((float*)&vals[1]) << "," << *((float*)&vals[2]) << ","
                      << *((float*)&vals[3]) << "\n";
    }
}
