///////////////////////////////////////////////////////////////////////////////
//	main.cpp
//	Reads the initializes the particles, either from file or inside the code
//	
//	Related Files: collideSphereSphere.cu, collideSphereSphere.cuh
//	Input File:		initializer.txt (optional: if initialize from file)
//					This file contains the sph particles specifications. The description 
//					reads the number of particles first. The each line provides the 
//					properties of one SPH particl: 
//					position(x,y,z), radius, velocity(x,y,z), mass, \rho, pressure, mu, particle_type(rigid or fluid)
//
//	Created by Arman Pazouki
///////////////////////////////////////////////////////////////////////////////
#include <stdio.h>      /* printf */
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <cstring>
#include <sstream>

using namespace std;
int main() {
	int runNumber = 45;
	ifstream iFile;
	ofstream oFile;
	int counter = 0;
	bool fileFlag = true;
	while (fileFlag) {
		char iFileName[255];
		sprintf(iFileName, "../run%d/povFiles/rigid_flex_BCE%d.csv",runNumber, counter);

		char folderName[255];
		sprintf(folderName,"mkdir -p ../run%d/povFilesB",runNumber);
		system(folderName);
		char oFileName[255];
		sprintf(oFileName, "../run%d/povFilesB/rigid_flex_BCE%d.csv",runNumber, counter);


		ifstream iFile(iFileName);
		if (!iFile.is_open()) {
			fileFlag = false;
			return 0;
		}
		stringstream ssMyLine;
		ofstream oFile(oFileName);
		while (true) {
			string myLine;
			getline(iFile, myLine);
			if (!iFile) break;
			ssMyLine << myLine.c_str() << "," << endl;
		}
		iFile.close();
		oFile << ssMyLine.str();
		oFile.close();

		counter ++;
	}
	return 0;
}
