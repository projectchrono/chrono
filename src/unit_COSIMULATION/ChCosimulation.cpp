#include "ChCosimulation.h"

using namespace chrono;
using namespace chrono::cosimul;


ChCosimulation::ChCosimulation(ChSocketFramework& mframework, 
					int n_in_values,		/// number of scalar variables to receive each timestep
					int n_out_values		/// number of scalar variables to send each timestep
					)
{
	this->in_n =  n_in_values;
	this->out_n =  n_out_values;
}


ChCosimulation::~ChCosimulation()
{
	//
}



bool ChCosimulation::WaitConnection(int nport)
{
	return true;
}

bool ChCosimulation::SendData(ChMatrix<double>* mdata)
{
	return true;
}

bool ChCosimulation::ReceiveData(ChMatrix<double>* mdata)
{
	return true;
}
