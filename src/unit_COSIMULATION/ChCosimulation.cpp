#include "ChCosimulation.h"
#include "ChExceptionSocket.h"
#include <vector>

using namespace chrono;
using namespace chrono::cosimul;


ChCosimulation::ChCosimulation(ChSocketFramework& mframework, 
					int n_in_values,		/// number of scalar variables to receive each timestep
					int n_out_values		/// number of scalar variables to send each timestep
					)
{
	this->myServer = 0;
	this->myClient = 0;
	this->in_n =  n_in_values;
	this->out_n =  n_out_values;
	this->nport = 0;
}


ChCosimulation::~ChCosimulation()
{
	if (this->myServer) 
		delete this->myServer; 
	this->myServer = 0;
	if (this->myClient) 
		delete this->myClient; 
	this->myClient = 0;
}



bool ChCosimulation::WaitConnection(int aport)
{
	this->nport = aport;

	// a server is created, that could listen at a given port
	this->myServer = new ChSocketTCP(aport);
		
	// bind socket to server
	this->myServer->bindSocket();
	
	// wait for a client to connect (this might put the program in 
	// a long waiting state... a timeout can be useful then)
	this->myServer->listenToClient(1);
    
	std::string clientHostName;      
	this->myClient = this->myServer->acceptClient(clientHostName);  // pick up the call!

	if (!this->myClient)
		throw (ChExceptionSocket(0,"Server failed in getting the client socket"));

	return true;
}

bool ChCosimulation::SendData(double mtime, ChMatrix<double>* out_data)
{
	if (out_data->GetColumns() != 1)
		throw ChExceptionSocket(0, "Error. Sent data must be a matrix with 1 column");
	if (out_data->GetRows() != this->out_n)
		throw ChExceptionSocket(0, "Error. Sent data must be a matrix with N rows and 1 column");
	if (!myClient)
		throw ChExceptionSocket(0, "Error. Attempted 'SendData' with no connected client.");

	std::vector<char> mbuffer; // now zero length 
	ChStreamOutBinaryVector stream_out(&mbuffer); // wrap the buffer, for easy formatting

	// Serialize datas (little endian)...

		// time:
	stream_out << mtime;	
		// variables:
	for (int i=0; i < out_data->GetRows(); i++)
		stream_out << out_data->Element(i,0);

		// -----> SEND!!!
	this->myClient->SendBuffer(*stream_out.GetVector());

	return true;
}

bool ChCosimulation::ReceiveData(double& mtime, ChMatrix<double>* in_data)
{
	if (in_data->GetColumns() != 1)
		throw ChExceptionSocket(0, "Error. Received data must be a matrix with 1 column");
	if (in_data->GetRows() != this->in_n)
		throw ChExceptionSocket(0, "Error. Received data must be a matrix with N rows and 1 column");
	if (!myClient)
		throw ChExceptionSocket(0, "Error. Attempted 'ReceiveData' with no connected client.");

	// Receive from the client
	int nbytes = sizeof(double)*(this->in_n + 1);
	std::vector<char> rbuffer; 
	rbuffer.resize(nbytes); // reserve to number of expected bytes
	ChStreamInBinaryVector stream_in(&rbuffer); // wrap the buffer, for easy formatting

		// -----> RECEIVE!!!
	int numBytes = this->myClient->ReceiveBuffer(*stream_in.GetVector(), nbytes);

	// Deserialize datas (little endian)...

		// time:
	stream_in >> mtime;	
		// variables:
	for (int i=0; i < in_data->GetRows(); i++)
		stream_in >>  in_data->Element(i,0);

	return true;
}
