#ifndef CHEXCEPTIONSOCKET_H
#define CHEXCEPTIONSOCKET_H

//////////////////////////////////////////////////
//
//   ChExceptionSocket.h
//
//   Custom exception class, for sockets
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
///////////////////////////////////////////////////


#include <fstream>
#include <iostream>
#include <string>
#include <time.h>

#include "core/ChException.h"
#include "core/ChLog.h"
#include "ChApiCosimulation.h"


namespace chrono 
{
namespace cosimul 
{



/// Class for exceptions that are thrown by TCP socket connections,
/// used for example when connecting with other sw for co-simulation.

class ChExceptionSocket : public ChException
{
public:

	ChExceptionSocket(int code,const std::string& what) :
				ChException(what), 
				errorCode(code) {};

		// get socket error code in thrown exception
	int getErrCode()    { return errorCode; }

	//std::string& getErrMsg() { return std::string(this->what()); }

	void response()
	{
		GetLog() << "TCP socket error: \n";
		GetLog() << "		==> error code: " << errorCode << "\n";
		GetLog() << "		==> error message: " << this->what() << "\n";
	}

private:
	int   errorCode;

};



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

#endif  // END of header