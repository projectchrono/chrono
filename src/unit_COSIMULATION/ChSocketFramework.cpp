#include "ChSocketFramework.h"
#include "ChExceptionSocket.h"

using namespace chrono;
using namespace chrono::cosimul;

ChSocketFramework::ChSocketFramework()
{
	#ifdef WINDOWS_XP

		// Initialize the winsock library
		WSADATA wsaData;
		
		if (WSAStartup(0x101, &wsaData))
		{
			throw chrono::cosimul::ChExceptionSocket(0,"Error: calling WSAStartup()");
		}

	#endif
}


ChSocketFramework::~ChSocketFramework()
{
	#ifdef WINDOWS_XP

		if (WSACleanup())
		{
			throw ChExceptionSocket(0,"Error: calling WSACleanup()"); 
        }

	#endif
}

