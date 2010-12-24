///////////////////////////////////////////////////
//
//   ChLog.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <stdarg.h>

#include "ChLog.h"


// This check is here, but could be in any other C::E cpp source.
#ifndef CH_API_COMPILE
#error Warning! You are compiling the Chrono::Engine library, \
	so you need to define CH_API_COMPILE (add that symbol \
	to the compiler defines, for all compiled files in this unit). 
#endif 


namespace chrono
{


//
// The pointer to the global logger
//

static ChLog*     GlobalLog = NULL ;



// Functions to set/get the global logger

ChLog& GetLog()
{
	if ( GlobalLog != NULL )
        return (*GlobalLog);
	else
	{
		static ChLogConsole static_cout_logger;
		return static_cout_logger;
	}
}

void  SetLog(ChLog& new_logobject)
{
	GlobalLog = &new_logobject;
}

void  SetLogDefault()
{
	GlobalLog = NULL;
}




//
// Logger class
//

ChLog::ChLog()
{
	default_level = ChMESSAGE;
	current_level = ChMESSAGE;
}

ChLog& ChLog::operator - (eChLogLevel mnewlev)
{
	SetCurrentLevel(mnewlev);
	return *this;
}

/*
void ChLog::PrintCurTime()
{
    char dateString[52];
    SYSTEMTIME cur;
    GetSystemTime(&cur);

    sprintf(dateString,"%d/%d/%d, %d:%d:%d - \n\n", cur.wYear, cur.wMonth,
            cur.wDay, cur.wHour, cur.wMinute, cur.wSecond);

    Output(dateString);
}
*/




} // END_OF_NAMESPACE____

