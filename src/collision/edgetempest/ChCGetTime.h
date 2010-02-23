#ifndef PQP_GETTIME_H
#define PQP_GETTIME_H

//////////////////////////////////////////////////
//  
//   ChCGetTime.h
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#if  (defined WIN32)||(defined WIN64)
  #include <time.h>
  #include <sys/timeb.h>
  inline
  double 
  GetTime()
  {
    struct _timeb thistime;
    _ftime(&thistime);    
    return (thistime.time + thistime.millitm * 1e-3);
  }
#else
  #include <sys/time.h>
  inline
  double 
  GetTime()
  {
    struct timeval thistime;
    gettimeofday(&thistime, 0);    
    return (thistime.tv_sec + thistime.tv_usec * 1e-6);
  }

#endif

#endif
