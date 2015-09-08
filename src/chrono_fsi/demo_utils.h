#ifndef DEMOS_UTILS_H
#define DEMOS_UTILS_H

#include <iostream>

#include "core/ChStream.h"
#include "chrono_parallel/physics/ChSystemParallel.h"

// =============================================================================
// Utility function for displaying an ASCII progress bar for the quantity x
// which must be a value between 0 and n. The width 'w' represents the number
// of '=' characters corresponding to 100%.

static inline void progressbar(unsigned int x, unsigned int n, unsigned int w = 50) {
  if ((x != n) && (x % (n / 100 + 1) != 0))
    return;

  float ratio = x / (float)n;
  int c = ratio * w;

  std::cout << std::setw(3) << (int)(ratio * 100) << "% [";
  for (int x = 0; x < c; x++)
    std::cout << "=";
  for (int x = c; x < w; x++)
    std::cout << " ";
  std::cout << "]\r" << std::flush;
}

// =============================================================================
// Utility function to print to console a few important step statistics

static inline void TimingOutput(chrono::ChSystem* mSys, chrono::ChStreamOutAsciiFile* ofile = NULL) {
  double TIME = mSys->GetChTime();
  double STEP = mSys->GetTimerStep();
  double BROD = mSys->GetTimerCollisionBroad();
  double NARR = mSys->GetTimerCollisionNarrow();
  double LCP = mSys->GetTimerLcp();
  double UPDT = mSys->GetTimerUpdate();
  double RESID = 0;
  int REQ_ITS = 0;
  int BODS = mSys->GetNbodies();
  int CNTC = mSys->GetNcontacts();
  if (chrono::ChSystemParallel* parallel_sys = dynamic_cast<chrono::ChSystemParallel*>(mSys)) {
    RESID = ((chrono::ChLcpSolverParallel*)(mSys->GetLcpSolverSpeed()))->GetResidual();
    REQ_ITS = ((chrono::ChLcpSolverParallel*)(mSys->GetLcpSolverSpeed()))->GetTotalIterations();
    BODS = parallel_sys->GetNbodies();
    CNTC = parallel_sys->GetNcontacts();
  }

  if (ofile) {
    char buf[200];
    sprintf(buf, "%8.5f  %7.4f  %7.4f  %7.4f  %7.4f  %7.4f  %7d  %7d  %7d  %7.4f\n", TIME, STEP, BROD, NARR, LCP, UPDT,
            BODS, CNTC, REQ_ITS, RESID);
    *ofile << buf;
  }

  printf("   %8.5f | %7.4f | %7.4f | %7.4f | %7.4f | %7.4f | %7d | %7d | %7d | %7.4f\n", TIME, STEP, BROD, NARR, LCP,
         UPDT, BODS, CNTC, REQ_ITS, RESID);
}

#endif
