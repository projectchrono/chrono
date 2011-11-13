#ifndef CHOPENMP_H
#define CHOPENMP_H


//////////////////////////////////////////////////
//
//   ChOpenMP.h
//
//   Interface for mutex utilities to be
//   used when doing multithreading via OpenMP
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChApiCE.h"

#ifdef _OPENMP
 # include <omp.h>
#endif


namespace chrono
{

#ifdef _OPENMP

		/// Class that wraps a 'omp_lock_t' for doing a mutex
		/// in OpenMP parallel sections.
	class ChApi ChOMPmutex
	{
	public:
	   ChOMPmutex() { omp_init_lock(&lock); }
	   ~ChOMPmutex() { omp_destroy_lock(&lock); }
	   void Lock() { omp_set_lock(&lock); }
	   void Unlock() { omp_unset_lock(&lock); }
	   
	   ChOMPmutex(const ChOMPmutex& ) { omp_init_lock(&lock); }
	   ChOMPmutex& operator= (const ChOMPmutex& ) { return *this; }
	private:
	   omp_lock_t lock;
	};

		/// Class that wraps some useful functions in OpenMP
		/// (in case no OpenMP is used, it defaults to dummy functions
		/// that do nothing)
	class ChApi ChOMPfunctions
	{
	public: 
				/// Sets the number of threads in subsequent parallel 
				/// regions, unless overridden by a 'num_threads' clause
		static void SetNumThreads(int mth) { omp_set_num_threads(mth); }

				/// Returns the number of threads in the parallel region.
		static int  GetNumThreads() { return omp_get_num_threads(); }

				/// Returns the thread number of the thread executing 
				/// within its thread team.
		static int  GetThreadNum() { return omp_get_thread_num(); }


	};
	

 #else
		/// Dummy mmutex that does nothing in case that no parallel 
		/// multithreading via OpenMP is available.
	class ChApi ChOMPmutex
	{
	public:
		void Lock() {}
		void Unlock() {}
	};

		/// Dummy functions that do nothing in case that no parallel 
		/// multithreading via OpenMP is available.
	class ChApi ChOMPfunctions
	{
	public: 
		static void SetNumThreads(int mth) { }
		static int  GetNumThreads() { return 1;}
		static int  GetThreadNum() { return 0;}
	};

 #endif
 











		/// Exception-safe wrapper to a mutex: it automatically locks the 
		/// mutex as soon as the wrapper is created, and releases the 
		/// mutex when the wrapper is deleted (you simply put the wrapper in
		/// a code section delimited by {} parentheses, so it is deleted
		/// by the compiler when exiting the scope of the section or in case
		/// of premature exit because of an exception throw)
 struct ChOMPscopedLock
 {
   explicit ChOMPscopedLock(ChOMPmutex& m) : 
						mut(m), 
						locked(true) 
		{ 
			mut.Lock(); 
		}

   ~ChOMPscopedLock() 
		{ 
			Unlock(); 
		}

   void Unlock() 
		{ 
			if(!locked) return; 
			locked=false; 
			mut.Unlock(); 
		}

   void LockAgain() 
		{ 
			if(locked) return; 
			mut.Lock(); 
			locked=true; 
		}

 private:
	ChOMPmutex& mut;
	bool locked;

 private: // trick to prevent copying the scoped lock.
	void operator=(const ChOMPscopedLock&);
	ChOMPscopedLock(const ChOMPscopedLock&);
 };




};  // END_OF_NAMESPACE____

#endif
