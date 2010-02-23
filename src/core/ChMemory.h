#ifdef	CH_MEMORY_DEBUG

#ifndef	CH_MEMORY_H
#define	CH_MEMORY_H

//////////////////////////////////////////////////
//  
//   ChMemory.h
//
//   Math functions for :
//      - MEMORY DEBUGING
//
//   This class is a memory leakage detector, based on
//   the original code of
//
//       mmgr.h  by Paul Nettle
//
//   ---------------------------------------------------------
//   Originally created on 12/22/2000 by Paul Nettle
//
//   Copyright 2000, Fluid Studios, Inc., all rights reserved.
//   ---------------------------------------------------------
//
//    To use this memory leakage detector, include this
//   header in your .cpp code (best if after the inclusion of
//   system headers, and before inclusion of your headers).
//
//    When the program exits, you'll find .log files in 
//   the program current directory, showing a list of memory
//   leakages, if you forgot to deallocate memory.
//
//    Remember to set the CH_MEMORY_DEBUG flag in 
//   compiler flags!  Also, you should use it only in 
//   debugging, not for release configuration (it slows
//   down program execution!)
// 
//     Note: you may use the memory debugger in your header
//   files as well (say you have a .h file with some 'new'
//   and 'delete' statements inside..) BUT!!! Remember that 
//   this may cause MAJOR troubles because of conflicts with
//   other headers, for example STL libraries and such (in fact,
//   lot of other .h and .cpp files in your project may load 
//   the memory-debugged header, as a cascade, then the result 
//   can be inpredictable!) 
//   Anyway, there is a quick solution: use 
//            #include"ChMemory.h" 
//   in your header file, _after_ you included other .h files,
//   then use
//            #include"ChMemorynomgr.h" 
//   at the _end_ of your .h file. Doing so, the default 'new' and
//   'delete' behaviour is restored after parsing your header,
//   and no conflicts will happen.
//
//
//   HEADER file for CHRONO,
//
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



// For systems that don't have the __FUNCTION__ variable, we can just define it here


#define	__FUNCTION__ "??"

// ---------------------------------------------------------------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------------------------------------------------------------

typedef	struct tag_au
{
	size_t		actualSize;
	size_t		reportedSize;
	void		*actualAddress;
	void		*reportedAddress;
	char		sourceFile[40];
	char		sourceFunc[40];
	unsigned int	sourceLine;
	unsigned int	allocationType;
	bool		breakOnDealloc;
	bool		breakOnRealloc;
	unsigned int	allocationNumber;
	struct tag_au	*next;
	struct tag_au	*prev;
} sAllocUnit;

typedef	struct
{
	unsigned int	totalReportedMemory;
	unsigned int	totalActualMemory;
	unsigned int	peakReportedMemory;
	unsigned int	peakActualMemory;
	unsigned int	accumulatedReportedMemory;
	unsigned int	accumulatedActualMemory;
	unsigned int	accumulatedAllocUnitCount;
	unsigned int	totalAllocUnitCount;
	unsigned int	peakAllocUnitCount;
} sMStats;

// ---------------------------------------------------------------------------------------------------------------------------------
// External constants
// ---------------------------------------------------------------------------------------------------------------------------------

extern	const	unsigned int	m_alloc_unknown;
extern	const	unsigned int	m_alloc_new;
extern	const	unsigned int	m_alloc_new_array;
extern	const	unsigned int	m_alloc_malloc;
extern	const	unsigned int	m_alloc_calloc;
extern	const	unsigned int	m_alloc_realloc;
extern	const	unsigned int	m_alloc_delete;
extern	const	unsigned int	m_alloc_delete_array;
extern	const	unsigned int	m_alloc_free;

// ---------------------------------------------------------------------------------------------------------------------------------
// Used by the macros
// ---------------------------------------------------------------------------------------------------------------------------------

void		m_setOwner(const char *file, const unsigned int line, const char *func);

// ---------------------------------------------------------------------------------------------------------------------------------
// Allocation breakpoints
// ---------------------------------------------------------------------------------------------------------------------------------

bool		&m_breakOnRealloc(void *reportedAddress);
bool		&m_breakOnDealloc(void *reportedAddress);

// ---------------------------------------------------------------------------------------------------------------------------------
// The meat of the memory tracking software
// ---------------------------------------------------------------------------------------------------------------------------------

void		*m_allocator(const char *sourceFile, const unsigned int sourceLine, const char *sourceFunc,
			     const unsigned int allocationType, const size_t reportedSize);
void		*m_reallocator(const char *sourceFile, const unsigned int sourceLine, const char *sourceFunc,
			       const unsigned int reallocationType, const size_t reportedSize, void *reportedAddress);
void		m_deallocator(const char *sourceFile, const unsigned int sourceLine, const char *sourceFunc,
			      const unsigned int deallocationType, const void *reportedAddress);

// ---------------------------------------------------------------------------------------------------------------------------------
// Utilitarian functions
// ---------------------------------------------------------------------------------------------------------------------------------

bool		m_validateAddress(const void *reportedAddress);
bool		m_validateAllocUnit(const sAllocUnit *allocUnit);
bool		m_validateAllAllocUnits();

// ---------------------------------------------------------------------------------------------------------------------------------
// Unused RAM calculations
// ---------------------------------------------------------------------------------------------------------------------------------

unsigned int	m_calcUnused(const sAllocUnit *allocUnit);
unsigned int	m_calcAllUnused();

// ---------------------------------------------------------------------------------------------------------------------------------
// Logging and reporting
// ---------------------------------------------------------------------------------------------------------------------------------

void		m_dumpAllocUnit(const sAllocUnit *allocUnit, const char *prefix = "");
void		m_dumpMemoryReport(const char *filename = "memreport.log", const bool overwrite = true);
sMStats		m_getMemoryStatistics();

// ---------------------------------------------------------------------------------------------------------------------------------
// Variations of global operators new & delete
// ---------------------------------------------------------------------------------------------------------------------------------

void	*operator new(size_t reportedSize);
void	*operator new[](size_t reportedSize);
void	*operator new(size_t reportedSize, const char *sourceFile, int sourceLine);
void	*operator new[](size_t reportedSize, const char *sourceFile, int sourceLine);
void	operator delete(void *reportedAddress);
void	operator delete[](void *reportedAddress);

#endif // ChMEMORY_H

// ---------------------------------------------------------------------------------------------------------------------------------
// Macros -- "Kids, please don't try this at home. We're trained professionals here." :)
// ---------------------------------------------------------------------------------------------------------------------------------

#include "core/ChMemorynomgr.h"
#define	new		(m_setOwner  (__FILE__,__LINE__,__FUNCTION__),false) ? NULL : new
#define	delete		(m_setOwner  (__FILE__,__LINE__,__FUNCTION__),false) ? m_setOwner("",0,"") : delete
#define	malloc(sz)	m_allocator  (__FILE__,__LINE__,__FUNCTION__,m_alloc_malloc,sz)
#define	calloc(sz)	m_allocator  (__FILE__,__LINE__,__FUNCTION__,m_alloc_calloc,sz)
#define	realloc(ptr,sz)	m_reallocator(__FILE__,__LINE__,__FUNCTION__,m_alloc_realloc,sz,ptr)
#define	free(ptr)	m_deallocator(__FILE__,__LINE__,__FUNCTION__,m_alloc_free,ptr)

// ---------------------------------------------------------------------------------------------------------------------------------
// ChMemory.h - End of file
// ---------------------------------------------------------------------------------------------------------------------------------

#endif // CH_MEMORY_DEBUG

