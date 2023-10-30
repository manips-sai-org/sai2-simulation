//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynGlobalDefnH
#define CDynGlobalDefnH
//---------------------------------------------------------------------------
#if defined(WIN32) | defined(WIN64)
#include <malloc.h> 
#ifndef alloca
#define alloca  _alloca
#endif
#endif
//---------------------------------------------------------------------------
#ifndef NULL
#ifdef null
#define NULL null
#else // null
#define NULL 0
#endif // null
#endif // NULL
//---------------------------------------------------------------------------
typedef void (*VoidFuncPtr)(void *arg);
typedef void (*VoidNoArgFuncPtr)();
//---------------------------------------------------------------------------
extern int cDynDebugLevel;
//---------------------------------------------------------------------------
#ifdef CDYN_LIBRARY_EXPORTS
#define CDYN_LIBRARY_API __cDynclspec(dllexport)
#else
#define CDYN_LIBRARY_API __cDynclspec(dllimport)
#endif
//---------------------------------------------------------------------------
#define MATRIX_API CDYN_LIBRARY_API
//---------------------------------------------------------------------------
#define CDYN_API_ENTRY
//---------------------------------------------------------------------------
#define CDYN_ALIGNMENT 16
//---------------------------------------------------------------------------
#define CDYN_ALIGNMENT_SIZE(x) ((((x)-1)|(CDYN_ALIGNMENT-1))+1)
//---------------------------------------------------------------------------
// seb 20101222 - 64-bit tweak
#if defined(BUILD64) | defined(WIN64)
    #define CDYN_ALLOCA(n) ((char*)CDYN_ALIGNMENT_SIZE(((long int)(alloca((n)+(CDYN_ALIGNMENT-1))))))
#else
    #define CDYN_ALLOCA(n) ((char*)CDYN_ALIGNMENT_SIZE(((int)(alloca((n)+(CDYN_ALIGNMENT-1))))))
#endif
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
#endif // CDynGlobalDefnH
//---------------------------------------------------------------------------
