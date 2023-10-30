//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynMathDefnH
#define CDynMathDefnH
//---------------------------------------------------------------------------
#include <math.h>
//---------------------------------------------------------------------------
#if defined(WIN32) | defined(WIN64)
#include <float.h>
#elif defined(LINUX)
// #include <bits/nan.h>
#include <math.h>
#endif
//---------------------------------------------------------------------------
#ifndef NULL // XXX
#ifdef  __cplusplus
#define NULL    0
#else
#define NULL    ((void *)0)
#endif
#endif
//---------------------------------------------------------------------------
#ifndef FLT_MAX
#define FLT_MAX	1e99
#endif
#ifndef NAN
#define NAN	((double)(0x8000000000000))
#endif
//---------------------------------------------------------------------------
#define CDYN_THOUSAND        1000
#define CDYN_MILLION         (CDYN_THOUSAND*CDYN_THOUSAND)
#define CDYN_BILLION         (CDYN_MILLION*CDYN_THOUSAND)
//---------------------------------------------------------------------------
#define CDYN_LB_TO_KG        (0.45359237) //=(1.0/2.2046226)
//---------------------------------------------------------------------------
#define CDYN_INCH_TO_METER   (0.0254)
//---------------------------------------------------------------------------
#define CDYN_GRAVITY_CONSTANT (9.81)

#define CDYN_BITS_PER_BYTE  8
#define CDYN_BYTES_PER_WORD (sizeof(unsigned int))
//---------------------------------------------------------------------------
#undef CDYN_DOUBLE_SUPPORT   // machine supports double precision
#undef CDYN_DOUBLE_PRECISION // double precision for computation
//---------------------------------------------------------------------------
//#define CDYN_EPSILON         0.00001f
//#define CDYN_COS_THRESHHOLD (1.0f - CDYN_EPSILON)
//
typedef enum {CDYN_AXIS_X=0,
              CDYN_AXIS_Y=1,
              CDYN_AXIS_Z=2} cDynAxis;
//---------------------------------------------------------------------------
typedef double cDynTime;
//const cDynTime CDYN_TIME_UNINITIALIZED = -1.0;
//const cDynTime CDYN_TIME_UNASSOCIATED = -2.0;
//const cDynTime CDYN_TIME_MAX = 1e99;
//---------------------------------------------------------------------------
//const double CDYN_MAXDEFLOAT = (double)1e33;
//---------------------------------------------------------------------------
#define CDYN_TIME_UNINITIALIZED	(-1.0)
#define CDYN_TIME_UNASSOCIATED  (-2.0)
#define CDYN_TIME_MAX           (1e99)
//---------------------------------------------------------------------------
#define CDYN_MAXDEFLOAT	((double)1e33)
//---------------------------------------------------------------------------
#if !defined(WIN32) & !defined(WIN64)
#define __inline        inline
#define __forceinline   inline
#include <cstdlib>
#endif
//---------------------------------------------------------------------------
#define	cDynSqrt(x)     ((double)sqrt(x))
#define cDynCos(x)      ((double)cos(x))
#define cDynSin(x)      ((double)sin(x))
#define cDynAcos(x)     ((double)acos(x))
#define cDynAsin(x)     ((double)asin(x))
#define cDynAtan2(x,y)  ((double)atan2(x,y))
#define cDynFabs(x)     ((double)fabs(x))
//---------------------------------------------------------------------------
#ifndef M_PI
#define M_PI        3.14159265358979323846      // pi
#endif
#ifndef M_PI_2
#define M_PI_2      1.57079632679489661923      // pi/2
#endif
#ifndef M_PI_4
#define M_PI_4      0.78539816339744830962      // pi/4
#endif
#ifndef M_SQRT2
#define M_SQRT2     ((double)1.414213562373)    // sqrt(2)
#endif
//---------------------------------------------------------------------------
#endif // CDynMathDefnH
//---------------------------------------------------------------------------
