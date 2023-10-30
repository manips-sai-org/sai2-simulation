//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynVector3fH
#define CDynVector3fH
//---------------------------------------------------------------------------
#include "matrix/CDynMathDefn.h"
//---------------------------------------------------------------------------
typedef double cDynVector3f[3];
//---------------------------------------------------------------------------

inline void cSetV3S3(cDynVector3f res, const double x, const double y, const double z)
{
    res[0] = x;
    res[1] = y;
    res[2] = z;
}

//---------------------------------------------------------------------------

inline void cSetV3V3(cDynVector3f res, const cDynVector3f v1)
{
    res[0] = v1[0];
    res[1] = v1[1];
    res[2] = v1[2];
}

//---------------------------------------------------------------------------

inline void cNegateV3V3(cDynVector3f res, const cDynVector3f v1)
{
    res[0] = -v1[0];
    res[1] = -v1[1];
    res[2] = -v1[2];
}

//---------------------------------------------------------------------------

inline void cAddV3S1(cDynVector3f res, const double s)
{
    res[0] += s;
    res[1] += s;
    res[2] += s;
}

//---------------------------------------------------------------------------

inline void cMulV3S1(cDynVector3f res, const double s)
{
    res[0] *= s;
    res[1] *= s;
    res[2] *= s;
}

//---------------------------------------------------------------------------

inline int cIsEqualV3V3(const cDynVector3f v1, const cDynVector3f v2)
{
    return (v1[0] == v2[0] && v1[1] == v2[1] && v1[2] == v2[2]);
}

//---------------------------------------------------------------------------

inline double cDotV3V3(const cDynVector3f v1, const cDynVector3f v2)
{
    return (v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]);
}

//---------------------------------------------------------------------------

inline void cZeroV3(cDynVector3f res)
{
    res[0] = 0;
    res[1] = 0;
    res[2] = 0;
}

//---------------------------------------------------------------------------

inline void cNegV3V3(cDynVector3f res, const cDynVector3f v1)
{
    res[0] = -v1[0];
    res[1] = -v1[1];
    res[2] = -v1[2];
}

//---------------------------------------------------------------------------

inline void cAddV3V3(cDynVector3f res, const cDynVector3f v1)
{
    res[0] += v1[0];
    res[1] += v1[1];
    res[2] += v1[2];
}

//---------------------------------------------------------------------------

inline void cSubV3V3(cDynVector3f res, const cDynVector3f v1)
{
    res[0] -= v1[0];
    res[1] -= v1[1];
    res[2] -= v1[2];
}

//---------------------------------------------------------------------------

inline void cMulV3V3(cDynVector3f res, const cDynVector3f v1)
{
    res[0] *= v1[0];
    res[1] *= v1[1];
    res[2] *= v1[2];
}

//---------------------------------------------------------------------------

inline void cAddV3V3S1(cDynVector3f res, const cDynVector3f v1, const double s)
{
    res[0] = v1[0] + s;
    res[1] = v1[1] + s;
    res[2] = v1[2] + s;
}

//---------------------------------------------------------------------------

inline void cMulV3V3S1(cDynVector3f res, const cDynVector3f v1, const double s)
{
    res[0] = v1[0] * s;
    res[1] = v1[1] * s;
    res[2] = v1[2] * s;
}

//---------------------------------------------------------------------------

inline void cAddV3V3V3(cDynVector3f res, const cDynVector3f v1, const cDynVector3f v2)
{
    res[0] = v1[0] + v2[0];
    res[1] = v1[1] + v2[1];
    res[2] = v1[2] + v2[2];
}

//---------------------------------------------------------------------------

inline void cSubV3V3V3(cDynVector3f res, const cDynVector3f v1, const cDynVector3f v2)
{
    res[0] = v1[0] - v2[0];
    res[1] = v1[1] - v2[1];
    res[2] = v1[2] - v2[2];
}

//---------------------------------------------------------------------------

inline void cMulV3V3V3(cDynVector3f res, const cDynVector3f v1, const cDynVector3f v2)
{
    res[0] = v1[0] * v2[0];
    res[1] = v1[1] * v2[1];
    res[2] = v1[2] * v2[2];
}

//---------------------------------------------------------------------------

inline void cCrossV3V3V3(cDynVector3f res, const cDynVector3f v1, const cDynVector3f v2)
{
    res[0] = v1[1] * v2[2] - v1[2] * v2[1];
    res[1] = v1[2] * v2[0] - v1[0] * v2[2];
    res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

//---------------------------------------------------------------------------

inline double cMagnitudeV3(const cDynVector3f v1)
{
    return cDynSqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2]);
}

//---------------------------------------------------------------------------

inline void cNormalizeV3(cDynVector3f res)
{
    double mag = 1 / cDynSqrt(res[0] * res[0] + res[1] * res[1] + res[2] * res[2]);
    res[0] *= mag;
    res[1] *= mag;
    res[2] *= mag;
}

//---------------------------------------------------------------------------

inline void cMinV3V3(cDynVector3f res, const cDynVector3f v1)
{
    if (v1[0] < res[0]) res[0] = v1[0];
    if (v1[1] < res[1]) res[1] = v1[1];
    if (v1[2] < res[2]) res[2] = v1[2];
}

//---------------------------------------------------------------------------

inline void cMaxV3V3(cDynVector3f res, const cDynVector3f v1)
{
    if (v1[0] > res[0]) res[0] = v1[0];
    if (v1[1] > res[1]) res[1] = v1[1];
    if (v1[2] > res[2]) res[2] = v1[2];
}

//---------------------------------------------------------------------------
#endif // CDynVector3fH
//---------------------------------------------------------------------------
