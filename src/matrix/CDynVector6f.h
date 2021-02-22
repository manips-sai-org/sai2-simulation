//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynVector6fH
#define CDynVector6fH
//---------------------------------------------------------------------------
#include "matrix/CDynMathDefn.h"
#include "matrix/CDynVector3f.h"
//---------------------------------------------------------------------------
typedef cDynVector3f cDynVector6f[2];
//---------------------------------------------------------------------------

inline double cDotV6V6(const cDynVector6f v1, const cDynVector6f v2)
{
    return (cDotV3V3(v1[0], v2[0]) + cDotV3V3(v1[1], v2[1]));
}

//---------------------------------------------------------------------------

inline void cZeroV6(cDynVector6f res)
{
    cZeroV3(res[0]);
    cZeroV3(res[1]);
}

//---------------------------------------------------------------------------

inline void cMulV6S1(cDynVector6f res, const double s)
{
    cMulV3S1(res[0], s);
    cMulV3S1(res[1], s);
}

//---------------------------------------------------------------------------

inline int cIsEqualV6V6(const cDynVector6f v1, const cDynVector6f v2)
{
    return (cIsEqualV3V3(v1[0], v2[0]) && cIsEqualV3V3(v1[1], v2[1]));
}

//---------------------------------------------------------------------------

inline void cNegV6V6(cDynVector6f res, const cDynVector6f v1)
{
    cNegV3V3(res[0], v1[0]);
    cNegV3V3(res[1], v1[1]);
}

//---------------------------------------------------------------------------

inline void cAddV6V6(cDynVector6f res, const cDynVector6f v1)
{
    cAddV3V3(res[0], v1[0]);
    cAddV3V3(res[1], v1[1]);
}

//---------------------------------------------------------------------------

inline void cSubV6V6(cDynVector6f res, const cDynVector6f v1)
{
    cSubV3V3(res[0], v1[0]);
    cSubV3V3(res[1], v1[1]);
}

//---------------------------------------------------------------------------

inline void cMulV6V6(cDynVector6f res, const cDynVector6f v1)
{
    cMulV3V3(res[0], v1[0]);
    cMulV3V3(res[1], v1[1]);
}

//---------------------------------------------------------------------------

inline void cMulV6V6S1(cDynVector6f res, const cDynVector6f v1, const double s)
{
    cMulV3V3S1(res[0], v1[0], s);
    cMulV3V3S1(res[1], v1[1], s);
}

//---------------------------------------------------------------------------

inline void cAddV6V6V6(cDynVector6f res, const cDynVector6f v1, const cDynVector6f v2)
{
    cAddV3V3V3(res[0], v1[0], v2[0]);
    cAddV3V3V3(res[1], v1[1], v2[1]);
}

//---------------------------------------------------------------------------

inline void cSubV6V6V6(cDynVector6f res, const cDynVector6f v1, const cDynVector6f v2)
{
    cSubV3V3V3(res[0], v1[0], v2[0]);
    cSubV3V3V3(res[1], v1[1], v2[1]);
}

//---------------------------------------------------------------------------

inline void cMulV6V6V6(cDynVector6f res, const cDynVector6f v1, const cDynVector6f v2)
{
    cMulV3V3V3(res[0], v1[0], v2[0]);
    cMulV3V3V3(res[1], v1[1], v2[1]);
}

//---------------------------------------------------------------------------
/*
   [a;b]X = [ bx , ax ; 0 , bx ]
   [ v[1] x , v[0] x ; 0 , v[1] x ] * [ v[0] , v[1] ]
   res[0] = v[1] x v[0] + v[0] x v[1]
   res[1] = v[1] x v[1]
*/
//---------------------------------------------------------------------------
inline void cCrossV6V6V6(cDynVector6f res, const cDynVector6f v1, const cDynVector6f v2)
{
    cCrossV3V3V3(res[0], v1[1], v2[0]);
    cCrossV3V3V3(res[1], v1[0], v2[1]);
    cAddV3V3(res[0], res[1]);
    cCrossV3V3V3(res[1], v1[1], v2[1]);
}

//---------------------------------------------------------------------------

inline void cSetV6S6(cDynVector6f res, const double x0, const double x1, const double x2,
                                           const double x3, const double x4, const double x5)
{
    cSetV3S3(res[0], x0, x1, x2);
    cSetV3S3(res[1], x3, x4, x5);
}

//---------------------------------------------------------------------------

inline void cSetV6V3V3(cDynVector6f res, const cDynVector3f v1, const cDynVector3f v2)
{
    cSetV3V3(res[0], v1);
    cSetV3V3(res[1], v2);
}

//---------------------------------------------------------------------------

inline void cSetV6V6(cDynVector6f res, const cDynVector6f v1)
{
    cSetV3V3(res[0], v1[0]);
    cSetV3V3(res[1], v1[1]);
}

//---------------------------------------------------------------------------

inline void cNegateV6V6(cDynVector6f res, const cDynVector6f v1)
{
    cNegateV3V3(res[0], v1[0]);
    cNegateV3V3(res[1], v1[1]);
}

//---------------------------------------------------------------------------

inline double cMagnitudeV6(const cDynVector6f v1)
{
    return cDotV6V6(v1, v1);
}

//---------------------------------------------------------------------------

inline void cNormalizeV6(cDynVector6f res)
{
    cMulV6S1(res, 1 / cMagnitudeV6(res));
}

//---------------------------------------------------------------------------
#endif  // CDynVector6fH
//---------------------------------------------------------------------------