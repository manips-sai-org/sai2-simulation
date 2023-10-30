//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynMatrix6fH
#define CDynMatrix6fH
//---------------------------------------------------------------------------
#include "matrix/CDynMathDefn.h"
#include "matrix/CDynVector3f.h"
#include "matrix/CDynMatrix3f.h"
#include "matrix/CDynVector6f.h"
//---------------------------------------------------------------------------
typedef cDynMatrix3f cDynMatrix6f[4];
//---------------------------------------------------------------------------

inline void cSetM6M3M3M3M3(cDynMatrix6f res, const cDynMatrix3f m11,const cDynMatrix3f m12,
                                                 const cDynMatrix3f m21,const cDynMatrix3f m22)
{
    cSetM3M3(res[0], m11);
    cSetM3M3(res[1], m12);
    cSetM3M3(res[2], m21);
    cSetM3M3(res[3], m22);
}

//---------------------------------------------------------------------------

inline void cDiagonalM6V6(cDynMatrix6f res, const cDynVector6f v1)
{
    cDiagonalM3V3(res[0], v1[0]);
    cDiagonalM3V3(res[3], v1[1]);
}

//---------------------------------------------------------------------------

inline void cDiagonalV6M6(cDynVector6f res, const cDynMatrix6f m1)
{
    cDiagonalV3M3(res[0], m1[0]);
    cDiagonalV3M3(res[1], m1[3]);
}

//---------------------------------------------------------------------------

inline void cSetM6M6(cDynMatrix6f res, const cDynMatrix6f m1)
{
    cSetM3M3(res[0], m1[0]);
    cSetM3M3(res[1], m1[1]);
    cSetM3M3(res[2], m1[2]);
    cSetM3M3(res[3], m1[3]);
}

//---------------------------------------------------------------------------

inline void cZeroM6(cDynMatrix6f res)
{
    cZeroM3(res[0]);
    cZeroM3(res[1]);
    cZeroM3(res[2]);
    cZeroM3(res[3]);
}

//---------------------------------------------------------------------------

inline void cIdentityM6(cDynMatrix6f res)
{
    cIdentityM3(res[0]);
    cZeroM3(res[1]);
    cZeroM3(res[2]);
    cIdentityM3(res[3]);
}

//---------------------------------------------------------------------------

inline void cNegateM6M6(cDynMatrix6f res, const cDynMatrix6f m1)
{
    cNegateM3M3(res[0], m1[0]);
    cNegateM3M3(res[1], m1[1]);
    cNegateM3M3(res[2], m1[2]);
    cNegateM3M3(res[3], m1[3]);
}

//---------------------------------------------------------------------------

inline void cAddM6M6M6(cDynMatrix6f res, const cDynMatrix6f m1, const cDynMatrix6f m2)
{
    cAddM3M3M3(res[0], m1[0], m2[0]);
    cAddM3M3M3(res[1], m1[1], m2[1]);
    cAddM3M3M3(res[2], m1[2], m2[2]);
    cAddM3M3M3(res[3], m1[3], m2[3]);
}

//---------------------------------------------------------------------------

inline void cSubM6M6M6(cDynMatrix6f res, const cDynMatrix6f m1, const cDynMatrix6f m2)
{
    cSubM3M3M3(res[0], m1[0], m2[0]);
    cSubM3M3M3(res[1], m1[1], m2[1]);
    cSubM3M3M3(res[2], m1[2], m2[2]);
    cSubM3M3M3(res[3], m1[3], m2[3]);
}

//---------------------------------------------------------------------------

inline void cMulM6M6S1(cDynMatrix6f res, const cDynMatrix6f m1, const double s)
{
    cMulM3M3S1(res[0], m1[0], s);
    cMulM3M3S1(res[1], m1[1], s);
    cMulM3M3S1(res[2], m1[2], s);
    cMulM3M3S1(res[3], m1[3], s);
}

//---------------------------------------------------------------------------

inline void cAddM6M6(cDynMatrix6f res, const cDynMatrix6f m1)
{
    cAddM3M3(res[0], m1[0]);
    cAddM3M3(res[1], m1[1]);
    cAddM3M3(res[2], m1[2]);
    cAddM3M3(res[3], m1[3]);
}

//---------------------------------------------------------------------------

inline void cSubM6M6(cDynMatrix6f res, const cDynMatrix6f m1)
{
    cSubM3M3(res[0], m1[0]);
    cSubM3M3(res[1], m1[1]);
    cSubM3M3(res[2], m1[2]);
    cSubM3M3(res[3], m1[3]);
}

//---------------------------------------------------------------------------

inline void cMulM6S1(cDynMatrix6f res, const double s)
{
    cMulM3S1(res[0], s);
    cMulM3S1(res[1], s);
    cMulM3S1(res[2], s);
    cMulM3S1(res[3], s);
}

//---------------------------------------------------------------------------

inline void cTransposeM6M6(cDynMatrix6f res, const cDynMatrix6f m1)
{
    cTransposeM3M3(res[0], m1[0]);
    cTransposeM3M3(res[1], m1[2]);
    cTransposeM3M3(res[2], m1[1]);
    cTransposeM3M3(res[3], m1[3]);
}

//---------------------------------------------------------------------------

inline void cMulV6M6V6(cDynVector6f res, const cDynMatrix6f m1, const cDynVector6f v2)
{
    cDynVector3f tmpV3;

    cMulV3M3V3(res[0], m1[0], v2[0]);
    cMulV3M3V3(tmpV3, m1[1], v2[1]);
    cAddV3V3(res[0], tmpV3);

    cMulV3M3V3(res[1], m1[2], v2[0]);
    cMulV3M3V3(tmpV3, m1[3], v2[1]);
    cAddV3V3(res[1], tmpV3);
}

//---------------------------------------------------------------------------

inline void cMulM6V6V6t(cDynMatrix6f res, const cDynVector6f v1, const cDynVector6f v2)
{
    cMulM3V3V3t(res[0], v1[0], v2[0]);
    cMulM3V3V3t(res[1], v1[0], v2[1]);
    cMulM3V3V3t(res[2], v1[1], v2[0]);
    cMulM3V3V3t(res[3], v1[1], v2[1]);
}

//---------------------------------------------------------------------------

inline void cSetM6V6x(cDynMatrix6f res, const cDynVector6f v1)
{
    cSetM3V3x(res[0], v1[1]);
    cSetM3V3x(res[1], v1[0]);
    cZeroM3(res[2]);
    cSetM3M3(res[3],res[0]);
}

//---------------------------------------------------------------------------
//
// res = m1^T * v
//
inline void cMulV6M6tV6(cDynVector6f res, const cDynMatrix6f m1, const cDynVector6f v2)
{
    cDynVector3f tmpV3;

    cMulV3M3tV3(res[0], m1[0], v2[0]);
    cMulV3M3tV3(tmpV3, m1[2], v2[1]);
    cAddV3V3(res[0], tmpV3);
           
    cMulV3M3tV3(res[1], m1[1], v2[0]);
    cMulV3M3tV3(tmpV3, m1[3], v2[1]);
    cAddV3V3(res[1], tmpV3);
}

//---------------------------------------------------------------------------
//
// [A0 A1;A2 A3]*[B0 B1;B2 B3] = [A0*B0+A1*B2 A0*B1+A1*B3;
//                                A2*B0+A3*B2 A2*B1+A3*B3]
//
inline void cMulM6M6M6(cDynMatrix6f res, const cDynMatrix6f m1, const cDynMatrix6f m2)
{
    cDynMatrix3f tmpM3;

    cMulM3M3M3(res[0], m1[0], m2[0]);
    cMulM3M3M3(tmpM3, m1[1], m2[2]);
    cAddM3M3(res[0], tmpM3);

    cMulM3M3M3(res[1], m1[0], m2[1]);
    cMulM3M3M3(tmpM3, m1[1], m2[3]);
    cAddM3M3(res[1], tmpM3);

    cMulM3M3M3(res[2], m1[2], m2[0]);
    cMulM3M3M3(tmpM3, m1[3], m2[2]);
    cAddM3M3(res[2], tmpM3);

    cMulM3M3M3(res[3], m1[2], m2[1]);
    cMulM3M3M3(tmpM3, m1[3], m2[3]);
    cAddM3M3(res[3], tmpM3);
}

//---------------------------------------------------------------------------
//
// res = m1^T * m2
//
inline void cMulM6M6tM6(cDynMatrix6f res, const cDynMatrix6f m1, const cDynMatrix6f m2)
{
    cDynMatrix3f tmpM3;

    cMulM3M3tM3(res[0], m1[0], m2[0]);
    cMulM3M3tM3(tmpM3, m1[2], m2[2]);
    cAddM3M3(res[0], tmpM3);

    cMulM3M3tM3(res[1], m1[0], m2[1]);
    cMulM3M3tM3(tmpM3, m1[2], m2[3]);
    cAddM3M3(res[1], tmpM3);

    cMulM3M3tM3(res[2], m1[1], m2[0]);
    cMulM3M3tM3(tmpM3, m1[3], m2[2]);
    cAddM3M3(res[2], tmpM3);

    cMulM3M3tM3(res[3], m1[1], m2[1]);
    cMulM3M3tM3(tmpM3, m1[3], m2[3]);
    cAddM3M3(res[3], tmpM3);
}

//---------------------------------------------------------------------------
//
// res = m1 * m2^T 
//
inline void cMulM6M6M6t(cDynMatrix6f res, const cDynMatrix6f m1, const cDynMatrix6f m2)
{
    cDynMatrix3f tmpM3;

    cMulM3M3M3t(res[0], m1[0], m2[0]);
    cMulM3M3M3t(tmpM3, m1[1], m2[1]);
    cAddM3M3(res[0], tmpM3);

    cMulM3M3M3t(res[1], m1[0], m2[2]);
    cMulM3M3M3t(tmpM3, m1[1], m2[3]);
    cAddM3M3(res[1], tmpM3);

    cMulM3M3M3t(res[2], m1[2], m2[0]);
    cMulM3M3M3t(tmpM3, m1[3], m2[1]);
    cAddM3M3(res[2], tmpM3);

    cMulM3M3M3t(res[3], m1[2], m2[2]);
    cMulM3M3M3t(tmpM3, m1[3], m2[3]);
    cAddM3M3(res[3], tmpM3);
}

//---------------------------------------------------------------------------
#endif /* CDynMatrix6fH */
//---------------------------------------------------------------------------