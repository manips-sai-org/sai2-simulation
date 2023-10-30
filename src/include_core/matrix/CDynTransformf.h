//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynTransformfH
#define CDynTransformfH
//---------------------------------------------------------------------------
#include "matrix/CDynMathDefn.h"
#include "matrix/CDynVector3f.h"
#include "matrix/CDynMatrix3f.h"
#include "matrix/CDynVector6f.h"
#include "matrix/CDynFramef.h"
//---------------------------------------------------------------------------
typedef double cDynTransformf[12];
//---------------------------------------------------------------------------
#define cRotM3T9(t)		(t)
#define cTransV3T9(t)	((t) + 9)
//---------------------------------------------------------------------------

inline void cSetT9T9(cDynTransformf res, const cDynTransformf t1)
{
    cSetM3M3(cRotM3T9(res), cRotM3T9(t1));
    cSetV3V3(cTransV3T9(res), cTransV3T9(t1));
}

//---------------------------------------------------------------------------

inline void cSetF7T9(cDynFramef res, const cDynTransformf t1)
{
    cSetQ4M3(deRotQ4F7(res), cRotM3T9(t1));
    cSetV3V3(deTransV3F7(res), cTransV3T9(t1));
}

//---------------------------------------------------------------------------

inline void cSetT9F7(cDynTransformf res, const cDynFramef f1)
{
    cSetM3Q4(cRotM3T9(res), deRotQ4F7(f1));
    cSetV3V3(cTransV3T9(res), deTransV3F7(f1));
}

//---------------------------------------------------------------------------

inline void cIdentityT9(cDynTransformf res)
{
    cIdentityM3(cRotM3T9(res));
    cZeroV3(cTransV3T9(res));
}

//---------------------------------------------------------------------------
//
// [r,p]v = r*v + p 
//
inline void cMulV3T9V3(cDynVector3f res, const cDynTransformf t1, const cDynVector3f v2)
{
    cMulV3M3V3(res, cRotM3T9(t1), v2);
    cAddV3V3(res, cTransV3T9(t1));
}

//---------------------------------------------------------------------------
//
// ~[r,p]*v = [~r, -(~r*p)]*v = ~r*v -~r*p = ~r*(v-p)
//
inline void cMulV3T9iV3(cDynVector3f res, const cDynTransformf t1, const cDynVector3f v2)
{
    cDynVector3f tmpV3;
    cSubV3V3V3(tmpV3, v2, cTransV3T9(t1));
    cMulV3M3tV3(res, cRotM3T9(t1), tmpV3);
}

//---------------------------------------------------------------------------
//
// [r1,p1][r2,p2] = [r1*r2, r1*p2 + p1] 
//
inline void cMulT9T9T9(cDynTransformf res, const cDynTransformf t1, const cDynTransformf t2)
{
    cMulM3M3M3(cRotM3T9(res), cRotM3T9(t1), cRotM3T9(t2));
    cMulV3M3V3(cTransV3T9(res), cRotM3T9(t1), cTransV3T9(t2));
    cAddV3V3(cTransV3T9(res), cTransV3T9(t1));
}

//---------------------------------------------------------------------------
/*
  ~[r1,p1][r2,p2] = [~r1, -(~r1*p1)][r2,p2] = [~r1*r2, ~r1*p2 - (~r1*p1)]
                  = [~r1*r2, ~r1*(p2-p1)]
*/
//---------------------------------------------------------------------------
inline void cMulT9T9iT9(cDynTransformf res, const cDynTransformf t1, const cDynTransformf t2)
{
    cDynVector3f tmpV3;
    cMulM3M3tM3(cRotM3T9(res), cRotM3T9(t1), cRotM3T9(t2));
    cSubV3V3V3(tmpV3, cTransV3T9(t2), cTransV3T9(t1));
    cMulV3M3tV3(cTransV3T9(res), cRotM3T9(t1), tmpV3);
}

//---------------------------------------------------------------------------
/*
  [r1,p1]~[r2,p2] = [r1,p1][~r2, -(~r2*p2)] = [r1*~r2, -r1*(~r2*p2) + p1]
                  = [(r1*~r2), p1-(r1*~r2)*p2]
*/
//---------------------------------------------------------------------------
inline void cMulT9T9T9i(cDynTransformf res, const cDynTransformf t1, const cDynTransformf t2)
{
    cDynVector3f tmpV3;
    cMulM3M3M3t(cRotM3T9(res), cRotM3T9(t1), cRotM3T9(t2));
    cMulV3M3V3(tmpV3, cRotM3T9(res), cTransV3T9(t2));
    cSubV3V3V3(cTransV3T9(res), cTransV3T9(t1), cTransV3T9(t2));
}

//---------------------------------------------------------------------------
//
// ~[r,p] = [~r, -(~r*p)] 
//
inline void cInvertT9T9(cDynTransformf res, const cDynTransformf t1)
{
    cTransposeM3M3(cRotM3T9(res), cRotM3T9(t1));
    cMulV3M3V3(cTransV3T9(res), cRotM3T9(res), cTransV3T9(t1));
    cNegV3V3(cTransV3T9(res), cTransV3T9(res));
}

//---------------------------------------------------------------------------

inline void cErrorV6T9T9(cDynVector6f res, const cDynTransformf t1, const cDynTransformf t2)
{
    cSubV3V3V3(res[0], cTransV3T9(t1), cTransV3T9(t2));
    cAngularErrorV3M3M3(res[1], cRotM3T9(t1), cRotM3T9(t2));
}

//---------------------------------------------------------------------------
//
// Denavit-Hartenberg
//
inline void cSetT9S4(cDynTransformf res, const double alpha, const double a, const double d, const double theta)
{
    double ca, sa, ct, st;

    ca = cDynCos(alpha);
    sa = cDynSin(alpha);
    ct = cDynCos(theta);
    st = cDynSin(theta);

    cSetM3S9(cRotM3T9(res), ct, -st, 0, st * ca, ct * ca, -sa, st * sa, ct * sa, ca);
    cSetV3S3(cTransV3T9(res), a, -sa * d, ca * d);
}

//---------------------------------------------------------------------------
//
// Screw
//
inline void cSetT9V3S2(cDynTransformf res, const cDynVector3f axis, const double pitch, const double angle)
{
    double d, c, s, v, x, y, z;

    d = angle * (1 - pitch);
    c = cDynCos(angle * pitch);
    s = cDynSin(angle * pitch);
    v = 1 - c;
    x = axis[0];
    y = axis[1];
    z = axis[2];

    cSetM3S9(cRotM3T9(res),	x * x * v + c,     x * y * v - z * s, x * z * v + y * s,
                                x * y * v + z * s, y * y * v + c,     y * z * v - x * s,
                                x * z * v - y * s, y * z * v + x * s, z * z * v + c);  

    cMulV3V3S1(cTransV3T9(res), axis, d);
}

//---------------------------------------------------------------------------
#endif  // CDynTransformfH
//---------------------------------------------------------------------------

