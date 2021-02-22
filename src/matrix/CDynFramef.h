//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynFramefH
#define CDynFramefH
//---------------------------------------------------------------------------
#include "matrix/CDynMathDefn.h"
#include "matrix/CDynVector3f.h"
#include "matrix/CDynQuaternionf.h"
#include "matrix/CDynVector6f.h"
//---------------------------------------------------------------------------
typedef double cDynFramef[7];
//---------------------------------------------------------------------------
extern const cDynFramef cDynFramefIdentity;
//---------------------------------------------------------------------------
#define deRotQ4F7(f)		(f)
#define deTransV3F7(f)		((f) + 4)
//---------------------------------------------------------------------------

inline void cSetF7F7(cDynFramef res, const cDynFramef f1)
{
    cSetQ4Q4(deRotQ4F7(res), deRotQ4F7(f1));
    cSetV3V3(deTransV3F7(res), deTransV3F7(f1));
}

//---------------------------------------------------------------------------

inline void cIdentityF7(cDynFramef res)
{
    cIdentityQ4(deRotQ4F7(res));
    cZeroV3(deTransV3F7(res));
}

//---------------------------------------------------------------------------
//
// [r,p]v = r*v + p 
//
inline void cMulV3F7V3(cDynVector3f res, const cDynFramef f1, const cDynVector3f v2)
{
    cMulV3Q4V3(res, deRotQ4F7(f1), v2);
    cAddV3V3(res, deTransV3F7(f1));
}

//--------------------------------------------------------------------------
//
// ~[r,p]*v = [~r, -(~r*p)]*v = ~r*v -~r*p = ~r*(v-p)
//
inline void cMulV3F7iV3(cDynVector3f res, const cDynFramef f1, const cDynVector3f v2)
{
    cDynVector3f tmpV3;
    cSubV3V3V3(tmpV3, v2, deTransV3F7(f1));
    cMulV3Q4iV3(res, deRotQ4F7(f1), tmpV3);
}

//---------------------------------------------------------------------------
//
// [r1,p1][r2,p2] = [r1*r2, r1*p2 + p1]
//
inline void cMulF7F7F7(cDynFramef res, const cDynFramef f1, const cDynFramef f2)
{
    cMulQ4Q4Q4(deRotQ4F7(res), deRotQ4F7(f1), deRotQ4F7(f2));
    cMulV3Q4V3(deTransV3F7(res), deRotQ4F7(f1), deTransV3F7(f2));
    cAddV3V3(deTransV3F7(res), deTransV3F7(f1));
}

//---------------------------------------------------------------------------
//
// ~[r1,p1][r2,p2] = [~r1, -(~r1*p1)][r2,p2] = [~r1*r2, ~r1*p2 - (~r1*p1)]
//                 = [~r1*r2, ~r1*(p2-p1)]
//
inline void cMulF7F7iF7(cDynFramef res, const cDynFramef f1, const cDynFramef f2)
{
    cDynVector3f tmpV3;
    cMulQ4Q4iQ4(deRotQ4F7(res), deRotQ4F7(f1), deRotQ4F7(f2));
    cSubV3V3V3(tmpV3, deTransV3F7(f2), deTransV3F7(f1));
    cMulV3Q4iV3(deTransV3F7(res), deRotQ4F7(f1), tmpV3);
}

//---------------------------------------------------------------------------
//
// ~[r,p] = [~r, -(~r*p)]
//
inline void cInvertF7F7(cDynFramef res, const cDynFramef f1)
{
    cInvertQ4Q4(deRotQ4F7(res), deRotQ4F7(f1));
    cMulV3Q4V3(deTransV3F7(res), deRotQ4F7(res), deTransV3F7(f1));
    cNegV3V3(deTransV3F7(res), deTransV3F7(res));
}

//---------------------------------------------------------------------------

inline void cErrorV6F7F7(cDynVector6f res, const cDynFramef f1, const cDynFramef f2)
{
    cSubV3V3V3(res[0], deTransV3F7(f1), deTransV3F7(f2));
    cAngularErrorV3Q4Q4(res[1], deRotQ4F7(f1), deRotQ4F7(f2));
}

//---------------------------------------------------------------------------
#endif // CDynFramefH
//---------------------------------------------------------------------------
