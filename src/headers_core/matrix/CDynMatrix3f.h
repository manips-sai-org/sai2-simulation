//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynMatrix3fH
#define CDynMatrix3fH
//---------------------------------------------------------------------------
#include "matrix/CDynMathDefn.h"
#include "matrix/CDynVector3f.h"
#include "matrix/CDynQuaternionf.h"
//---------------------------------------------------------------------------
typedef double cDynMatrix3f[9];
//---------------------------------------------------------------------------
#define cRowV3M3S1(m,row)	((m) + (row) * 3)
//---------------------------------------------------------------------------
void cSetQ4M3(cDynQuaternionf res, const cDynMatrix3f m1);

// Euler XYZ
void cSetV3M3xyz(cDynVector3f res, const cDynMatrix3f m1);
void cLUdecomposeM3M3(cDynMatrix3f lu, const cDynMatrix3f m1);
void cBackSubstituteV3M3V3(cDynVector3f x, const cDynMatrix3f lu, const cDynVector3f y);
void cSetM3S2(cDynMatrix3f res, const int axis, const double angle);

//---------------------------------------------------------------------------

inline void cSetM3S9(cDynMatrix3f res,
                           const double a0, const double a1, const double a2,
                           const double a3, const double a4, const double a5,
                           const double a6, const double a7, const double a8)
{
    res[0] = a0;	res[1] = a1;	res[2] = a2;
    res[3] = a3;	res[4] = a4;	res[5] = a5;
    res[6] = a6;	res[7] = a7;	res[8] = a8;
}

//---------------------------------------------------------------------------

inline void cZeroM3(cDynMatrix3f res)
{
    res[0] = 0;	res[1] = 0;	res[2] = 0;
    res[3] = 0;	res[4] = 0;	res[5] = 0;
    res[6] = 0;	res[7] = 0;	res[8] = 0;
}

//---------------------------------------------------------------------------

inline void cIdentityM3(cDynMatrix3f res)
{
    res[0] = 1;	res[1] = 0;	res[2] = 0;
    res[3] = 0;	res[4] = 1;	res[5] = 0;
    res[6] = 0;	res[7] = 0;	res[8] = 1;
}

inline void cMulM3S1(cDynMatrix3f res, const double s)
{
    res[0] *= s;	res[1] *= s;	res[2] *= s;
    res[3] *= s;	res[4] *= s;	res[5] *= s;
    res[6] *= s;	res[7] *= s;	res[8] *= s;
}

//---------------------------------------------------------------------------

inline void cNegateM3M3(cDynMatrix3f res, const cDynMatrix3f m1)
{
    res[0] = -m1[0];	res[1] = -m1[1];	res[2] = -m1[2];
    res[3] = -m1[3];	res[4] = -m1[4];	res[5] = -m1[5];
    res[6] = -m1[6];	res[7] = -m1[7];	res[8] = -m1[8];
}

//---------------------------------------------------------------------------

inline void cSetM3M3(cDynMatrix3f res, const cDynMatrix3f m1)
{
    res[0] = m1[0];	res[1] = m1[1];	res[2] = m1[2];
    res[3] = m1[3];	res[4] = m1[4];	res[5] = m1[5];
    res[6] = m1[6];	res[7] = m1[7];	res[8] = m1[8];
}

//---------------------------------------------------------------------------

inline void cAddM3M3(cDynMatrix3f res, const cDynMatrix3f m1)
{
    res[0] += m1[0];	res[1] += m1[1];	res[2] += m1[2];
    res[3] += m1[3];	res[4] += m1[4];	res[5] += m1[5];
    res[6] += m1[6];	res[7] += m1[7];	res[8] += m1[8];
}

//---------------------------------------------------------------------------

inline void cSubM3M3(cDynMatrix3f res, const cDynMatrix3f m1)
{
    res[0] -= m1[0];	res[1] -= m1[1];	res[2] -= m1[2];
    res[3] -= m1[3];	res[4] -= m1[4];	res[5] -= m1[5];
    res[6] -= m1[6];	res[7] -= m1[7];	res[8] -= m1[8];
}

//---------------------------------------------------------------------------

inline void cAddM3M3M3(cDynMatrix3f res, const cDynMatrix3f m1, const cDynMatrix3f m2)
{
    res[0] = m1[0] + m2[0];	res[1] = m1[1] + m2[1];	res[2] = m1[2] + m2[2];
    res[3] = m1[3] + m2[3];	res[4] = m1[4] + m2[4];	res[5] = m1[5] + m2[5];
    res[6] = m1[6] + m2[6];	res[7] = m1[7] + m2[7];	res[8] = m1[8] + m2[8];
}

//---------------------------------------------------------------------------

inline void cSubM3M3M3(cDynMatrix3f res, const cDynMatrix3f m1, const cDynMatrix3f m2)
{
    res[0] = m1[0] - m2[0];	res[1] = m1[1] - m2[1];	res[2] = m1[2] - m2[2];
    res[3] = m1[3] - m2[3];	res[4] = m1[4] - m2[4];	res[5] = m1[5] - m2[5];
    res[6] = m1[6] - m2[6];	res[7] = m1[7] - m2[7];	res[8] = m1[8] - m2[8];
}

//---------------------------------------------------------------------------

inline void cMulM3M3S1(cDynMatrix3f res, const cDynMatrix3f m1, const double s)
{
    res[0] = m1[0] * s;	res[1] = m1[1] * s;	res[2] = m1[2] * s;
    res[3] = m1[3] * s;	res[4] = m1[4] * s;	res[5] = m1[5] * s;
    res[6] = m1[6] * s;	res[7] = m1[7] * s;	res[8] = m1[8] * s;
}

//---------------------------------------------------------------------------

inline void cMulM3M3M3(cDynMatrix3f res, const cDynMatrix3f m1, const cDynMatrix3f m2)
{
    res[0] = m1[0] * m2[0] + m1[1] * m2[3] + m1[2] * m2[6];
    res[1] = m1[0] * m2[1] + m1[1] * m2[4] + m1[2] * m2[7];
    res[2] = m1[0] * m2[2] + m1[1] * m2[5] + m1[2] * m2[8];
    res[3] = m1[3] * m2[0] + m1[4] * m2[3] + m1[5] * m2[6];
    res[4] = m1[3] * m2[1] + m1[4] * m2[4] + m1[5] * m2[7];
    res[5] = m1[3] * m2[2] + m1[4] * m2[5] + m1[5] * m2[8];
    res[6] = m1[6] * m2[0] + m1[7] * m2[3] + m1[8] * m2[6];
    res[7] = m1[6] * m2[1] + m1[7] * m2[4] + m1[8] * m2[7];
    res[8] = m1[6] * m2[2] + m1[7] * m2[5] + m1[8] * m2[8];
}

//---------------------------------------------------------------------------
//
// res = m1^T * m2
//
inline void cMulM3M3tM3(cDynMatrix3f res, const cDynMatrix3f m1, const cDynMatrix3f m2)
{
    res[0] = m1[0] * m2[0] + m1[3] * m2[3] + m1[6] * m2[6];
    res[1] = m1[0] * m2[1] + m1[3] * m2[4] + m1[6] * m2[7];
    res[2] = m1[0] * m2[2] + m1[3] * m2[5] + m1[6] * m2[8];
    res[3] = m1[1] * m2[0] + m1[4] * m2[3] + m1[7] * m2[6];
    res[4] = m1[1] * m2[1] + m1[4] * m2[4] + m1[7] * m2[7];
    res[5] = m1[1] * m2[2] + m1[4] * m2[5] + m1[7] * m2[8];
    res[6] = m1[2] * m2[0] + m1[5] * m2[3] + m1[8] * m2[6];
    res[7] = m1[2] * m2[1] + m1[5] * m2[4] + m1[8] * m2[7];
    res[8] = m1[2] * m2[2] + m1[5] * m2[5] + m1[8] * m2[8];
}

//---------------------------------------------------------------------------
//
// res = m1 * m2^T
//
inline void cMulM3M3M3t(cDynMatrix3f res, const cDynMatrix3f m1, const cDynMatrix3f m2)
{
    res[0] = m1[0] * m2[0] + m1[1] * m2[1] + m1[2] * m2[2];
    res[1] = m1[0] * m2[3] + m1[1] * m2[4] + m1[2] * m2[5];
    res[2] = m1[0] * m2[6] + m1[1] * m2[7] + m1[2] * m2[8];
    res[3] = m1[3] * m2[0] + m1[4] * m2[1] + m1[5] * m2[2];
    res[4] = m1[3] * m2[3] + m1[4] * m2[4] + m1[5] * m2[5];
    res[5] = m1[3] * m2[6] + m1[4] * m2[7] + m1[5] * m2[8];
    res[6] = m1[6] * m2[0] + m1[7] * m2[1] + m1[8] * m2[2];
    res[7] = m1[6] * m2[3] + m1[7] * m2[4] + m1[8] * m2[5];
    res[8] = m1[6] * m2[6] + m1[7] * m2[7] + m1[8] * m2[8];
}

//---------------------------------------------------------------------------

inline void cMulV3M3V3(cDynVector3f res, const cDynMatrix3f m1,const cDynVector3f v2)
{
    res[0] = m1[0] * v2[0] + m1[1] * v2[1] + m1[2] * v2[2];
    res[1] = m1[3] * v2[0] + m1[4] * v2[1] + m1[5] * v2[2];
    res[2] = m1[6] * v2[0] + m1[7] * v2[1] + m1[8] * v2[2];
}

//---------------------------------------------------------------------------
//
// res = m2^T * v2
//
inline void cMulV3M3tV3(cDynVector3f res, const cDynMatrix3f m1, const cDynVector3f v2)
{
    res[0] = m1[0] * v2[0] + m1[3] * v2[1] + m1[6] * v2[2];
    res[1] = m1[1] * v2[0] + m1[4] * v2[1] + m1[7] * v2[2];
    res[2] = m1[2] * v2[0] + m1[5] * v2[1] + m1[8] * v2[2];
}

//---------------------------------------------------------------------------
//
// res = m1 * (v2 x)
//
inline void cMulM3M3V3x(cDynMatrix3f res, const cDynMatrix3f m1, const cDynVector3f v2)
{
    res[0] =				  m1[1] * v2[2] - m1[2] * v2[1];
    res[1] = -m1[0] * v2[2]                 + m1[2] * v2[0];
    res[2] =  m1[0] * v2[1]	- m1[1] * v2[0];
    res[3] =                  m1[4] * v2[2] - m1[5] * v2[1];
    res[4] = -m1[3] * v2[2]                 + m1[5] * v2[0];
    res[5] =  m1[3] * v2[1]	- m1[4] * v2[0];
    res[6] =                  m1[7] * v2[2] - m1[8] * v2[1];
    res[7] = -m1[6] * v2[2]                 + m1[8] * v2[0];
    res[8] =  m1[6] * v2[1]	- m1[7] * v2[0];
}

//---------------------------------------------------------------------------
//
// [0 1 2;3 4 5;6 7 8]^T = [0 3 6;1 4 7;2 5 8]
//
inline void cTransposeM3M3(cDynMatrix3f res, const cDynMatrix3f m1)
{
    res[0] = m1[0];    res[1] = m1[3];    res[2] = m1[6];
    res[3] = m1[1];    res[4] = m1[4];    res[5] = m1[7];
    res[6] = m1[2];    res[7] = m1[5];    res[8] = m1[8];
}

//---------------------------------------------------------------------------

inline void cColumnV3M3S1(cDynVector3f res, const cDynMatrix3f m1, const int col)
{
    res[0] = m1[col];
    res[1] = m1[col+3];
    res[2] = m1[col+6];
}

//---------------------------------------------------------------------------

inline void cDiagonalM3V3(cDynMatrix3f res, const cDynVector3f v1)
{
    res[0] = v1[0];	res[1] = 0;		res[2] = 0;
    res[3] = 0;		res[4] = v1[1];	res[5] = 0;
    res[6] = 0;		res[7] = 0;		res[8] = v1[2];
}

//---------------------------------------------------------------------------

inline void cDiagonalM3S3(cDynMatrix3f res, const double x, const double y, const double z)
{
    res[0] = x;		res[1] = 0;		res[2] = 0;
    res[3] = 0;		res[4] = y;		res[5] = 0;
    res[6] = 0;		res[7] = 0;		res[8] = z;
}

//---------------------------------------------------------------------------

inline void cDiagonalV3M3(cDynVector3f res, const cDynMatrix3f m1)
{
    res[0] = m1[0];
    res[1] = m1[4];
    res[2] = m1[8];
}

//---------------------------------------------------------------------------

inline double cDetM3(const cDynMatrix3f m1)
{
    return (  m1[0] * (m1[4] * m1[8] - m1[5] * m1[7])
            - m1[1] * (m1[3] * m1[8] - m1[5] * m1[6])
            + m1[2] * (m1[3] * m1[7] - m1[4] * m1[6]));
}

//---------------------------------------------------------------------------

inline void cInvertDetSPDM3M3(cDynMatrix3f res, const cDynMatrix3f m1)
{
     double ood = 1 / cDetM3(m1);
    
     res[0] =  ood * (m1[4] * m1[8] - m1[5] * m1[7]);
     res[1] = -ood * (m1[1] * m1[8] - m1[2] * m1[7]);
     res[2] =  ood * (m1[1] * m1[5] - m1[2] * m1[4]);
     res[3] =  res[1];
     res[4] =  ood * (m1[0] * m1[8] - m1[2] * m1[6]);
     res[5] = -ood * (m1[0] * m1[5] - m1[2] * m1[3]);
     res[6] =  res[2];
     res[7] =  res[5];
     res[8] =  ood * (m1[0] * m1[4] - m1[1] * m1[3]);
}

//---------------------------------------------------------------------------

inline void cInvertDetM3M3(cDynMatrix3f res, const cDynMatrix3f m1)
{
     double ood = 1 / cDetM3(m1);
    
     res[0] =  ood * (m1[4] * m1[8] - m1[5] * m1[7]);
     res[1] = -ood * (m1[1] * m1[8] - m1[2] * m1[7]);
     res[2] =  ood * (m1[1] * m1[5] - m1[2] * m1[4]);
     res[3] = -ood * (m1[3] * m1[8] - m1[5] * m1[6]);
     res[4] =  ood * (m1[0] * m1[8] - m1[2] * m1[6]);
     res[5] = -ood * (m1[0] * m1[5] - m1[2] * m1[3]);
     res[6] =  ood * (m1[3] * m1[7] - m1[4] * m1[6]);
     res[7] = -ood * (m1[0] * m1[7] - m1[1] * m1[6]);
     res[8] =  ood * (m1[0] * m1[4] - m1[1] * m1[3]);
}

//---------------------------------------------------------------------------
//
// res = m1 - m2
//
inline void cAngularErrorV3M3M3(cDynVector3f res, const cDynMatrix3f m1, const cDynMatrix3f m2)
{
    res[0] = -0.5*((m1[3] * m2[6] + m1[4] * m2[7] + m1[5] * m2[8]) 
                 - (m1[6] * m2[3] + m1[7] * m2[4] + m1[8] * m2[5]));
    res[1] = -0.5*((m1[6] * m2[0] + m1[7] * m2[1] + m1[8] * m2[2])
                 - (m1[0] * m2[6] + m1[1] * m2[7] + m1[2] * m2[8]));
    res[2] = -0.5*((m1[0] * m2[3] + m1[1] * m2[4] + m1[2] * m2[5])
                 - (m1[3] * m2[0] + m1[4] * m2[1] + m1[5] * m2[2]));
}

//---------------------------------------------------------------------------
//
// 27 mult + 12 add
// Intro. to Robotics by J. Craig: p. 55
//
inline void cSetM3Q4(cDynMatrix3f res, const cDynQuaternionf q1)
{
    res[0] = 1 - 2 * (q1[1] * q1[1] + q1[2] * q1[2]);
    res[1] =     2 * (q1[0] * q1[1] - q1[3] * q1[2]);
    res[2] =     2 * (q1[0] * q1[2] + q1[3] * q1[1]);
    res[3] =     2 * (q1[0] * q1[1] + q1[3] * q1[2]);
    res[4] = 1 - 2 * (q1[0] * q1[0] + q1[2] * q1[2]);
    res[5] =     2 * (q1[1] * q1[2] - q1[3] * q1[0]);
    res[6] =     2 * (q1[0] * q1[2] - q1[3] * q1[1]);
    res[7] =     2 * (q1[1] * q1[2] + q1[3] * q1[0]);
    res[8] = 1 - 2 * (q1[0] * q1[0] + q1[1] * q1[1]);
}

//---------------------------------------------------------------------------
//
// res = v1 * v2^T 
//
inline void cMulM3V3V3t(cDynMatrix3f res, const cDynVector3f v1, const cDynVector3f v2)
{
    res[0] = v1[0] * v2[0];
    res[1] = v1[0] * v2[1];
    res[2] = v1[0] * v2[2];
    res[3] = v1[1] * v2[0];
    res[4] = v1[1] * v2[1];
    res[5] = v1[1] * v2[2];
    res[6] = v1[2] * v2[0];
    res[7] = v1[2] * v2[1];
    res[8] = v1[2] * v2[2];
}

//---------------------------------------------------------------------------
//
// ([x y z] x) = [0 -z y; z 0 -x; -y x 0] 
//
inline void cSetM3V3x(cDynMatrix3f res, const cDynVector3f v1)
{
    res[0]=      0;	res[1]= -v1[2]; res[2]=  v1[1];
    res[3]=  v1[2]; res[4]=      0; res[5]= -v1[0];
    res[6]= -v1[1]; res[7]=  v1[0]; res[8]=      0;
}

//---------------------------------------------------------------------------
//
// res = (v1 x)*m2 
//
inline void cMulM3V3xM3(cDynMatrix3f res, const cDynVector3f v1, const cDynMatrix3f m2)
{
    res[0]= -v1[2] * m2[3] + v1[1] * m2[6];
    res[1]= -v1[2] * m2[4] + v1[1] * m2[7];
    res[2]= -v1[2] * m2[5] + v1[1] * m2[8];
    res[3]=  v1[2] * m2[0] - v1[0] * m2[6];
    res[4]=  v1[2] * m2[1] - v1[0] * m2[7];
    res[5]=  v1[2] * m2[2] - v1[0] * m2[8];
    res[6]= -v1[1] * m2[0] + v1[0] * m2[3];
    res[7]= -v1[1] * m2[1] + v1[0] * m2[4];
    res[8]= -v1[1] * m2[2] + v1[0] * m2[5];
}


//---------------------------------------------------------------------------
//
// setting rotation matrices - axis angle 
//
inline void cSetM3V3S1(cDynMatrix3f res, const cDynVector3f axis, const double angle)
{
    /* also, see Intro. to Robotics by J. Craig: p. 52 */

    double c = cDynCos(angle);
    double v = 1 - c;
    double s = cDynSin(angle);

    res[0] = axis[0] * axis[0] * v + c;
    res[1] = axis[0] * axis[1] * v - axis[2] * s;
    res[2] = axis[0] * axis[2] * v + axis[1] * s;
    res[3] = axis[1] * axis[0] * v + axis[2] * s;
    res[4] = axis[1] * axis[1] * v + c;
    res[5] = axis[1] * axis[2] * v - axis[0] * s;
    res[6] = axis[2] * axis[0] * v - axis[1] * s;
    res[7] = axis[2] * axis[1] * v + axis[0] * s;
    res[8] = axis[2] * axis[2] * v + c;
}

//---------------------------------------------------------------------------
#endif /* CDynMatrix3fH */
//---------------------------------------------------------------------------