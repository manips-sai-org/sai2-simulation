//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynQuaternionfH
#define CDynQuaternionfH
//---------------------------------------------------------------------------
#include "matrix/CDynMathDefn.h"
#include "matrix/CDynVector3f.h"
//---------------------------------------------------------------------------
#define CDYN_QUATERNIONF_EPSILON			((double)0.001)
#define CDYN_QUATERNIONF_COS_THRESHHOLD	(1 - CDYN_QUATERNIONF_EPSILON)
//---------------------------------------------------------------------------
/* 
 * q = [w,v] = [cos(theta/2);, axis*sin(theta/2);] <--- axisAngle(axis,theta);
 * q = [vx,vy,vz,w] --> double[4];
 */
typedef double cDynQuaternionf[4];

//---------------------------------------------------------------------------
void cColumnV3Q4S1(cDynVector3f res, const cDynQuaternionf q1, const int col);
void cConsistentSignQ4Q4Q4(cDynQuaternionf res, const cDynQuaternionf q1, const cDynQuaternionf q2);
void cSlerpQ4Q4Q4S2(cDynQuaternionf res, const cDynQuaternionf q1, const cDynQuaternionf q2, const double t, const double addedSpins);
void cAxisAngleV3S1Q4(cDynVector3f axis, double *angle, const cDynQuaternionf q1);
//---------------------------------------------------------------------------

inline void cSetQ4S4(cDynQuaternionf res, const double x, const double y, const double z, const double w)
{
    res[0] = x;	res[1] = y;	res[2] = z;	res[3] = w;
}

//---------------------------------------------------------------------------

inline void cIdentityQ4(cDynQuaternionf res)
{
    res[0] = res[1] = res[2] = 0;	res[3] = 1;
}

//---------------------------------------------------------------------------

inline void cZeroQ4(cDynQuaternionf res)
{
    res[0] = res[1] = res[2] = res[3] = 0;
}

//---------------------------------------------------------------------------

inline void cNegateQ4Q4(cDynQuaternionf res, const cDynQuaternionf q1)
{
    res[0] = -q1[0];
    res[1] = -q1[1];
    res[2] = -q1[2];
    res[3] = -q1[3];
}

//---------------------------------------------------------------------------

inline void cInvertQ4Q4(cDynQuaternionf res, const cDynQuaternionf q1)
{
    res[0] = -q1[0];
    res[1] = -q1[1];
    res[2] = -q1[2];
    res[3] =  q1[3];
}

//---------------------------------------------------------------------------

inline void cSetQ4Q4(cDynQuaternionf res, const cDynQuaternionf q1)
{
    res[0] = q1[0];
    res[1] = q1[1];
    res[2] = q1[2];
    res[3] = q1[3];
}

//---------------------------------------------------------------------------

inline void cAddQ4Q4Q4(cDynQuaternionf res, const cDynQuaternionf q1, const cDynQuaternionf q2)
{
    res[0] = q1[0] + q2[0];
    res[1] = q1[1] + q2[1];
    res[2] = q1[2] + q2[2];
    res[3] = q1[3] + q2[3];
}

//---------------------------------------------------------------------------

inline void cSubQ4Q4Q4(cDynQuaternionf res, const cDynQuaternionf q1, const cDynQuaternionf q2)
{
    res[0] = q1[0] - q2[0];
    res[1] = q1[1] - q2[1];
    res[2] = q1[2] - q2[2];
    res[3] = q1[3] - q2[3];
}

//---------------------------------------------------------------------------

inline double cDotQ4Q4(const cDynQuaternionf q1, const cDynQuaternionf q2)
{
    return (q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3]);
}

//---------------------------------------------------------------------------
//
// res = (w1*w2 - v1.v2, v1*w2 + v2*w1 + v1xv2)
//
inline void cMulQ4Q4Q4(cDynQuaternionf res, const cDynQuaternionf q1, const cDynQuaternionf q2)
{
    res[0] = q1[0] * q2[3] + q2[0] * q1[3] + (q1[1] * q2[2] - q1[2] * q2[1]);
    res[1] = q1[1] * q2[3] + q2[1] * q1[3] + (q1[2] * q2[0] - q1[0] * q2[2]);
    res[2] = q1[2] * q2[3] + q2[2] * q1[3] + (q1[0] * q2[1] - q1[1] * q2[0]);
    res[3] = q1[3] * q2[3] - (q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2]);
}

//---------------------------------------------------------------------------

inline void cMulQ4Q4iQ4(cDynQuaternionf res, const cDynQuaternionf q1, const cDynQuaternionf q2)
{
    /* w_ = w_, v_ = - v_ */
    res[0] = -q1[0] * q2[3] + q2[0] * q1[3] - (q1[1] * q2[2] - q1[2] * q2[1]);
    res[1] = -q1[1] * q2[3] + q2[1] * q1[3] - (q1[2] * q2[0] - q1[0] * q2[2]);
    res[2] = -q1[2] * q2[3] + q2[2] * q1[3] - (q1[0] * q2[1] - q1[1] * q2[0]);
    res[3] =  q1[3] * q2[3] + (q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2]);
}

//---------------------------------------------------------------------------

inline void cMulQ4S1(cDynQuaternionf res, const double s)
{
    res[0] *= s;
    res[1] *= s;
    res[2] *= s;
    res[3] *= s;
}

//---------------------------------------------------------------------------

inline void cAddQ4Q4(cDynQuaternionf res, const cDynQuaternionf q1)
{
    res[0] += q1[0];
    res[1] += q1[1];
    res[2] += q1[2];
    res[3] += q1[3];
}

//---------------------------------------------------------------------------

inline void cSubQ4Q4(cDynQuaternionf res, const cDynQuaternionf q1)
{
    res[0] -= q1[0];
    res[1] -= q1[1];
    res[2] -= q1[2];
    res[3] -= q1[3];
}

//---------------------------------------------------------------------------
/*
    [w,v'] = q * [0,v] * ~q  <==> v' = R v
    = [w1,v1]*[0,v2]*[w1,-v1] = [ -v1.v2, w1v2+ v1 x v2]*[w1,-v1]
    = [(-v1.v2)(w1)+(w1v2+ v1 x v2).v1,
    (v1.v2)v1 + w1(w1v2+ v1 x v2) + v1 x (w1v2+ v1 x v2)]
    
    since A x (B + C) = A x B + A x C
    = [w, (w1*w1)v2 + 2*w1(v1 x v2) + (v1.v2)v1 + v1 x (v1 x v2)]
    
    since A x (B x C) = (A dot C)B - (A dot B)C
    = [w, (w1*w1)v2 + 2*w1(v1 x v2) + (v1.v2)v1 + (v1.v2)v1 - (v1.v1)v2]
    = [w, (w1*w1 - v1.v1)v2 + 2*w1(v1 x v2) + 2*(v1.v2)v1]
    v' = [(w1*w1 - v1.v1)v2 + 2*w1(v1 x v2) + 2*(v1.v2)v1]
*/
//---------------------------------------------------------------------------
inline void cMulV3Q4V3(cDynVector3f res, const cDynQuaternionf q1, const cDynVector3f v2)
{
    double m11 = q1[3] * q1[3] - (q1[0] * q1[0] + q1[1] * q1[1] + q1[2] * q1[2]);
    double m12 = 2 * (q1[0] * v2[0] + q1[1] * v2[1] + q1[2] * v2[2]);
    double w2 = q1[3] + q1[3];

    res[0] = q1[0] * m12 + v2[0] * m11 + (q1[1] * v2[2] - q1[2] * v2[1]) * w2;
    res[1] = q1[1] * m12 + v2[1] * m11 + (q1[2] * v2[0] - q1[0] * v2[2]) * w2;
    res[2] = q1[2] * m12 + v2[2] * m11 + (q1[0] * v2[1] - q1[1] * v2[0]) * w2;
}

//---------------------------------------------------------------------------
/*
    [w,v'] = ~q * [0,v] * q  <==> v' = R v
    = [w1,-v1]*[0,v2]*[w1,v1] = [ v1.v2, w1v2 - v1 x v2]*[w1,v1]
    v' = [(w1*w1 - v1.v1)v2 - 2*w1(v1 x v2) + 2*(v1.v2)v1]
*/
//---------------------------------------------------------------------------
inline void cMulV3Q4iV3(cDynVector3f res, const cDynQuaternionf q1, const cDynVector3f v2)
{
    // w_ = w_, v_ = - v_
    double m11 = q1[3] * q1[3] - (q1[0] * q1[0] + q1[1] * q1[1] + q1[2] * q1[2]);
    double m12 = 2 * (q1[0] * v2[0] + q1[1] * v2[1] + q1[2] * v2[2]);
    double w2 = -(q1[3] + q1[3]);

    res[0] = q1[0] * m12 + v2[0] * m11 + (q1[1] * v2[2] - q1[2] * v2[1]) * w2;
    res[1] = q1[1] * m12 + v2[1] * m11 + (q1[2] * v2[0] - q1[0] * v2[2]) * w2;
    res[2] = q1[2] * m12 + v2[2] * m11 + (q1[0] * v2[1] - q1[1] * v2[0]) * w2;
}

//---------------------------------------------------------------------------

inline int cIsEqualQ4Q4(const cDynQuaternionf q1, const cDynQuaternionf q2)
{
    double mag2 = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];
    return (mag2 > CDYN_QUATERNIONF_COS_THRESHHOLD || mag2 < -CDYN_QUATERNIONF_COS_THRESHHOLD);
}

//---------------------------------------------------------------------------
//
// [w1,v1]*[w2,v2] = [(w1*w2 - v1 dot v2), (w1*v2+w2*v1+ v1 cross v2)]
//
inline void cNormalizeQ4(cDynQuaternionf res)
{
    double mag = 1 / cDynSqrt(res[0] * res[0] + res[1] * res[1] + res[2] * res[2] + res[3] * res[3]);
    res[0] *= mag;
    res[1] *= mag;
    res[2] *= mag;
    res[3] *= mag;
}

//---------------------------------------------------------------------------
//
// axis angle
//
inline void cSetQ4V3S1(cDynQuaternionf res, const cDynVector3f axis, const double angle)
{
    double s = cDynSin(0.5 * angle);
    res[0] = axis[0] * s;
    res[1] = axis[1] * s;
    res[2] = axis[2] * s;
    res[3] = cDynCos(0.5 * angle);
}

//---------------------------------------------------------------------------
//
// axis angle
//
inline void cSetQ4S2(cDynQuaternionf res, const int axis, const double angle)
{
    // also, see Intro. to Robotics by J. Craig: p. 51.
    res[0] = res[1] = res[2] = 0;
    res[axis] = cDynSin(0.5 * angle);
    res[3] = cDynCos(0.5 * angle);
}	

//---------------------------------------------------------------------------
/*
    dPhi = Einv ( q - qd) = -2*q_tilde^T qd <-- since (q_tilde^T q) = 0
    E = 0.5*q_tilde
    Einv = (EtE)inv Et = 4Et = 2*qtilde^T <-- since (q_tilde^T q_tilde) = I
 
    q = [w x y z]
    Khatib
    q_tilde = [ -x -y -z
                 w  z -y
                -z  w  x
                 y -x  w ] 
 q_tilde^T = [  -x  w -z  y
                -y  z  w -x
                -z -y  x  w ] 	       
*/
//---------------------------------------------------------------------------
inline void cAngularErrorV3Q4Q4(cDynVector3f res, const cDynQuaternionf q1, const cDynQuaternionf q2)
{
  res[0] = -2 * ( q1[3] * q2[0] - q1[2] * q2[1] + q1[1] * q2[2] - q1[0] * q2[3]);
  res[1] = -2 * ( q1[2] * q2[0] + q1[3] * q2[1] - q1[0] * q2[2] - q1[1] * q2[3]);
  res[2] = -2 * (-q1[1] * q2[0] + q1[0] * q2[1] + q1[3] * q2[2] - q1[2] * q2[3]);
}

//---------------------------------------------------------------------------
/*
    dq = E v2
    dq = 0.5 q_tilde v2
*/
//---------------------------------------------------------------------------
inline void cVelocityQ4Q4V3(cDynQuaternionf res, const cDynQuaternionf q1, const cDynVector3f v2)
{
  res[0] = 0.5 * ( q1[3] * v2[0] + q1[2] * v2[1] - q1[1] * v2[2]);
  res[1] = 0.5 * (-q1[2] * v2[0] + q1[3] * v2[1] + q1[0] * v2[2]);
  res[2] = 0.5 * ( q1[1] * v2[0] - q1[0] * v2[1] + q1[3] * v2[2]);
  res[3] = 0.5 * (-q1[0] * v2[0] - q1[1] * v2[1] - q1[2] * v2[2]);
}

//---------------------------------------------------------------------------
#endif /* CDynQuaternionH */
//---------------------------------------------------------------------------