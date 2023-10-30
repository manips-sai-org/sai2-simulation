//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynVector3H
#define CDynVector3H
//---------------------------------------------------------------------------
#include "matrix/CDynVector3f.h"
#include "matrix/CDynMatrix3f.h"
#include "matrix/CDynQuaternionf.h"
#include "matrix/CDynFramef.h"
#include "matrix/CDynTransformf.h"
//---------------------------------------------------------------------------

class cDynVector3
{
    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
public:

    //! Constructor of cDynVector3.    
    cDynVector3() {}
    cDynVector3(double x, double y, double z) { cSetV3S3(data_, x, y, z); }


    //----------------------------------------------------------------------
    // OPERATORS:
    //----------------------------------------------------------------------
public:
    //! this = v;
    void operator=(const cDynVector3f &v) { cSetV3V3(data_,v); }
    
    //! Return this[i]
    double &operator[](const int i) { return data_[i]; }
    
    //! Return const this[i]
    const double &operator[](const int i) const { return data_[i]; }
    
    //! (const cDynVector3f &)this
    operator const cDynVector3f&() const { return data_; }

    //! Return pointer to vector data.
    operator double*() { return data_; }

    //! Return (this == v)
    int operator==(const cDynVector3f &v) const { return cIsEqualV3V3(data_, v); }

    //! this += v
    void operator+=(const cDynVector3f &v) { cAddV3V3(data_, v); }
    
    //! this -= v
    void operator-=(const cDynVector3f &v) { cSubV3V3(data_, v); }
    
    //! this[i] *= v[i]
    void operator*=(const cDynVector3f &v) { cMulV3V3(data_, v); }
    
    //! this[i] *= s
    void operator*=(const double s) { cMulV3S1(data_, s); }
    
    //! this[i] += s
    void operator+=(const double s) { cAddV3S1(data_, s); }


    //----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //----------------------------------------------------------------------
public:

    //! this = 0
    void zero() { cZeroV3(data_); }

    //! Return this[i]
    double &elementAt(const int i) { return data_[i]; }

    //! Return const this[i]
    const double &elementAt(const int i) const { return data_[i]; }

    //! Return this^T * v
    double dot(const cDynVector3f &v) const { return cDotV3V3(data_, v); }

    //! Return sqrt(this^T * this)
    double magnitude() const { return cMagnitudeV3(data_); }

    //! this[i] *= (1 / magnitude())
    void normalize() { cNormalizeV3(data_); }

    //! this = (x, y, z)
    void set(const double x, const double y, const double z) { cSetV3S3(data_, x, y, z); }

    //! this[i] = min(this[i], v[i])
    void minimum(const cDynVector3f &v) { cMinV3V3(data_, v); }

    //! this[i] = max(this[i], v[i])
    void maximum(const cDynVector3f &v) { cMaxV3V3(data_, v); }

    //! this = -v
    void negate(const cDynVector3f &v) { cNegateV3V3(data_, v); }

    //! this = v1 + v2
    void add(const cDynVector3f &v1, const cDynVector3f &v2) { cAddV3V3V3(data_, v1, v2); }

    //! this = v1 - v2
    void subtract(const cDynVector3f &v1, const cDynVector3f &v2) { cSubV3V3V3(data_, v1, v2); }

    //! this[i] = v1[i] * v2[i]
    void multiply(const cDynVector3f &v1, const cDynVector3f &v2) { cMulV3V3V3(data_, v1, v2); }

    //! this[i] = v[i] * s
    void multiply(const cDynVector3f &v, const double s) { cMulV3V3S1(data_, v, s); }

    //! this[i] = v[i] + s
    void add(const cDynVector3f &v, const double s) { cAddV3V3S1(data_, v, s); }	

    //! this = v1 x v2
    void crossMultiply(const cDynVector3f &v1, const cDynVector3f &v2) { cCrossV3V3V3(data_, v1, v2); }

    //! this = m * v
    void multiply(const cDynMatrix3f &m, const cDynVector3f &v) { cMulV3M3V3(data_, m, v); }

    //! this = m^T * v
    void transposedMultiply(const cDynMatrix3f &m, const cDynVector3f &v) { cMulV3M3tV3(data_, m, v); }

    //! this = [r,p]v = r*v + p
    void multiply(const cDynTransformf &t, const cDynVector3f &v) { cMulV3T9V3(data_, t, v); }

    //! this = ~[r,p]*v = [~r, -(~r*p)]*v = ~r*v -~r*p = ~r*(v-p)
    void inversedMultiply(const cDynTransformf &t, const cDynVector3f &v) { cMulV3T9iV3(data_, t, v); }

    //! this = diag(m)
    void diagonal(const cDynMatrix3f &m) { cDiagonalV3M3(data_, m); }

    //! this = col of m
    void column(const cDynMatrix3f &m, const int col) { cColumnV3M3S1(data_, m, col); }

    //! this = dPhi = R - Rd
    void angularError(const cDynMatrix3f &R, const cDynMatrix3f &Rd) { cAngularErrorV3M3M3(data_, R, Rd); }

    //! this = Euler angles X,Y,Z of m
    void eulerXYZ(const cDynMatrix3f &m) { cSetV3M3xyz(data_, m); }

    //! this = x where y = LU x
    void backSub(const cDynMatrix3f &LU, const cDynVector3f &y) { cBackSubstituteV3M3V3(data_, LU, y); }

    //! this = q * v
    void multiply(const cDynQuaternionf &q, const cDynVector3f &v) { cMulV3Q4V3(data_, q, v); }

    //! this = q^-1 * v
    void inversedMultiply(const cDynQuaternionf &q, const cDynVector3f &v) { cMulV3Q4iV3(data_, q, v); }

    //! this = col of q
    void column(const cDynQuaternionf &q, const int col) { cColumnV3Q4S1(data_, q, col); }

    //! this = [r,p]v = r*v + p
    void multiply(const cDynFramef &f, const cDynVector3f &v) { cMulV3F7V3(data_, f, v); }

    //! this = ~[r,p]*v = [~r, -(~r*p)]*v = ~r*v -~r*p = ~r*(v-p)
    void inversedMultiply(const cDynFramef &f, const cDynVector3f &v) { cMulV3F7iV3(data_, f, v); }

    //-----------------------------------------------------------------------
    /*!
        dPhi = Einv ( q - qd) = -2*q_tilde^T qd <-- since (q_tilde^T q) = 0
        E = 0.5*q_tilde
        Einv = (EtE)inv Et = 4Et = 2*qtilde^T <-- since (q_tilde^T q_tilde) = I
        q_tilde = [ -x -y -z; w -z  y; z  w -x; -y  x  w ] 
        q_tilde^T = [ -x  w  z -y; -y -z  w  x; -z  y -x  w ] 
    */
    //-----------------------------------------------------------------------
    void angularError(const cDynQuaternionf &q, const cDynQuaternionf &qd) { cAngularErrorV3Q4Q4(data_, q, qd); }

#ifdef CDYN_DEBUG
    void display(char *str);
#endif // CDYN_DEBUG


    //----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //----------------------------------------------------------------------
private:
    
    cDynVector3f data_;
};

//---------------------------------------------------------------------------
extern cDynVector3 cVector3Zero; // defined in cDynWorld.cpp
//---------------------------------------------------------------------------
#endif // CDynVector3H
//---------------------------------------------------------------------------