//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynQuaternionH
#define CDynQuaternionH
//---------------------------------------------------------------------------
#include "matrix/CDynVector3f.h"
#include "matrix/CDynMatrix3f.h"
#include "matrix/CDynQuaternionf.h"
//---------------------------------------------------------------------------
#define CDYN_QUATERNION_EPSILON	CDYN_QUATERNIONF_EPSILON 
//---------------------------------------------------------------------------
//
// q = [v,w] = [axis*sin(theta/2), cos(theta/2)] <--- axisAngle(axis,theta)
//
//---------------------------------------------------------------------------

class cDynQuaternion
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
public:

    //! Constructor of cDynQuaternion.    
    cDynQuaternion() {}
    cDynQuaternion(double x, double y, double z, double w) { cSetQ4S4(data_, x, y, z, w);  }


    //-----------------------------------------------------------------------
    // OPERATORS:
    //-----------------------------------------------------------------------
public:

    //! (const cDynQuaternionf &)this
    operator const cDynQuaternionf&() const { return data_; }

    //! return q[i]
    double &operator[](const int i) { return data_[i]; }

    //! return const q[i]
    const double &operator[](const int i) const  { return data_[i]; }

    //! this = q
    void operator=(const cDynQuaternionf &q) { cSetQ4Q4(data_, q); }

    //! return (this == q )
    int operator==(const cDynQuaternionf &q) { return cIsEqualQ4Q4(data_, q); }

    //! this[i] += q[i]
    void operator+=(const cDynQuaternionf &q) { cAddQ4Q4(data_, q); }

    //! this[i] -= q[i]
    void operator-=(const cDynQuaternionf &q) { cSubQ4Q4(data_, q); }

    //! this[i] *= s
    void operator*=(const double s) { cMulQ4S1(data_, s); }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    //! this = (0, 0, 0, 1)
    void identity() { cIdentityQ4(data_); }

    //! this = (0, 0, 0, 0)
    void zero() { cZeroQ4(data_); }
    
    //! return this^T * q
    double dot(const cDynQuaternionf &q) { return cDotQ4Q4(data_, q); }

    //! this[i] *= (1 / sqrt(this^T * this))
    void normalize() { cNormalizeQ4(data_); }

    // convert to axis-angle notation
    void get(cDynVector3f &axis, double &angle) const { cAxisAngleV3S1Q4(axis, &angle, data_); }

    //! this = m
    void set(const cDynMatrix3f &m) { cSetQ4M3(data_, m); }

    //! this = (axis, angle)
    void set(const int axis, const double angle) { cSetQ4S2(data_, axis, angle); }

    //! this = (axis, angle)
    void set(const cDynVector3f &axis, const double angle) { cSetQ4V3S1(data_, axis, angle); }

    //! this = (x, y, z, q)
    void set(const double x, const double y, const double z, const double w) { cSetQ4S4(data_, x, y, z, w); }

    //! this = -q
    void negate(const cDynQuaternionf &q) { cNegateQ4Q4(data_, q); }

    //! this = q^-1
    void inverse(const cDynQuaternionf &q) { cInvertQ4Q4(data_, q); }

    //! this = q1 + q2
    void add(const cDynQuaternionf &q1, const cDynQuaternionf &q2) { cAddQ4Q4Q4(data_, q1, q2); }

    //! this = q1 - q2
    void subtract(const cDynQuaternionf &q1, const cDynQuaternionf &q2) { cSubQ4Q4Q4(data_, q1, q2); }

    //! this = q1 * q2
    void multiply(const cDynQuaternionf &q1, const cDynQuaternionf &q2) { cMulQ4Q4Q4(data_, q1, q2); }

    //! this = q1^-1 * q2
    void inversedMultiply(const cDynQuaternionf &q1, const cDynQuaternionf &q2) { cMulQ4Q4iQ4(data_, q1, q2); }


    //-----------------------------------------------------------------------
    /*!
        this = dq
        dq = E omega
        dq = 0.5 q_tilde omega
    */
    //-----------------------------------------------------------------------
    void velocity(const cDynQuaternionf &q, const cDynVector3f &omega) 
    { 
        cVelocityQ4Q4V3(data_, q, omega); 
    }


    //-----------------------------------------------------------------------
    /*!
        this = consistent sign
        this function compares this to qg and put sign consistent qg to res (= qg or -qg).
        (q is equivalent to -q) for all angles (theta) where q = [cos(theta/2), v*sin(theta/2)]
    */
    //-----------------------------------------------------------------------
    void consistentSign(const cDynQuaternionf &q, const cDynQuaternionf &qg) 
    { 
        cConsistentSignQ4Q4Q4(data_, q, qg); 
    }


    //-----------------------------------------------------------------------
    /*!
        this = SLERP
        spherical linear interpolation with extra spins
    */
    //-----------------------------------------------------------------------
    void slerp(const cDynQuaternionf &q, const cDynQuaternionf &qg, const double t, const double addedSpins) 
    { 
        cSlerpQ4Q4Q4S2(data_, q, qg, t, addedSpins); 
    }


#ifdef CDYN_DEBUG
    void display(char *str);
#endif // CDYN_DEBUG


    //-----------------------------------------------------------------------
    // PRIVATE METHODS:
    //-----------------------------------------------------------------------
private:
    cDynQuaternionf data_;
};

//---------------------------------------------------------------------------
#endif // CDynQuaternionH
//---------------------------------------------------------------------------