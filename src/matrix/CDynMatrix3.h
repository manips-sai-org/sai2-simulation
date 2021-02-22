//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynMatrix3H
#define CDynMatrix3H
//---------------------------------------------------------------------------
#include "matrix/CDynVector3f.h"
#include "matrix/CDynMatrix3f.h"
//---------------------------------------------------------------------------
class cDynMatrix3
{
    //-----------------------------------------------------------------------
    // OPERATORS:
    //-----------------------------------------------------------------------
public:

    //! Operator 'const'.
    operator const cDynMatrix3f&() const { return data_; }

    //! Operator '='.
    void operator=(const cDynMatrix3f &m) { cSetM3M3(data_, m); }

    //! Operator '[]'.
    double *operator[](const int row) { return &data_[row * 3]; }

    //! Operator '[]'.
    const double *operator[](const int row) const { return &data_[row * 3]; }

    //! Operator '+='.
    void operator+=(const cDynMatrix3f &m) { cAddM3M3(data_, m); }

    //! Operator '-='.
    void operator-=(const cDynMatrix3f &m) { cSubM3M3(data_, m); }

    //! Operator '*='.
    void operator*=(const double s) { cMulM3S1(data_, s); }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS: (OPERATIONS)
    //-----------------------------------------------------------------------
public:
    
    //! Get element (i,j).
    double &elementAt(const int i, const int j) { return data_[i * 3 + j]; }

    //! Get element (i,j).
    const double &elementAt(const int i, const int j) const { return data_[i * 3 + j]; }

    //! Initialize matrix to zero.
    void zero() { cZeroM3(data_); }

    // Set identity matrix.
    void identity() { cIdentityM3(data_); }

    //! this = -m
    void negate(const cDynMatrix3f &m) { cNegateM3M3(data_, m); }

    //! res = this + m
#if defined(WIN32) | defined(WIN64)
    void add(const cDynMatrix3f &m1, const cDynMatrix3f &m2) { cAddM3M3M3(data_, m1, m2); }
#else	
    // ALTERNATE VERSION WHICH AVOIDS PROBLEMS WITH GCC OPTIMIZATIONS
    void add(const cDynMatrix3f &m1, const cDynMatrix3f &m2) { for (int i = 0; i < 9; i++) data_[i] = m1[i] + m2[i]; }
#endif


    //! Addition.
    void subtract(const cDynMatrix3f &m1, const cDynMatrix3f &m2) { cSubM3M3M3(data_, m1, m2); }

    //! Multiplication.
    void multiply(const cDynMatrix3f &m1, const cDynMatrix3f &m2) { cMulM3M3M3(data_, m1, m2); }

    // res = this^T * m
    void transposedMultiply(const cDynMatrix3f &m1, const cDynMatrix3f &m2) { cMulM3M3tM3(data_, m1, m2); }

    // res = this * m^T
    void multiplyTransposed(const cDynMatrix3f &m1, const cDynMatrix3f &m2) { cMulM3M3M3t(data_, m1, m2); }

    void multiply(const cDynMatrix3f &m, const double s) { cMulM3M3S1(data_, m, s); }

    //! Set the diagonal terms.
    void diagonal(const double x, const double y, const double z) { cDiagonalM3S3(data_, x, y, z); }

    //! Set the diagonal terms.
    void diagonal(const cDynVector3f &v)  { cDiagonalM3V3(data_, v); }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS: (ROTATION MATRICES)
    //-----------------------------------------------------------------------
public:

    //! Setting rotation matrix.
    void set(const int axis, const double angle) { cSetM3S2(data_, axis, angle); }

    //! Setting rotation matrix.
    void set(const cDynVector3f &axis, const double angle) { cSetM3V3S1(data_, axis, angle); }

    //! Setting rotation matrix.
    void set(const double a0, const double a1, const double a2,
            const double a3, const double a4, const double a5,
            const double a6, const double a7, const double a8)
    { cSetM3S9(data_, a0, a1, a2, a3, a4, a5, a6, a7, a8); }	
    
    //! return determinant of this
    double det() const { return cDetM3(data_); }

    //! this = m^-1 using determinant
    void inverseDet(const cDynMatrix3f &m) { cInvertDetM3M3(data_, m); }

    //! this = m^-1 using determinant where m is SPD
    void inverseDetSPD(const cDynMatrix3f &m) { cInvertDetSPDM3M3(data_, m); }

    //! this = LU decomposition of m
    void luDecomp(const cDynMatrix3f &m) { cLUdecomposeM3M3(data_, m); }

    //! this = m^T
    void transpose(const cDynMatrix3f &m) { cTransposeM3M3(data_, m); }
    //! this = v1 * v2^T
    void multiplyTransposed(const cDynVector3f &v1, const cDynVector3f &v2) { cMulM3V3V3t(data_, v1, v2); }

    //! this = (v x) * m
    void crossMultiply(const cDynVector3f &v, const cDynMatrix3f &m) { cMulM3V3xM3(data_, v, m); }

    //! this = (v x)
    void cross(const cDynVector3f &v) { cSetM3V3x(data_, v); }

    //! this = m * (v x)
    void multiplyCross(const cDynMatrix3f &m, const cDynVector3f &v) { cMulM3M3V3x(data_, m, v); }

    //! this = q
    void set(const cDynQuaternionf &q) { cSetM3Q4(data_, q); }


#ifdef CDYN_DEBUG
    void display(char *str);
#endif // CDYN_DEBUG


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
private:
    cDynMatrix3f data_;
};

extern cDynMatrix3 cMatrix3Zero; // defined in cDynWorld.cpp

//---------------------------------------------------------------------------
#endif // CDynMatrix3H
//---------------------------------------------------------------------------