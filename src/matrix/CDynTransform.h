//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynTransformH
#define CDynTransformH
//---------------------------------------------------------------------------
#include "matrix/CDynTransformf.h"
#include "matrix/CDynFramef.h"
//---------------------------------------------------------------------------
class cDynTransform
{
    //----------------------------------------------------------------------
    // OPERATORS:
    //----------------------------------------------------------------------
public:

    //! (const cDynTransformf &)this
    operator const cDynTransformf&() const { return data_; }
    void operator=(const cDynTransformf &t) { cSetT9T9(data_, t); }


    //----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //----------------------------------------------------------------------
public:

    cDynMatrix3f &rotation() { return (cDynMatrix3f &)*cRotM3T9(data_); }
    const cDynMatrix3f &rotation() const  { return (const cDynMatrix3f &)*cRotM3T9(data_); }
    
    cDynVector3f &translation()  { return (cDynVector3f &)*cTransV3T9(data_); }
    const cDynVector3f &translation() const { return (const cDynVector3f &)*cTransV3T9(data_); }

    double &rotation(const int i, const int j) { return cRotM3T9(data_)[i * 3 + j]; }
    double &translation(const int i) { return cTransV3T9(data_)[i]; }

    void identity() { cIdentityT9(data_); }

    //! this = [r1,p1][r2,p2] = [r1*r2, r1*p2 + p1]
    void multiply(const cDynTransformf &t1, const cDynTransformf &t2) { cMulT9T9T9(data_, t1, t2); }
    
    //! this =  ~[r,p] = [~r, -(~r*p)]
    void inverse(const cDynTransformf &t) { cInvertT9T9(data_, t); }
    
    //! this = ~[r1,p1][r2,p2] = [~r1, -(~r1*p1)][r2,p2] = [~r1*r2, ~r1*(p2-p1)]
    void inversedMultiply(const cDynTransformf &t1, const cDynTransformf &t2) { cMulT9T9iT9(data_, t1, t2); }
    
    //! this = [r1,p1]~[r2,p2] = [r1,p1][~r2, -(~r2*p2)] = [(r1*~r2), p1-(r1*~r2)*p2]
    void multiplyInversed(const cDynTransformf &t1, const cDynTransformf &t2) { cMulT9T9T9i(data_, t1, t2); }
    
    //! this = f
    void set(const cDynFramef &f) { cSetT9F7(data_, f); }
    
    //! this = Denavit-Hartenberg parameter
    void set(const double alpha,const double a,const double d,const double theta) { cSetT9S4(data_, alpha, a, d, theta); }
    
    //! this = Screw coordinates
    void set(const cDynVector3f &axis,const double pitch,const double angle) { cSetT9V3S2(data_, axis, pitch, angle); }
    void set(const cDynMatrix3f &m) { cSetM3M3(cRotM3T9(data_), m); }
    void set(const cDynQuaternionf &q) { cSetM3Q4(cRotM3T9(data_), q); }
    void set(const cDynVector3f &v) { cSetV3V3(cTransV3T9(data_), v); }
    void set(const double x, const double y, const double z) { cSetV3S3(cTransV3T9(data_), x, y, z); }

#ifdef CDYN_DEBUG
    void display(char *str);
#endif // CDYN_DEBUG


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
private:
    cDynTransformf data_;
};

//---------------------------------------------------------------------------
#endif // CDynTransformH
//---------------------------------------------------------------------------
