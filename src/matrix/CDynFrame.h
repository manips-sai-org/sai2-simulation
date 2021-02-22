//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynFrameH
#define CDynFrameH
//---------------------------------------------------------------------------
#include "matrix/CDynFramef.h"
#include "matrix/CDynTransformf.h"
//---------------------------------------------------------------------------
class cDynFrame
{
public:
    //-----------------------------------------------------------------------
    // OPERATORS:
    //-----------------------------------------------------------------------
public:
    
    // operator cDynFramef&() { return data_; }

    //! (const cDynFramef &)this
    operator const cDynFramef&() const { return data_; }

    //! Operator '='
    void operator=(const cDynFramef &t) { cSetF7F7(data_, t); }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    void set(const cDynTransformf &t) { cSetF7T9(data_, t); }
    void set(const cDynMatrix3f &m) { cSetQ4M3(deRotQ4F7(data_), m); }
    void set(const cDynQuaternionf &q) { cSetQ4Q4(deRotQ4F7(data_), q); }
    void set(const cDynVector3f &v) { cSetV3V3(deTransV3F7(data_), v); }
    void set(const double x, const double y, const double z) { cSetV3S3(deTransV3F7(data_), x, y, z); }

    void identity() { cIdentityF7(data_); };

    //! this = f1 * f2 = [r1,p1][r2,p2] = [r1*r2, r1*p2 + p1]
    void multiply(const cDynFramef &f1, const cDynFramef &f2) { cMulF7F7F7(data_, f1, f2); }

    //-----------------------------------------------------------------------
    /*!
        this = f1^-1 * f2 
             = ~[r1,p1][r2,p2] = [~r1, -(~r1*p1)][r2,p2] = [~r1*r2, ~r1*p2 - (~r1*p1)]
             = [~r1*r2, ~r1*(p2-p1)]
    */
    //-----------------------------------------------------------------------
    void inversedMultiply(const cDynFramef &f1, const cDynFramef &f2) { cMulF7F7iF7(data_, f1, f2); }

    //! this = f^-1 =  ~[r,p] = [~r, -(~r*p)]
    void inverse(const cDynFramef &f) { cInvertF7F7(data_, f); }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS: (ROTATION / TRANSLATION)
    //-----------------------------------------------------------------------
public:

    cDynQuaternionf &rotation() { return (cDynQuaternionf &)*deRotQ4F7(data_); }
    const cDynQuaternionf &rotation() const { return (const cDynQuaternionf &)*deRotQ4F7(data_); }

    cDynVector3f &translation() { return (cDynVector3f &)*deTransV3F7(data_); }
    const cDynVector3f &translation() const { return (const cDynVector3f &)*deTransV3F7(data_); }

    double &rotation(const int i) { return deRotQ4F7(data_)[i];; }
    double &translation(const int i) { return deTransV3F7(data_)[i]; }

    

#ifdef CDYN_DEBUG
    void display(char *str);
#endif // CDYN_DEBUG


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
private:

    cDynFramef data_;
};

//---------------------------------------------------------------------------
#endif // CDynFrameH
//---------------------------------------------------------------------------
