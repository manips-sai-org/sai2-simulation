//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynInterpolateH
#define CDynInterpolateH
//---------------------------------------------------------------------------
#ifndef CDYN_DONT_DOCUMENT
//---------------------------------------------------------------------------
#include "matrix/CDynMathDefn.h"
#include "matrix/CDynVector3.h"
#include "matrix/CDynMatrix3.h"
#include "matrix/CDynVector6.h"
#include "matrix/CDynMatrix6.h"
//---------------------------------------------------------------------------

class cDynInterpolate
{
    //----------------------------------------------------------------------
    // PUBLIC METHODS:
    //----------------------------------------------------------------------
public:

    void generate(const double dt,
        cDynVector6& a, 
        const cDynVector3& ql, 
        const cDynVector3& qu);
        
    void solve(const double pt,
        const cDynVector6& a,
        cDynVector3& r);


    //----------------------------------------------------------------------
    // PRIVATE METHODS:
    //----------------------------------------------------------------------
private:

    void update(const double dt);
    double dt_;


    //----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //----------------------------------------------------------------------
private:

    int valid_;
    cDynMatrix3 LU_;
    cDynVector6 t_;
    cDynVector6 ti_;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------