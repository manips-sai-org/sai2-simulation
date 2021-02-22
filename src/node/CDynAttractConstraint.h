//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynAttractConstraintH
#define CDynAttractConstraintH
//---------------------------------------------------------------------------
#include "matrix/CDynVector3.h"
#include "object/CDynObject.h"
#include "node/CDynConstraint.h"
//---------------------------------------------------------------------------

class cDynAttractConstraintInfo: public cDynConstraintInfo
{
    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    virtual double maxImpulse() { return(maxImpulse_); }
    virtual double maxForce() { return(maxForce_); }
    virtual double velocity() { return(velocity_); }
    void maxImpulse(double max) { maxImpulse_=max; }
    void maxForce(double max) { maxForce_=max; }
    void velocity(double velocity) { velocity_=velocity; }


    //-----------------------------------------------------------------------
    // PRIVATE METHODS:
    //-----------------------------------------------------------------------
private:

    double maxImpulse_;
    double maxForce_;
    double velocity_;
};

//---------------------------------------------------------------------------

class  cDynAttractConstraint: public cDynConstraint
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
public:

    //! Constructor of.    
    cDynAttractConstraint(cDynObject* a, cDynVector3& pa, cDynObject* b, cDynVector3& pb, double vmax=1.0);

    //! Destructor of.
    virtual ~cDynAttractConstraint() {};


    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    virtual void check();
    virtual void define(const cDynTime& time, cDynObject* a, cDynObject* b);
    cDynObject* nodeA() { return(a_); }
    cDynObject* nodeB() { return(b_); }
    cDynVector3& localA() { return(la_); }
    cDynVector3& localB() { return(lb_); }

    double velocity() { return(velocity_); }
    void maxImpulse(double max) { maxImpulse_=max; }
    double maxImpulse() { return(maxImpulse_); }
    void maxForce(double max) { maxForce_=max; }
    double maxForce() { return(maxForce_); }


    //-----------------------------------------------------------------------
    // PRIVATE METHODS:
    //-----------------------------------------------------------------------
private:

    cDynObject* a_;
    cDynObject* b_;
    cDynVector3 la_;
    cDynVector3 lb_;
    cDynVector3 va_;

    double   velocity_;

    cDynVector3 pos_;
    cDynVector3 diff_;

    double maxImpulse_;
    double maxForce_;

    cDynAttractConstraintInfo info_[3];
};


//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------

