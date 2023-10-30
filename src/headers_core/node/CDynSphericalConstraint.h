//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynSphericalConstraintH
#define CDynSphericalConstraintH
//---------------------------------------------------------------------------
#include "matrix/CDynVector3.h"
#include "matrix/CDynFrame.h"
#include "node/CDynConstraint.h"
//---------------------------------------------------------------------------
class cDynObject;
//---------------------------------------------------------------------------

class cDynSphericalConstraintInfo: public cDynConstraintInfo
{
    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //-----------------------------------------------------------------------
public:

    virtual double maxImpulse() { return(maxImpulse_); }
    virtual double maxForce() { return(maxForce_); }
    virtual double velocity() { return(velocity_); }
    void maxImpulse(double max) { maxImpulse_=max; }
    void maxForce(double max) { maxForce_=max; }
    void velocity(double velocity) { velocity_=velocity; }
    
    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
private:
    
    double maxImpulse_;
    double maxForce_;
    double velocity_;
};

//---------------------------------------------------------------------------

class cDynSphericalConstraint: public cDynConstraint
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
public:

    //! Constructor of cDynSphericalConstraint.    
    cDynSphericalConstraint(cDynObject* a, cDynFrame& Fa, cDynObject* b, cDynFrame& Fb, double vmax=1.0, double wmax=1.0);

    //! Destructor of cDynSphericalConstraint.
    virtual ~cDynSphericalConstraint() {};
    

    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //-----------------------------------------------------------------------
public:   
    
    virtual void check();
    virtual void define(const cDynTime& time, cDynObject* a, cDynObject* b);
    cDynObject* nodeA() { return(a_); }
    cDynObject* nodeB() { return(b_); }
    cDynFrame& localA() { return(la_); }
    cDynFrame& localB() { return(lb_); }

    double vmax() { return(vmax_); }
    double wmax() { return(wmax_); }

    void vmax(const double v) { vmax_=v; }
    void wmax(const double w) { wmax_=w; }

    void maxImpulse(double max) { maxImpulse_=max; }
    double maxImpulse() { return(maxImpulse_); }

    void maxForce(double max) { maxForce_=max; }
    double maxForce() { return(maxForce_); }


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
private:

    cDynObject* a_;
    cDynObject* b_;
    cDynFrame   la_;
    cDynFrame   lb_;
    cDynVector3 va_;
    double   vmax_;
    cDynVector3 wa_;
    double   wmax_;

    cDynFrame pos_;
    cDynVector3 difft_;
    cDynVector3 diffr_;

    double maxImpulse_;
    double maxForce_;

    cDynSphericalConstraintInfo info_[6];
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------