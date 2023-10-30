//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynTrackConstraintH
#define CDynTrackConstraintH
//---------------------------------------------------------------------------
#include "matrix/CDynVector3.h"
#include "object/CDynObject.h"
#include "node/CDynConstraint.h"
//---------------------------------------------------------------------------

class cDynTrackConstraintInfo: public cDynConstraintInfo
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

class  cDynTrackConstraint: public cDynConstraint
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
public:

    //! Constructor of cDynTrackConstraint.    
    cDynTrackConstraint(cDynVector3& pos, cDynObject* a, cDynObject* b=NULL);

    //! Destructor of cDynTrackConstraint.
    virtual ~cDynTrackConstraint() {};

  
    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    virtual void check();
    virtual void define(const cDynTime& time, cDynObject* a, cDynObject* b);
    void goal(cDynVector3& goal) { goal_ = goal; }
    void goal(double velocity, cDynVector3& goal) { velocity_=velocity, goal_ = goal; }
    cDynVector3 goal() { return(goal_); }

    cDynObject* nodeA() { return(a_); }
    cDynObject* nodeB() { return(b_); }
    cDynVector3 localA() { return(la_); }
    cDynVector3 localB() { return(lb_); }

    double velocity() { return(velocity_); }
    void velocity(double velocity) { if (velocity > 0) velocity_=velocity; }

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
    cDynVector3 la_;
    cDynVector3 lb_;
    cDynVector3 va_;
    cDynVector3 goal_;
    double   velocity_;

    cDynVector3 ga_;
    cDynVector3 gb_;

    double maxImpulse_;
    double maxForce_;

    cDynTrackConstraintInfo info_[3];
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
