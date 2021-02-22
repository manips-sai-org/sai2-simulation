//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynConstraintH
#define CDynConstraintH
//---------------------------------------------------------------------------
#include "object/CDynObject.h"
#include "node/CDynBaseNode.h"
#include "node/CDynContact.h"
//---------------------------------------------------------------------------
class cDynConstraintInfo;
class cDynConstraintList;
//---------------------------------------------------------------------------

class  cDynConstraint 
{
    friend class cDynContact;
    friend class cDynConstraintList;

    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    virtual void check();
    inline void node(cDynObject* node);
    cDynTime time(const int type) const { return(time_[type]); } // 0=lower, 1=upper
    //void constraint(cDynTime& time, cDynObject* a, cDynObject* b=NULL, cDynPrim* pa=NULL, cDynPrim* pb=NULL);
    void constraint(cDynTime& time, cDynObject* a, cDynObject* b=NULL);

    virtual void define(const cDynTime& time, cDynObject* a, cDynObject* b);
    void add(cDynVector3& n, cDynVector3& pos, double err, double maxerr, int type, cDynConstraintInfo* c=NULL);


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
private:

    void contact(cDynContact* contact) {contact_=contact;}
    cDynContact* contact() {return(contact_);}
    cDynTime time_[2];
    cDynContact* contact_;

    cDynConstraint* prev_;
    cDynConstraint* next_;
};

//---------------------------------------------------------------------------

class cDynConstraintInfo 
{
    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    virtual double maxImpulse() { return(0.0); }
    virtual double maxForce() { return(0.0); }
    virtual double velocity() { return(0.0); }
    virtual double acceleration() { return(0.0); }
};

//---------------------------------------------------------------------------

inline void cDynConstraint::node(cDynObject* node)
{
    if (node == NULL) return;
    if (time_[1] > node->baseNode()->contact()->time())
        time_[1]= node->baseNode()->contact()->time();
}

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
