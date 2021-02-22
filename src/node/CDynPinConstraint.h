//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynPinConstraintH
#define CDynPinConstraintH
//---------------------------------------------------------------------------
#include "matrix/CDynVector3.h"
#include "node/CDynConstraint.h"
//---------------------------------------------------------------------------
class cDynObject;
//---------------------------------------------------------------------------

class  cDynPinConstraint: public cDynConstraint
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
public:

    //! Constructor of cDynPinConstraint.  
    cDynPinConstraint(cDynVector3& pos, double maxerr, cDynObject* a, cDynObject* b=NULL);

    //! Destructor of cDynPinConstraint.
    virtual ~cDynPinConstraint() {};


    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:
    
    virtual void check();
    virtual void define(const cDynTime& time, cDynObject* a, cDynObject* b);


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
private:

    cDynObject* a_;
    cDynObject* b_;
    cDynVector3 la_;
    cDynVector3 lb_;

    cDynVector3 ga_;
    cDynVector3 gb_;
    cDynVector3 diff_;

    double maxerr_;
    bool invalid_;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
