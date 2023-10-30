//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynConstraintListH
#define CDynConstraintListH
//---------------------------------------------------------------------------
#include "node/CDynConstraint.h"
//---------------------------------------------------------------------------

class  cDynConstraintList 
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
public:

    //! Constructor of cDynConstraintList.    
    cDynConstraintList();

    //! Destructor of cDynConstraintList.
    ~cDynConstraintList();


    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    void insert(cDynConstraint* c);
    void remove(cDynConstraint* c);
    void check(const cDynTime& lower, const cDynTime& upper);


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
private:

    cDynConstraint* head_;
    cDynConstraint* tail_;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
