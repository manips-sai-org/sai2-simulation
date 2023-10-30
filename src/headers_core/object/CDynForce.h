//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynForceH
#define CDynForceH
//---------------------------------------------------------------------------
#include <assert.h>
#include "matrix/CDynMathDefn.h"
#include "matrix/CDynVector6.h"
//---------------------------------------------------------------------------
class cDynForceProperty;
class cDynObject;
//---------------------------------------------------------------------------
class cDynForce
{
    friend class cDynObject;
    friend class cDynForceProperty;

    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
public:

    //! Constructor of cDynForce.
    cDynForce(cDynObject* obj=NULL);

    //! Destructor of cDynForce.
    ~cDynForce();


    //----------------------------------------------------------------------
    // PUBLIC METHODS:
    //----------------------------------------------------------------------
public:

    //! Add new force property for object.
    void add(cDynForceProperty* property);
    
    //! Remove force property form object.
    void remove(cDynForceProperty* property);
        
    //! Get 6x1 spacial vector that is the accumulation of the force properties for an object.
    void get(cDynVector6& force);


    //----------------------------------------------------------------------
    // PRIVATE METHODS:
    //----------------------------------------------------------------------
private:

    void object(cDynObject* obj) { obj_=obj; }
    cDynForceProperty* properties_;
    cDynObject* obj_;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------