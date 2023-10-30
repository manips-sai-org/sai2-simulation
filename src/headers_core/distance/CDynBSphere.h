//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynBSphereH
#define CDynBSphereH
//---------------------------------------------------------------------------
#include "matrix/CDynVector3.h"
//---------------------------------------------------------------------------
class cDynPrim;
//---------------------------------------------------------------------------
#define CDYN_BSPHERE_P(bs)			((bs)->_p)
#define CDYN_BSPHERE_R(bs)			((bs)->_r)
#define CDYN_BSPHERE_STATE(bs)		((bs)->_state)
#define CDYN_BSPHERE_PRIM(bs)		((bs)->_prim)
#define CDYN_BSPHERE_LEFT(bs)		((bs)->_left)
#define CDYN_BSPHERE_RIGHT(bs)		((bs)->_right)
//---------------------------------------------------------------------------

class cDynBSphere
{
    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
public:

    //! Constructor of cDynBSphere.    
    cDynBSphere(cDynVector3* v, double radius, cDynPrim *p);

    //! Destructor of cDynBSphere.
    ~cDynBSphere() {}

    //----------------------------------------------------------------------
    // PUBLIC METHODS:
    //----------------------------------------------------------------------
public:

    void remove();
    double max(const cDynVector3 *dir, cDynPrim *wprim, int *witness, const double d, cDynPrim* cache[]) const;


    //----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //----------------------------------------------------------------------
public:
    int _state;
    double _r;
    cDynVector3 _p;

    cDynPrim *_prim;

    cDynBSphere *_left;
    cDynBSphere *_right;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
