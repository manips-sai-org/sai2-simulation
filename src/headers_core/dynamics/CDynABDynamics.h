//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynABDynamicsH
#define CDynABDynamicsH
//---------------------------------------------------------------------------
class cDynABNode;
class cDynVector3;
class cDynVector6;
class cDynFrame;
class cDynABDynamicsData;
class cDynABDynamicsData2;
//---------------------------------------------------------------------------
#include "object/CDynObject.h"
//---------------------------------------------------------------------------
typedef cDynObject cDynDNode;
//---------------------------------------------------------------------------
#define CDYN_ABDYNAMICS_CONTROL_FLOAT 2
//---------------------------------------------------------------------------

class cDynABDynamics
{
    //-----------------------------------------------------------------------
    // PUBLIC STATIC MEMBERS:
    //-----------------------------------------------------------------------
public:

    static void updateLocalXTreeOut(cDynDNode* root);
    static void resetInertiaTreeOut(cDynDNode* root);
    static void resetFlagTreeOut(cDynDNode* root);
    static void forwardDynamics(cDynDNode* root, const cDynVector3* gravity);
    static void inverseDynamics(cDynDNode* root, const cDynVector3* gravity, const int compensateGravity);
    static void forwardDynamicsConfigInit(cDynDNode* root);
    static void forwardDynamicsConfig(cDynDNode* root, cDynDNode* contact);
    static void forwardDynamicsImpulse(cDynDNode* root, cDynDNode* contact, const cDynVector3* point, const cDynVector3* impulse);
    static void globalJacobian(cDynDNode* root);
    static double potentialEnergy(cDynDNode* root, const cDynVector3* gravity);
    static double kineticEnergy(cDynDNode* root, const cDynVector6* Vh = NULL);
    static void resetInertia(cDynDNode* obj);

    //-----------------------------------------------------------------------
    // PRIVATE STATIC MEMBERS:
    //-----------------------------------------------------------------------
private:

    static void _forwardDynamicsOutIn(cDynDNode* root, cDynABDynamicsData* datah, cDynVector6* Pah, const cDynVector6* Vh);
    static void _inverseDynamicsOutIn(cDynDNode* root, cDynABDynamicsData2* datah, cDynVector6* Fh, const cDynVector6* Vh, const cDynVector6* Ah, const int compensateGravity);
    static void _forwardDynamicsConfigInitOutIn(cDynDNode* root, cDynMatrix6* Iah);
    static void _accelerationTreeOut(cDynDNode* root, const cDynVector6* Ah);
    static void _articulatedImpulsePathIn(cDynDNode* contact);
    static void _articulatedBiasForceConfigPathIn(cDynDNode* contact);
    static void _velocityDeltaTreeOut(cDynDNode* root, const cDynVector6* dVh);
};

//---------------------------------------------------------------------------
#endif // CDynABDynamicsH
//---------------------------------------------------------------------------
