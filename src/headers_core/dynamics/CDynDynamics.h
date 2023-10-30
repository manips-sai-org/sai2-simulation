//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynDynamicsH
#define CDynDynamicsH
//---------------------------------------------------------------------------
#include "matrix/CDynVector3.h"
#include "matrix/CDynVector6.h"
#include "matrix/CDynMatrix3.h"
#include "matrix/CDynMatrix6.h"
#include "matrix/CDynTransform.h"
//---------------------------------------------------------------------------
class cDynObject;
class cDynJoint;
//---------------------------------------------------------------------------

void cDynamicsInitialize(cDynObject *rootNode, long callnum);

void cDynamicsInvDynamics(cDynObject *rootNode, cDynVector3 *gravity, const int compensateGravity);
void cDynamicsFwdDynamics(cDynObject *rootNode, cDynVector3 *gravity, long callnum);

//---------------------------------------------------------------------------
/*!
    Use initFwdDynamicsConfig(flag) to set configuration only depend terms
    then use fwdDynamicsConfig() to solve : ddq = Ainv tau
*/
//---------------------------------------------------------------------------
void cDynamicsInitFwdDynamicsConfig(cDynObject *rootNode, long callnum);


//---------------------------------------------------------------------------
/*!
    Use initFwdDynamicsConfig(flag) to set configuration only depend terms
    then use fwdDynamicsConfig() to solve : ddq = Ainv tau
*/
//---------------------------------------------------------------------------
void cDynamicsFwdDynamicsConfig(cDynObject *rootNode,
    cDynObject *contactNode,
    double (*fwdIn)(cDynJoint *j), 
    const cDynVector3& (*fwdInSphere)(cDynJoint *j),
    void (*fwdOut)(cDynJoint* j, const double x), 
    void (*fwdOutSphere)(cDynJoint* j, const cDynVector3& x), 
    int rotate);


//---------------------------------------------------------------------------
/*!
    Use initFwdDynamicsConfig() to set configuration only depend terms
    then use fwdDynamicsImpulse() or fwdDynamicsVel() to solve the forward dynamics
*/
//---------------------------------------------------------------------------
void cDynamicsFwdDynamicsVel(cDynObject *rootNode);


//---------------------------------------------------------------------------
/*!
    Assuming contactPoint,impulseVector are in local frame of contactNode
*/
//---------------------------------------------------------------------------
void cDynamicsImpulse(cDynObject *rootNode,
    cDynObject *contactNode,
    const cDynVector3 *contactPoint,
    const cDynVector3 *impulseVector);

void cDynamicsFwdDynamicsIn(cDynObject *contactNode, 
    double (*fwdIn)(cDynJoint *j), 
    const cDynVector3& (*fwdInSphere)(cDynJoint *j));

// void cDynamicsFwdDynamicsImpulse(cDynObject *rootNode,cDynObject *impulseNode);


//---------------------------------------------------------------------------
/*!
    Use after call to fwdDynamicsVel()
*/
//---------------------------------------------------------------------------
// void cDynamicsGlobalBiasAcceleration(cDynObject *rootNode,cDynObject *contactNode,cDynVector6 *H);

double cDynamicsPotentialEnergy(cDynObject *rootNode,const cDynVector3 *gravity);
double cDynamicsKineticEnergy(cDynObject *rootNode);

void cDynamicsGlobalJacobian(cDynObject *rootNode);
void cDynamicsTorqueExternalZeroInwardPath(cDynObject *node);


//---------------------------------------------------------------------------
/*!
    Reset mass and inertia for one node.
    Assuming all dimensions stay the same and the body consists of only one.
*/
//---------------------------------------------------------------------------
void cDynamicsResetMass(cDynObject *node,const double mass);
void cDynamicsInvOSIM(cDynObject *rootNode);

#ifdef CDYN_DEBUG
void cDynamicsDisplay(cDynObject *rootNode);
#endif // CDYN_DEBUG

//---------------------------------------------------------------------------
#endif // CDynDynamicsH
//---------------------------------------------------------------------------