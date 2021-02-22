//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynInternalH
#define CDynInternalH
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CDynInternal.h

    \brief    
    <b> Dynamics Module </b> \n 
    Internal Header File.
*/
//===========================================================================

//---------------------------------------------------------------------------
//!     \defgroup   Dynamics Module 
//---------------------------------------------------------------------------

// distance
#include "distance/CDynCover.h"
#include "distance/CDynDist.h"
#include "distance/CDynHierarchy.h"
#include "distance/CDynPrim.h"

// dynamics
#include "dynamics/CDynABDynamics.h"
#include "dynamics/CDynABJoint.h"
#include "dynamics/CDynABNode.h"
#include "dynamics/CDynDynamics.h"

// extras
#include "extras/CDynHull.h"

// global
#include "global/CDynGlobalDefn.h"

// matrix
#include "matrix/CDynMathDefn.h"
#include "matrix/CDynVector3.h"
#include "matrix/CDynVector3f.h"
#include "matrix/CDynVector6.h"
#include "matrix/CDynVector6f.h"
#include "matrix/CDynMatrix3.h"
#include "matrix/CDynMatrix3f.h"
#include "matrix/CDynMatrix6.h"
#include "matrix/CDynMatrix6f.h"
#include "matrix/CDynQuaternion.h"
#include "matrix/CDynQuaternionf.h"
#include "matrix/CDynTransform.h"
#include "matrix/CDynTransformf.h"
#include "matrix/CDynFrame.h"
#include "matrix/CDynFramef.h"
#include "matrix/CDynFrameStack.h"

// node
#include "node/CDynAttractConstraint.h"
#include "node/CDynBaseNode.h"
#include "node/CDynCollision.h"
#include "node/CDynCollisionCheckRecord.h"
#include "node/CDynConstraint.h"
#include "node/CDynConstraintList.h"
#include "node/CDynContact.h"
#include "node/CDynContactEvent.h"
#include "node/CDynContactPointCache.h"
#include "node/CDynFrictionRecord.h"
#include "node/CDynJointLimit.h"
#include "node/CDynPinConstraint.h"
#include "node/CDynPrimPair.h"
#include "node/CDynRayCast.h"
#include "node/CDynSphericalConstraint.h"
#include "node/CDynTrackConstraint.h"
#include "node/CDynWorld.h"

// object
#include "object/CDynForce.h"
#include "object/CDynForceProperty.h"
#include "object/CDynProperty.h"
#include "object/CDynGeometry.h"
#include "object/CDynJoint.h"
#include "object/CDynMass.h"
#include "object/CDynMaterial.h"
#include "object/CDynObject.h"
#include "object/CDynPrimitive.h"

// utility
#include "utility/CDynError.h"
#include "utility/CDynLogger.h"
#include "utility/CDynQHeader.h"
#include "utility/CDynTimeEvent.h"
#include "utility/CDynTimeInterval.h"

// var
#include "var/CDynInterpolate.h"
#include "var/CDynState.h"
#include "var/CDynVar.h"

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
