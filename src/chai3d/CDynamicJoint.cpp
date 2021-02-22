//===========================================================================
/*
    This file is part of the Newton dynamics library.
    Copyright (C) 2011, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1196 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "chai3d/CDynamicJoint.h"
//---------------------------------------------------------------------------
#include "chai3d/CDynamicLink.h"
#include "chai3d/CDynamicBase.h"
//---------------------------------------------------------------------------
using namespace chai3d; 
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cDynamicJoint.

    \param  a_parentLink  Pointer to associated link.
    \param  a_jointType  Values are: DYN_JOINT_PRISMATIC, DYN_JOINT_REVOLUTE, DYN_JOINT_SPHERICAL.
    \param  a_jointAxis  Values are: DYN_AXIS_X, DYN_AXIS_Y, DYN_AXIS_Z.
*/
//===========================================================================
cDynamicJoint::cDynamicJoint(cDynamicLink* a_parentLink, int a_jointType, int a_jointAxis)
{
    // set joint type and parent link
    m_name = "";
    m_jointType  = a_jointType;
    m_jointAxis  = a_jointAxis;
    m_dynamicParentLink = a_parentLink;
    m_dynamicParentBase = m_dynamicParentLink->m_dynamicParentBase;

    // sanity check
    if ((m_jointType < 0) || (m_jointType > 2)) 
    {
        m_jointType = 0;
    }

    if ((m_jointAxis < 0) || (m_jointAxis > 2)) 
    {
        m_jointAxis = 0;
    }

    cDynObject* dynObject = m_dynamicParentLink->m_dynObject;

    // create joint
    switch (m_jointType)
    {
        case DYN_JOINT_PRISMATIC:
            m_dynJoint = dynObject->joint.prismatic(cDynAxis(m_jointAxis));
        break;

        case DYN_JOINT_REVOLUTE:
            m_dynJoint = dynObject->joint.revolute(cDynAxis(m_jointAxis));
        break;

        case DYN_JOINT_SPHERICAL:
            m_dynJoint = dynObject->joint.spherical();
        break;
    }
}


//===========================================================================
/*!
    Destructor of cDynamicJoint.
*/
//===========================================================================
cDynamicJoint::~cDynamicJoint()
{
    m_dynamicParentLink->m_dynObject->joint.remove(m_dynJoint);
}


//===========================================================================
/*!
    Set joint position.

    \param    a_value  Desired joint position.
*/
//===========================================================================
void cDynamicJoint::setPos(double a_value)
{
    if (m_jointType != DYN_JOINT_SPHERICAL)
    {
        m_dynJoint->position(a_value);
    }
}


//===========================================================================
/*!
    Set joint position for spherical joint.

    \param    a_value  Desired joint position.
*/
//===========================================================================
void cDynamicJoint::setPosSpherical(chai3d::cQuaternion& a_value)
{
    if (m_jointType == DYN_JOINT_SPHERICAL)
    {
        cDynQuaternion quat(a_value.x, a_value.y, a_value.z, a_value.w);
        m_dynJoint->positionSpherical(quat);
    }
}

//===========================================================================
/*!
    Get joint position.
*/
//===========================================================================
double cDynamicJoint::getPos()
{
    if (m_jointType != DYN_JOINT_SPHERICAL)
    {
        return(m_dynJoint->q());
    }
    else
    {
        return (0.0);
    }
}

//===========================================================================
/*!
    Get joint position for spherical joint.
*/
//===========================================================================
chai3d::cQuaternion  cDynamicJoint::getPosSpherical()
{
    if (m_jointType == DYN_JOINT_SPHERICAL)
    {
        cDynQuaternion quat = m_dynJoint->sq();
        chai3d::cQuaternion ret_value = chai3d::cQuaternion(quat[3],quat[0],quat[1],quat[2]);

        return ret_value;
    }
}

//===========================================================================
/*!
    Set joint velocity.

    \param    a_value  Desired joint velocity.
*/
//===========================================================================
void cDynamicJoint::setVel(double a_value)
{
    if (m_jointType != DYN_JOINT_SPHERICAL)
    {
        m_dynJoint->velocity(a_value);
    }
}

//===========================================================================
/*!
    Set joint velocity for spherical joint.

    \param    a_value  Desired joint velocity.
*/
//===========================================================================
void cDynamicJoint::setVelSpherical(chai3d::cVector3d a_value)
{
    if (m_jointType == DYN_JOINT_SPHERICAL)
    {
        cDynVector3 vel(a_value(0), a_value(1), a_value(2));
        m_dynJoint->velocitySpherical(vel);
    }
}

//===========================================================================
/*!
    Get joint velocity.
*/
//===========================================================================
double cDynamicJoint::getVel()
{
    if (m_jointType != DYN_JOINT_SPHERICAL)
    {
        return (m_dynJoint->v());
    }
    else
    {
        return (0.0);
    }
}

//===========================================================================
/*!
    Get joint velocity for Spherical joint.
*/
//===========================================================================
chai3d::cVector3d cDynamicJoint::getVelSpherical()
{
    if (m_jointType == DYN_JOINT_SPHERICAL)
    {
        cDynVector3 vel = m_dynJoint->sv();
        chai3d::cVector3d ret_value = chai3d::cVector3d(vel.elementAt(0),vel.elementAt(1),vel.elementAt(2));
        return ret_value;
    }
    else
    {
        return chai3d::cVector3d(0,0,0);
    }
}

//===========================================================================
/*!
    Get joint acceleration.
*/
//===========================================================================
double cDynamicJoint::getAccel()
{
    if (m_jointType != DYN_JOINT_SPHERICAL)
    {
        return (m_dynJoint->a());
    }
    else
    {
        return (0.0);
    }
}

//===========================================================================
/*!
    Get joint aceleration for Spherical joint.
*/
//===========================================================================
chai3d::cVector3d cDynamicJoint::getAccelSpherical()
{
    if (m_jointType == DYN_JOINT_SPHERICAL)
    {
        cDynVector3 accel = m_dynJoint->sa();
        chai3d::cVector3d ret_value = chai3d::cVector3d(accel.elementAt(0),accel.elementAt(1),accel.elementAt(2));
        return ret_value;
    }
    else
    {
        return chai3d::cVector3d(0,0,0);
    }
}

//===========================================================================
/*!
    Set joint force.

    \param    a_value  Desired joint force/torque.
*/
//===========================================================================
void cDynamicJoint::setForce(double a_value)
{
    if (m_jointType != DYN_JOINT_SPHERICAL)
    {
        m_dynJoint->torque(a_value);
    }
}


//===========================================================================
/*!
    Set spherical joint torque.

    \param  a_value  Desired joint torque.
*/
//===========================================================================
void cDynamicJoint::setTorque(chai3d::cVector3d& a_torque)
{
    if (m_jointType == DYN_JOINT_SPHERICAL)
    {
        cDynVector3 torque(a_torque.x(), a_torque.y(), a_torque.z());
        m_dynJoint->torqueSpherical(torque);
    }
}


//===========================================================================
/*!
    Set minimum joint limit.

    \param  a_jointLimitMin  Minimum joint limit.
    \param  a_jointLimitMax  Maximum joint limit.
    \param  a_error  Tolerance error. 
*/
//===========================================================================
void cDynamicJoint::setJointLimits(double a_jointLimitMin, double a_jointLimitMax, double a_error)
{
    double error = cAbs(a_error);
    if ((a_jointLimitMin + error) < (a_jointLimitMax - error))
    {
        m_dynJoint->bound(a_jointLimitMin, a_jointLimitMax, error);
    }
}


//===========================================================================
/*!
    Remove joint limits.
*/
//===========================================================================
void cDynamicJoint::removeJointLimits()
{
    m_dynJoint->unbound(CDYN_LOWER);
    m_dynJoint->unbound(CDYN_UPPER);
}


//===========================================================================
/*!
    Set damping term.

    \param  a_value  Damping factor.
*/
//===========================================================================
void cDynamicJoint::setDamping(double a_value)
{
    m_dynJoint->damping(a_value);
}


//===========================================================================
/*!
    Get damping term.
*/
//===========================================================================
double cDynamicJoint::getDamping()
{
    return (m_dynJoint->damping());
}