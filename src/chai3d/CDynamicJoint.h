//===========================================================================
/*
    This file is part of the Newton dynamics library.
    Copyright (C) 2011, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1196 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynamicJointH
#define CDynamicJointH
//---------------------------------------------------------------------------
#include "chai3d/CDynInternal.h"
#include "chai3d.h"
//---------------------------------------------------------------------------
class cDynamicBase;
class cDynamicLink;
//---------------------------------------------------------------------------
const int DYN_JOINT_PRISMATIC = 0;
const int DYN_JOINT_REVOLUTE  = 1;
const int DYN_JOINT_SPHERICAL = 2;
//---------------------------------------------------------------------------
const int DYN_AXIS_X = 0;
const int DYN_AXIS_Y = 1;
const int DYN_AXIS_Z = 2;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CDynJoint.h

    \brief 
    <b> Dynamics Module </b> \n 
    Dynamic Joint Object.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cDynamicJoint
    \ingroup    Dynamics

    \brief      
    cDynamicJoint models a joint.
*/
//===========================================================================
struct cDynamicJoint
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

public:

    //! Constructor of cDynamicJoint.
    cDynamicJoint(cDynamicLink* a_parentLink, 
        int a_jointType, 
        int a_jointAxis);

    //! Destructor of cDynJoint.
    virtual ~cDynamicJoint();


    //-----------------------------------------------------------------------
    // PUBLIC METHODS: (DYNAMICS)
    //-----------------------------------------------------------------------

public:

    //! Set joint position.
    void setPos(double a_value);

    //! Set joint position for spherical joint.
    void setPosSpherical(chai3d::cQuaternion& a_value);

    //! Get joint position.
    double getPos();

    //! Get joint position for spherical joint.
    chai3d::cQuaternion getPosSpherical();

    //! Set joint velocity.
    void setVel(double a_value);

    //! Set joint velocity for spherical joint.
    void setVelSpherical(chai3d::cVector3d a_value);

    //! Get joint velocity.
    double getVel();

    //! Get joint velocity for spherical joint.
    chai3d::cVector3d getVelSpherical();

    //! Get joint acceleration.
    double getAccel();

    //! Get joint acceleration for spherical joint.
    chai3d::cVector3d getAccelSpherical();

    //! Set joint force or torque (prismatic or revolute joints)
    void setForce(double a_value);

    //! Set torque (spherical joints only)
    void setTorque(chai3d::cVector3d& a_torque);

    //! Get joint type. (Prismatic, Revolute, Spherical).
    int getJointType() { return(m_jointType); }

    //! Get axis type. X, Y or Z.
    int getJointAxis() { return(m_jointAxis); }

    //! Set minimum joint limit.
    void setJointLimits(double a_jointLimitMin, double a_jointLimitMax, double a_error);

    //! Remove joint limits
    void removeJointLimits();

    //! Set damping term.
    void setDamping(double a_value);

    //! Get damping term.
    double getDamping();


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //-----------------------------------------------------------------------

public:

    //! Joint name
    std::string m_name;


    //-----------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //-----------------------------------------------------------------------

protected:

    //! Joint type (DYN_JOINT_PRISMATIC, DYN_JOINT_REVOLUTE, DYN_JOINT_SPHERICAL)
    int m_jointType;

    //! Joint axis (DYN_AXIS_X, DYN_AXIS_Y, DYN_AXIS_Z)
    int m_jointAxis;

    //! Parent link.
    cDynamicLink* m_dynamicParentLink;

    //! Parent base.
    cDynamicBase* m_dynamicParentBase;

    //! Joint.
    cDynJoint* m_dynJoint;
};


//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
