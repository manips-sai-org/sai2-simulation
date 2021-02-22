//===========================================================================
/*
    This file is part of the Newton dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1289 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef PrimitivesH
#define PrimitivesH
//---------------------------------------------------------------------------
#include "dynamics3d.h"
#include "chai3d.h"
//---------------------------------------------------------------------------
const double ROVER_LENGTH   = 0.98;
const double ROVER_WIDTH    = 0.70;
const double ROVER_HEIGHT   = 0.15;
const double ROVER_WHEEL_RADIUS = 0.12;
const double ROVER_SUSPENSION_KS = 3.5;   // stiffness
const double ROVER_SUSPENSION_KD = 0.2;   // damping
const int    ROVER_NUM_SENSORS = 12;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \class      cRover

    \brief      
    cRover defines models a rover.
*/
//===========================================================================
class cRover
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
public:

    //! Constructor of cRover.
    cRover(cDynamicWorld* a_world, 
           double a_size,
           chai3d::cVector3d a_pos = chai3d::cVector3d(0,0,0), 
           chai3d::cMatrix3d a_rot = chai3d::cIdentity3d());

    //! Destructor of cRover.
    virtual ~cRover() {};

    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
    
    //! Update controller commands
    void updateController();

    //! Set desired rover velocity
    void setVelocity(double a_velocity) { m_desiredVelocity = chai3d::cClamp(a_velocity, -1.0, 1.0); }

    //! Get desired rover velocity
    double getVelocity() { return (m_desiredVelocity); }

    //! Set desired Direction
    void setDirectionDeg(double a_directionDeg) { m_desiredDirectionDeg = chai3d::cClamp(a_directionDeg, -45.0, 45.0); }

    //! Get desired Direction
    double getDirectionDeg() { return (m_desiredDirectionDeg); }

    //! Set braking level
    void setBrakingLevel(double a_braking) { m_desiredBrake = chai3d::cClamp01(a_braking); }

    //! Get braking level
    double getBrakingLevel() { return (m_desiredBrake); }

    //! Update sensor data
    void updateSensorData(chai3d::cWorld* a_world);


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
public:

    //! desired velocity (-1.0 - +1.0)
    double m_desiredVelocity;

    //! desired direction (-45 - +45)
    double m_desiredDirectionDeg;

    //! braking level (0.0 - 1.0)
    double m_desiredBrake;

    //! basenode
    cDynamicBase* m_dynamicBase;

    //! rover body
    cDynamicLink* m_dynamicBody;
    cDynamicJoint* m_jointBodyX;
    cDynamicJoint* m_jointBodyY;
    cDynamicJoint* m_jointBodyZ;
    cDynamicJoint* m_jointBodyS;

    //! rover wheel (front-left)
    cDynamicLink* m_dynamicFL;
    cDynamicJoint* m_jointSuspensionFL;
    cDynamicJoint* m_jointDirectionFL;
    cDynamicJoint* m_jointFL;

    //! rover wheel (front-right)
    cDynamicLink* m_dynamicFR;
    cDynamicJoint* m_jointSuspensionFR;
    cDynamicJoint* m_jointDirectionFR;
    cDynamicJoint* m_jointFR;

    //! rover wheel (back-left)
    cDynamicLink* m_dynamicBL;
    cDynamicJoint* m_jointSuspensionBL;
    cDynamicJoint* m_jointBL;

    //! rover wheel (back-front)
    cDynamicLink* m_dynamicBR;
    cDynamicJoint* m_jointSuspensionBR;
    cDynamicJoint* m_jointBR;

    //! sensor lines
    double m_sensorValues[ROVER_NUM_SENSORS];
    chai3d::cVector3d m_sensorPoints[ROVER_NUM_SENSORS][2];
    chai3d::cGenericObject* m_sensorNose;
};


//===========================================================================
/*!
    \class      cBlock

    \brief      
    cBlock defines a free block.
*/
//===========================================================================
class cBlock
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
public:

    //! Constructor of cBlock.
    cBlock(cDynamicWorld* a_world, 
           double a_sizeX,
           double a_sizeY,
           double a_sizeZ,
           chai3d::cVector3d a_pos = chai3d::cVector3d(0,0,0), 
           chai3d::cMatrix3d a_rot = chai3d::cIdentity3d());

    //! Destructor of cBlock.
    virtual ~cBlock() {};
    

    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
public:

    //! base node
    cDynamicBase* m_dynamicBase;

    //! dynamic body
    cDynamicLink* m_dynamicBody;
    cDynamicJoint* m_jointBodyX;
    cDynamicJoint* m_jointBodyY;
    cDynamicJoint* m_jointBodyZ;
    cDynamicJoint* m_jointBodyS;
};




//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
