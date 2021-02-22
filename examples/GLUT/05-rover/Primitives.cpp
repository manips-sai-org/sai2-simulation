//===========================================================================
/*
    This file is part of the Newton dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1289 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "Primitives.h"
//---------------------------------------------------------------------------
using namespace chai3d; 
using namespace std; 
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cRover.
*/
//===========================================================================
cRover::cRover(cDynamicWorld* a_world, 
    double a_size,
    chai3d::cVector3d a_pos, 
    chai3d::cMatrix3d a_rot)
{
    double size = a_size;
    cMultiMesh* collisionModel;
    cMultiMesh* graphicalModel;
    cMesh* mesh;

    // create default material
    cDynamicMaterial* dynMaterial = new cDynamicMaterial();
    dynMaterial->setDynamicFriction(1.0);
    dynMaterial->setStaticFriction(1.0);

    double defaultRadius = 0.001 * size;
    double defaultError = 0.001 * size;

    // init controller settings
    m_desiredVelocity      = 0.0;
    m_desiredDirectionDeg  = 20.0;
    m_desiredBrake         = 0.0;

    // create base node
    m_dynamicBase = a_world->newBaseObject(cVector3d(0,0,0), cIdentity3d());


    /////////////////////////////////////////////////////////////////////////
    // BODY
    /////////////////////////////////////////////////////////////////////////

    // create body and links
    m_dynamicBody = m_dynamicBase->newLink(dynMaterial);
    cVector3d inertia(0.1, 0.1, 0.1);
    cVector3d poscofm(0.0, 0.0, 0.0);
    m_dynamicBody->setMassProperties(1.0, 
                                     inertia,
                                     poscofm);

    // create joints
    m_jointBodyX = m_dynamicBody->newJoint(DYN_JOINT_PRISMATIC, DYN_AXIS_X);
    m_jointBodyY = m_dynamicBody->newJoint(DYN_JOINT_PRISMATIC, DYN_AXIS_Y);
    m_jointBodyZ = m_dynamicBody->newJoint(DYN_JOINT_PRISMATIC, DYN_AXIS_Z);
    m_jointBodyS = m_dynamicBody->newJoint(DYN_JOINT_SPHERICAL);

    // collision model
    collisionModel = new cMultiMesh();
    m_dynamicBody->setCollisionModel(collisionModel);
    mesh = collisionModel->newMesh();
    cCreateBox(mesh, 1.5*size*ROVER_LENGTH, 1.0*size*ROVER_WIDTH, 1.2*size*ROVER_HEIGHT);
    m_dynamicBody->buildCollisionHull(defaultRadius, defaultError);

    // graphical model
    graphicalModel = collisionModel->copy();
    graphicalModel->m_material->setWhite();
    m_dynamicBody->setImageModel(graphicalModel);

    // link body
    m_dynamicBase->linkChild(m_dynamicBody, a_pos, a_rot);


    /////////////////////////////////////////////////////////////////////////
    // SENSORS ON BODY
    /////////////////////////////////////////////////////////////////////////
    
    for (int i=0; i<ROVER_NUM_SENSORS; i++)
    {
        m_sensorValues[i] = 0.0;
    }

    double height = -0.22;
    double rayStart, rayLength, angleDeg;

    // sensor 0:
    rayStart = 0.8; rayLength = 0.6; angleDeg = 170;
    m_sensorPoints[0][0] = cVector3d(rayStart * cCosDeg(angleDeg), rayStart * cSinDeg(angleDeg), height); 
    m_sensorPoints[0][1] = cVector3d((rayStart+rayLength) * cCosDeg(angleDeg), (rayStart+rayLength) * cSinDeg(angleDeg), height);
    
    // sensor 2:
    rayStart = 0.4; rayLength = 0.6; angleDeg = 90;
    m_sensorPoints[2][0] = cVector3d(rayStart * cCosDeg(angleDeg), rayStart * cSinDeg(angleDeg), height); 
    m_sensorPoints[2][1] = cVector3d((rayStart+rayLength) * cCosDeg(angleDeg), (rayStart+rayLength) * cSinDeg(angleDeg), height);

    // sensor 1:
    m_sensorPoints[1][0] = m_sensorPoints[2][0] - cVector3d(0.3, 0.0, 0.0);
    m_sensorPoints[1][1] = m_sensorPoints[2][1] - cVector3d(0.3, 0.0, 0.0);

    // sensor 3:
    m_sensorPoints[3][0] = m_sensorPoints[2][0] + cVector3d(0.3, 0.0, 0.0);
    m_sensorPoints[3][1] = m_sensorPoints[2][1] + cVector3d(0.3, 0.0, 0.0);

    // sensor 4:
    rayStart = 0.7; rayLength = 0.6; angleDeg = 25;
    m_sensorPoints[4][0] = cVector3d(rayStart * cCosDeg(angleDeg), rayStart * cSinDeg(angleDeg), height); 
    m_sensorPoints[4][1] = cVector3d((rayStart+rayLength) * cCosDeg(angleDeg), (rayStart+rayLength) * cSinDeg(angleDeg), height);

    // sensor 5:
    rayStart = 0.8; rayLength = 0.6; angleDeg = 6;
    m_sensorPoints[5][0] = cVector3d(rayStart * cCosDeg(angleDeg), rayStart * cSinDeg(angleDeg), height); 
    m_sensorPoints[5][1] = cVector3d((rayStart+rayLength) * cCosDeg(angleDeg), (rayStart+rayLength) * cSinDeg(angleDeg), height);

    //

    // sensor 6:
    rayStart = 0.8; rayLength = 0.6; angleDeg =-6;
    m_sensorPoints[6][0] = cVector3d(rayStart * cCosDeg(angleDeg), rayStart * cSinDeg(angleDeg), height); 
    m_sensorPoints[6][1] = cVector3d((rayStart+rayLength) * cCosDeg(angleDeg), (rayStart+rayLength) * cSinDeg(angleDeg), height);

    // sensor 7:
    rayStart = 0.7; rayLength = 0.6; angleDeg =-25;
    m_sensorPoints[7][0] = cVector3d(rayStart * cCosDeg(angleDeg), rayStart * cSinDeg(angleDeg), height); 
    m_sensorPoints[7][1] = cVector3d((rayStart+rayLength) * cCosDeg(angleDeg), (rayStart+rayLength) * cSinDeg(angleDeg), height);

    // sensor 9:
    rayStart = 0.4; rayLength = 0.6; angleDeg =-90;
    m_sensorPoints[9][0] = cVector3d(rayStart * cCosDeg(angleDeg), rayStart * cSinDeg(angleDeg), height); 
    m_sensorPoints[9][1] = cVector3d((rayStart+rayLength) * cCosDeg(angleDeg), (rayStart+rayLength) * cSinDeg(angleDeg), height);

    // sensor 8:
    m_sensorPoints[8][0] = m_sensorPoints[9][0] + cVector3d(0.4, 0.0, 0.0);
    m_sensorPoints[8][1] = m_sensorPoints[9][1] + cVector3d(0.4, 0.0, 0.0);

    // sensor 10:
    m_sensorPoints[10][0] = m_sensorPoints[9][0] - cVector3d(0.4, 0.0, 0.0);
    m_sensorPoints[10][1] = m_sensorPoints[9][1] - cVector3d(0.4, 0.0, 0.0);

    // sensor 11:
    rayStart = 0.8; rayLength = 0.6; angleDeg =-170;
    m_sensorPoints[11][0] = cVector3d(rayStart * cCosDeg(angleDeg), rayStart * cSinDeg(angleDeg), height); 
    m_sensorPoints[11][1] = cVector3d((rayStart+rayLength) * cCosDeg(angleDeg), (rayStart+rayLength) * cSinDeg(angleDeg), height);

    m_sensorNose = new cGenericObject();
    for (int i=0; i<ROVER_NUM_SENSORS; i++)
    {
        cShapeLine* line = new cShapeLine(m_sensorPoints[i][0], m_sensorPoints[i][1]);
        line->m_colorPointA.setRed();
        line->m_colorPointB.setRed();
        line->setLineStipple(2, 0x00ff);
        m_sensorNose->addChild(line);
    }

    /////////////////////////////////////////////////////////////////////////
    // WHEEL MODEL
    /////////////////////////////////////////////////////////////////////////

    cVector3d pos(0.0, size * 0.5 * ROVER_WHEEL_RADIUS, 0.0);
    cMatrix3d rot;
    rot.identity();

    // collision model
    cMultiMesh* wheelModel = new cMultiMesh();
    mesh = wheelModel->newMesh();
    rot.rotateAboutLocalAxisDeg(1,0,0,90);

    cCreateCylinder(mesh, 
        size * ROVER_WHEEL_RADIUS, 
        size * ROVER_WHEEL_RADIUS,
        16,
        1,
        1,
        true,
        true,
        pos,
        rot);

    double suspensionLimitMin =-size*0.5*ROVER_WHEEL_RADIUS;
    double suspensionLimitMax = size*0.2*ROVER_WHEEL_RADIUS;


    /////////////////////////////////////////////////////////////////////////
    // FRONT LEFT WHEEL AND SUSPENSION
    /////////////////////////////////////////////////////////////////////////

    // create body and links
    m_dynamicFL = m_dynamicBase->newLink(dynMaterial);
    inertia.set(0.01, 0.01, 0.01);
    poscofm.set(0.0, 0.0, 0.0);
    m_dynamicFL->setMassProperties(0.1, inertia, poscofm);

    m_jointSuspensionFL = m_dynamicFL->newJoint(DYN_JOINT_PRISMATIC, DYN_AXIS_Z);
    m_jointSuspensionFL->setJointLimits(suspensionLimitMin, suspensionLimitMax, defaultError);
    m_jointDirectionFL = m_dynamicFL->newJoint(DYN_JOINT_REVOLUTE, DYN_AXIS_Z);
    m_jointFL = m_dynamicFL->newJoint(DYN_JOINT_REVOLUTE, DYN_AXIS_Y);

    // collision model
    collisionModel = wheelModel->copy();
    m_dynamicFL->setCollisionModel(collisionModel);
    m_dynamicFL->buildCollisionHull(defaultRadius, defaultError);

    // graphical model
    graphicalModel = collisionModel->copy();
    graphicalModel->m_material->setWhite();
    m_dynamicFL->setImageModel(graphicalModel);

    // link body
    cVector3d child(size*0.5*ROVER_LENGTH, size*0.5*ROVER_WIDTH, -size* (1*ROVER_WHEEL_RADIUS + 0.5*ROVER_HEIGHT));
    cMatrix3d identity = cIdentity3d();
    m_dynamicBody->linkChild(m_dynamicFL, child, identity);


    /////////////////////////////////////////////////////////////////////////
    // FRONT RIGHT WHEEL AND SUSPENSION
    /////////////////////////////////////////////////////////////////////////

    // create body and links
    m_dynamicFR = m_dynamicBase->newLink(dynMaterial);
    inertia.set(0.01, 0.01, 0.01);
    poscofm.set(0.0, 0.0, 0.0);
    m_dynamicFR->setMassProperties(0.1, inertia, poscofm);

    m_jointSuspensionFR = m_dynamicFR->newJoint(DYN_JOINT_PRISMATIC, DYN_AXIS_Z);
    m_jointSuspensionFR->setJointLimits(suspensionLimitMin, suspensionLimitMax, defaultError);
    m_jointDirectionFR = m_dynamicFR->newJoint(DYN_JOINT_REVOLUTE, DYN_AXIS_Z);
    m_jointFR = m_dynamicFR->newJoint(DYN_JOINT_REVOLUTE, DYN_AXIS_Y);

    // collision model
    collisionModel = wheelModel->copy();
    m_dynamicFR->setCollisionModel(collisionModel);
    m_dynamicFR->buildCollisionHull(defaultRadius, defaultError);

    // graphical model
    graphicalModel = collisionModel->copy();
    graphicalModel->m_material->setWhite();
    m_dynamicFR->setImageModel(graphicalModel);

    // link body
    child.set(size*0.5*ROVER_LENGTH, -size*0.5*ROVER_WIDTH, -size* (1*ROVER_WHEEL_RADIUS + 0.5*ROVER_HEIGHT));
    m_dynamicBody->linkChild(m_dynamicFR, child, identity);


    /////////////////////////////////////////////////////////////////////////
    // BACK LEFT WHEEL AND SUSPENSION
    /////////////////////////////////////////////////////////////////////////

    // create body and links
    m_dynamicBL = m_dynamicBase->newLink(dynMaterial);
    inertia.set(0.01, 0.01, 0.01);
    poscofm.set(0.0, 0.0, 0.0);
    m_dynamicBL->setMassProperties(0.1, inertia, poscofm);

    m_jointSuspensionBL = m_dynamicBL->newJoint(DYN_JOINT_PRISMATIC, DYN_AXIS_Z);
    m_jointSuspensionBL->setJointLimits(suspensionLimitMin, suspensionLimitMax, defaultError);
    m_jointBL = m_dynamicBL->newJoint(DYN_JOINT_REVOLUTE, DYN_AXIS_Y);

    // collision model
    collisionModel = wheelModel->copy();
    m_dynamicBL->setCollisionModel(collisionModel);
    m_dynamicBL->buildCollisionHull(defaultRadius, defaultError);

    // graphical model
    graphicalModel = collisionModel->copy();
    graphicalModel->m_material->setWhite();
    m_dynamicBL->setImageModel(graphicalModel);

    // link body
    child.set (-size*0.5*ROVER_LENGTH, size*0.5*ROVER_WIDTH, -size* (1*ROVER_WHEEL_RADIUS + 0.5*ROVER_HEIGHT));
    m_dynamicBody->linkChild(m_dynamicBL, child, identity);


    /////////////////////////////////////////////////////////////////////////
    // BACK RIGHT WHEEL AND SUSPENSION
    /////////////////////////////////////////////////////////////////////////

    // create body and links
    m_dynamicBR = m_dynamicBase->newLink(dynMaterial);
    inertia.set(0.01, 0.01, 0.01);
    poscofm.set(0.0, 0.0, 0.0);
    m_dynamicBR->setMassProperties(0.1, inertia, poscofm);

    m_jointSuspensionBR = m_dynamicBR->newJoint(DYN_JOINT_PRISMATIC, DYN_AXIS_Z);
    m_jointSuspensionBR->setJointLimits(suspensionLimitMin, suspensionLimitMax, defaultError);
    m_jointBR = m_dynamicBR->newJoint(DYN_JOINT_REVOLUTE, DYN_AXIS_Y);

    // collision model
    collisionModel = wheelModel->copy();
    m_dynamicBR->setCollisionModel(collisionModel);
    m_dynamicBR->buildCollisionHull(defaultRadius, defaultError);

    // graphical model
    graphicalModel = collisionModel->copy();
    graphicalModel->m_material->setWhite();
    m_dynamicBR->setImageModel(graphicalModel);

    // link body
    child.set(-size*0.5*ROVER_LENGTH,-size*0.5*ROVER_WIDTH, -size* (1*ROVER_WHEEL_RADIUS + 0.5*ROVER_HEIGHT));
    m_dynamicBody->linkChild(m_dynamicBR, child, identity);
}


//===========================================================================
/*!
    Update control commands.
*/
//===========================================================================
void cRover::updateController()
{
    double pos, vel, force;

    // update suspensions
    pos = m_jointSuspensionBR->getPos();
    vel = m_jointSuspensionBR->getVel();
    force = -ROVER_SUSPENSION_KS - ROVER_SUSPENSION_KD * vel;
    m_jointSuspensionBR->setForce(force);

    pos = m_jointSuspensionBL->getPos();
    vel = m_jointSuspensionBL->getVel();
    force = -ROVER_SUSPENSION_KS - ROVER_SUSPENSION_KD * vel;
    m_jointSuspensionBL->setForce(force);

    pos = m_jointSuspensionFR->getPos();
    vel = m_jointSuspensionFR->getVel();
    force = -ROVER_SUSPENSION_KS - ROVER_SUSPENSION_KD * vel;
    m_jointSuspensionFR->setForce(force);

    pos = m_jointSuspensionFL->getPos();
    vel = m_jointSuspensionFL->getVel();
    force = -ROVER_SUSPENSION_KS - ROVER_SUSPENSION_KD * vel;
    m_jointSuspensionFL->setForce(force);

    // update direction
    double const KS = 1.0;
    double const KD = 0.5;
    pos = m_jointDirectionFL->getPos();
    vel = m_jointDirectionFL->getVel();
    force = KS * (m_desiredDirectionDeg - cRadToDeg(pos)) - KD * vel;
    m_jointDirectionFL->setForce(force);

    pos = m_jointDirectionFR->getPos();
    vel = m_jointDirectionFR->getVel();
    force = KS * (m_desiredDirectionDeg - cRadToDeg(pos)) - KD * vel;
    m_jointDirectionFR->setForce(force);

    // update velocity and braking
    const double KB = 1.0;
    const double KV = 0.5;
    const double KF = 10.2;

    vel = m_jointBL->getVel();
    force = KF * m_desiredVelocity - (KV + KB*m_desiredBrake) * vel;
    m_jointBL->setForce(force);

    vel = m_jointBR->getVel();
    force = KF * m_desiredVelocity - (KV + KB*m_desiredBrake) * vel;
    m_jointBR->setForce(force);

    vel = m_jointFL->getVel();
    force = KF * m_desiredVelocity - (KV + KB*m_desiredBrake) * vel;
    m_jointFL->setForce(force);

    vel = m_jointFR->getVel();
    force = KF * m_desiredVelocity - (KV + KB*m_desiredBrake) * vel;
    m_jointFR->setForce(force);
}


//===========================================================================
/*!
     Update sensor data
*/
//===========================================================================
void cRover::updateSensorData(cWorld* a_world)
{
    for (int i=0; i<ROVER_NUM_SENSORS; i++)
    {
        
        cCollisionRecorder collisionRecorder;
        cCollisionSettings collisionSettings;
       
        cVector3d pos0 = m_dynamicBody->getLocalPos() + m_dynamicBody->getLocalRot() * m_sensorPoints[i][0];
        cVector3d pos1 = m_dynamicBody->getLocalPos() + m_dynamicBody->getLocalRot() * m_sensorPoints[i][1];

        bool result = a_world->computeCollisionDetection(
            pos0,
            pos1,
            collisionRecorder,
            collisionSettings);
        
        if (result)
        {
            double distance = sqrt(collisionRecorder.m_nearestCollision.m_squareDistance);
            double range = cDistance(pos0, pos1);
            m_sensorValues[i] = 1.0 - ((1.0/range) * distance);
        }
        else
        {
            m_sensorValues[i] = 0.0;
        }
    }
}


//===========================================================================
/*!
    Constructor of cBlock.
*/
//===========================================================================
cBlock::cBlock(cDynamicWorld* a_world, 
    double a_sizeX,
    double a_sizeY,
    double a_sizeZ,
    cVector3d a_pos, 
    cMatrix3d a_rot)
{
    cMultiMesh* collisionModel;
    cMultiMesh* graphicalModel;
    cMesh* mesh;

    // create default material
    cDynamicMaterial* dynMaterial = new cDynamicMaterial();
    dynMaterial->setDynamicFriction(1.0);
    dynMaterial->setStaticFriction(1.0);

    double defaultRadius = 0.01 * cMin(cMin(a_sizeX, a_sizeY), a_sizeZ);
    double defaultError = 0.01 * cMin(cMin(a_sizeX, a_sizeY), a_sizeZ);

    // create base node
    m_dynamicBase = a_world->newBaseObject(cVector3d(0,0,0), cIdentity3d());


    /////////////////////////////////////////////////////////////////////////
    // BODY
    /////////////////////////////////////////////////////////////////////////

    // create body and links
    m_dynamicBody = m_dynamicBase->newLink(dynMaterial);
    cVector3d inertia(0.1, 0.1, 0.1);
    cVector3d poscofm(0.0, 0.0, 0.0);
    m_dynamicBody->setMassProperties(1.0, inertia, poscofm);

    // create joints
    m_jointBodyX = m_dynamicBody->newJoint(DYN_JOINT_PRISMATIC, DYN_AXIS_X);
    m_jointBodyY = m_dynamicBody->newJoint(DYN_JOINT_PRISMATIC, DYN_AXIS_Y);
    m_jointBodyZ = m_dynamicBody->newJoint(DYN_JOINT_PRISMATIC, DYN_AXIS_Z);
    m_jointBodyS = m_dynamicBody->newJoint(DYN_JOINT_SPHERICAL);

    // collision model
    collisionModel = new cMultiMesh();
    m_dynamicBody->setCollisionModel(collisionModel);
    mesh = collisionModel->newMesh();
    mesh->m_material->setWhite();
    cCreateBox(mesh, a_sizeX, a_sizeY, a_sizeZ);
    m_dynamicBody->buildCollisionHull(defaultRadius, defaultError);

    // graphical model
    graphicalModel = collisionModel->copy();
    graphicalModel->createAABBCollisionDetector(0.0);
    m_dynamicBody->setImageModel(graphicalModel);

    // link body
    m_dynamicBase->linkChild(m_dynamicBody, a_pos, a_rot);
}
