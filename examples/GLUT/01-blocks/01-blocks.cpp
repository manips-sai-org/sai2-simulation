//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 2540 $
*/
//===========================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
#include "dynamics3d.h"
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------
#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled 
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//------------------------------------------------------------------------------
// MOUSE STATE
//------------------------------------------------------------------------------

// mouse interaction state
enum cSimulationMouseMode
{
    C_SIMULATION_MOUSE_CAMERA,
    C_SIMULATION_MOUSE_FORCE
};


//------------------------------------------------------------------------------
// CHAI3D
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera that renders the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cSpotLight *light;

// flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool simulationFinished = false;

// mouse interaction
cGenericObject* mouseObject;
cShapeLine* mouseLine;
cShapeSphere* mouseSphere0;
cShapeSphere* mouseSphere1;

// mouse interaction
bool flagCameraInMotion = false;
bool flagApplyForce = false;
cSimulationMouseMode mouseMode = C_SIMULATION_MOUSE_CAMERA;
int mouseX = 0;
int mouseY = 0;
int mouseButton = 0;
cDynamicLink* selectLink;
chai3d::cVector3d selectLocalPos;
cDynPositionForceProperty* lastForceProperty;
cDynObject* lastDynObject;

// camera position in spherical coordinates
double cameraAngleH;
double cameraAngleV;
double cameraDistance;
cVector3d cameraPosition(0.0, 0.0, 0.0);

// information about computer screen and GLUT display window
int screenW;
int screenH;
int windowW;
int windowH;
int windowPosX;
int windowPosY;


//---------------------------------------------------------------------------
// DYNAMICS3D
//---------------------------------------------------------------------------

// dynamic world
cDynamicWorld* dynamicWorld;

// ground
cDynamicBase* ground;

// free object
cDynamicBase* object;

// joints
cDynamicJoint* jointBodyX;
cDynamicJoint* jointBodyY;
cDynamicJoint* jointBodyZ;
cDynamicJoint* jointBodyS;


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void resizeWindow(int w, int h);

// callback when a key is pressed
void keySelect(unsigned char key, int x, int y);

// main simulation loop
void updateSimulation(void);

// callback to render graphic scene
void updateGraphics(void);

// callback of GLUT timer
void graphicsTimer(int data);

// function that closes the application
void close(void);

// update camera position
void updateCameraPosition();

// callback to handle mouse click
void mouseClick(int button, int state, int x, int y);

// callback to handle mouse motion
void mouseMove(int x, int y);


//==============================================================================
/*
    DEMO:    01-blocks.cpp

    This application illustrates how to create a dynamic cube and a ground
    using the Stanford Dynamics library.

    Interactions with the environment can be operated with the computer mouse.
*/
//==============================================================================

int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "DYNAMICS3D" << endl;
    cout << "Demo: 01-blocks" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Mouse Options:" << endl << endl;
    cout << "    - Move camera" << endl;
    cout << "    - Apply force onto object" << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[1] - Show/Hide contact normal forces" << endl;
    cout << "[x] - Exit application" << endl;
    cout << endl << endl;


    //--------------------------------------------------------------------------
    // OPENGL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLUT
    glutInit(&argc, argv);

    // retrieve  resolution of computer display and position window accordingly
    screenW = glutGet(GLUT_SCREEN_WIDTH);
    screenH = glutGet(GLUT_SCREEN_HEIGHT);
    windowW = (int)(0.8 * screenH);
    windowH = (int)(0.5 * screenH);
    windowPosY = (screenH - windowH) / 2;
    windowPosX = windowPosY; 

    // initialize the OpenGL GLUT window
    glutInitWindowPosition(windowPosX, windowPosY);
    glutInitWindowSize(windowW, windowH);

    if (stereoMode == C_STEREO_ACTIVE)
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STEREO);
    else
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

    // create display context and initialize GLEW library
    glutCreateWindow(argv[0]);

#ifdef GLEW_VERSION
    // initialize GLEW
    glewInit();
#endif

    // setup GLUT options
    glutDisplayFunc(updateGraphics);
    glutKeyboardFunc(keySelect);
    glutReshapeFunc(resizeWindow);
    glutSetWindowTitle("CHAI3D");
    glutMouseFunc(mouseClick);
    glutMotionFunc(mouseMove);

    // set fullscreen mode
    if (fullscreen)
    {
        glutFullScreen();
    }


    //--------------------------------------------------------------------------
    // SETUP CHAI3D ENVIORNMENT
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setWhite();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set( cVector3d (1.0, 0.0, 0.5),    // camera position (eye)
                 cVector3d (0.0, 0.0, 0.0),    // look at position (target)
                 cVector3d (0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10.0);

    // initialize camera position
    cameraAngleH = 0;
    cameraAngleV = 30;
    cameraDistance = 1.0;
    updateCameraPosition();

    // create a directional light source
    light = new cSpotLight(world);

    // insert light source inside world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);                   

    // set light source position
    light->setLocalPos(1.0, 1.0, 1.0);

    // define direction of light beam
    light->setDir(-1.0,-1.0,-1.0); 
    
    // enable shadow casting for this light source
    light->setShadowMapEnabled(true);
    light->setShadowMapProperties(0.1, 2.0);

    // define cutoff angle of spotlight
    light->setCutOffAngleDeg(25);

    // create a background
    cBackground* background = new cBackground();
    camera->m_backLayer->addChild(background);

    // enable shadow casting
    // camera->setUseShadowCasting(true);

    // set background properties
    background->setCornerColors(cColorf(1.0, 1.0, 1.0),
                                cColorf(1.0, 1.0, 1.0),
                                cColorf(0.8, 0.8, 0.8),
                                cColorf(0.8, 0.8, 0.8));


    //-----------------------------------------------------------------------
    // MOUSE OBJECT
    //-----------------------------------------------------------------------

    // size of 3D cursor
    double mouseSphereRadius = 0.01;

    // setup double cursor model with line to illustrate external forces
    mouseSphere0 = new cShapeSphere(mouseSphereRadius);
    mouseSphere0->m_material->setBlueRoyal();

    mouseSphere1 = new cShapeSphere(mouseSphereRadius);
    mouseSphere1->m_material->setBlueRoyal();

    mouseLine = new cShapeLine();
    mouseLine->m_colorPointA.setGray();
    mouseLine->m_colorPointB.setGray();

    // add model to world
    mouseObject = new cGenericObject();
    world->addChild(mouseObject);
    mouseObject->addChild(mouseSphere0);
    mouseObject->addChild(mouseSphere1);
    mouseObject->addChild(mouseLine);

    // hide model for now
    mouseObject->setShowEnabled(false, true);


    //-----------------------------------------------------------------------
    // SETUP DYNAMIC3D ENVIRONMENT
    //-----------------------------------------------------------------------

    // create an dynamic world
    dynamicWorld = new cDynamicWorld(world);

    // add dynamic world as a node to the CHAI3D world
    world->addChild(dynamicWorld);

    // set some gravity
    dynamicWorld->setGravity(0.0, 0.0, -9.81);
  
    // collision settings
    double error = 0.0001;   // tolerance error for contact detection
    double radius = 0.0001;  // radius zone for contact detection

    // default material
    cDynamicMaterial* dynMaterial = new cDynamicMaterial();
    dynMaterial->setDynamicFriction(1.0);
    dynMaterial->setStaticFriction(1.0);


    //-----------------------------------------------------------------------
    // CREATE GROUND
    //-----------------------------------------------------------------------

    // dimension of ground
    double sizeGround = 0.5;

    // create a base node
    ground = dynamicWorld->newBaseObject(cVector3d(0,0,0), cIdentity3d());

    // create link for our ground. No joints will be defined as the ground in static!
    cDynamicLink* linkGround = ground->newLink(dynMaterial);
   
    // create a multimesh structure to describe the shape of the ground.
    cMultiMesh* groundModel = new cMultiMesh();

    // create new mesh inside the multimesh structure
    cMesh* meshGround = groundModel->newMesh();

    // build a plane inside the mesh structure composed of triangles
    cCreatePlane(meshGround, sizeGround, sizeGround);

    // assign a material color to the mesh
    meshGround->m_material->setBlueCornflower();

    // assign the object model as the collision model
    linkGround->setCollisionModel(groundModel);

    // assign the object model as the graphical model
    linkGround->setImageModel(groundModel);

    // create a convex hull or triangle model to handle collision detection
    linkGround->buildCollisionHull(radius, error);
    //linkGround->buildCollisionTriangles(radius, error);

    // connect link to basenode
    ground->linkChild(linkGround, cVector3d(0,0,0), cIdentity3d());


    //-----------------------------------------------------------------------
    // CREATE BLOCK
    //-----------------------------------------------------------------------

    // dimensions of our block
    double sizeX = 0.1;
    double sizeY = 0.1;
    double sizeZ = 0.1;

    // set position and orientation of object in scene
    cVector3d pos(0.0, 0.0, 0.5);
    cMatrix3d rot;
    rot.identity();
    rot.rotateAboutGlobalAxisDeg(0,0,1, 20);

    // create base node
    object = dynamicWorld->newBaseObject(cVector3d(0,0,0), cIdentity3d());

    // create link for our object
    cDynamicLink* linkObject = object->newLink(dynMaterial);

    // set mass properties
    cVector3d inertia(0.1, 0.1, 0.1);
    cVector3d poscofm(0.0, 0.0, 0.0);
    linkObject->setMassProperties(1.0,        // mass
                                  inertia,    // inertia properties
                                  poscofm);   // position of center of mass

    // create 3 prismatic joints (x,y,z)
    jointBodyX = linkObject->newJoint(DYN_JOINT_PRISMATIC, DYN_AXIS_X);
    jointBodyY = linkObject->newJoint(DYN_JOINT_PRISMATIC, DYN_AXIS_Y);
    jointBodyZ = linkObject->newJoint(DYN_JOINT_PRISMATIC, DYN_AXIS_Z);

    // create 1 spherical joint
    jointBodyS = linkObject->newJoint(DYN_JOINT_SPHERICAL);

    // set some damping
    jointBodyX->setDamping(5);
    jointBodyY->setDamping(5);
    jointBodyZ->setDamping(5);
    jointBodyS->setDamping(1);

    // create a multimesh structure to describe the shape of our object
    cMultiMesh* objectModel = new cMultiMesh();

    // create new mesh inside the multimesh structure
    cMesh* meshObject = objectModel->newMesh();

    // create a block
    cCreateBox(meshObject, sizeX, sizeY, sizeZ);
    meshObject->m_material->setWhiteAliceBlue();

    // assign the object model as the collision model
    linkObject->setCollisionModel(objectModel);

    // assign the object model as the graphical model
    linkObject->setImageModel(objectModel);

    // create a convex hull or triangle model to handle collision detection
    linkObject->buildCollisionHull(radius, error);
    //linkObject->buildCollisionTriangles(radius, error);

    // connect link to basenode
    object->linkChild(linkObject, pos, rot);

    // create collision detector for interaction with graphical object
    meshObject->createAABBCollisionDetector(0.0);


    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------

    // simulation in now running
    simulationRunning = true;

    // create a thread which starts the main haptics rendering loop
    cThread* simulationThread = new cThread();
    simulationThread->start(updateSimulation, CTHREAD_PRIORITY_GRAPHICS);


    //-----------------------------------------------------------------------
    // START GRAPHICS
    //-----------------------------------------------------------------------

    // start the main graphics rendering loop
    glutTimerFunc(30, graphicsTimer, 0);
    glutMainLoop();

    // close everything
    close();

    // exit
    return (0);
}

//------------------------------------------------------------------------------

void resizeWindow(int w, int h)
{
    windowW = w;
    windowH = h;
}

//------------------------------------------------------------------------------

void updateCameraPosition()
{
    // check values
    if (cameraDistance < 0.1) { cameraDistance = 0.1; }
    if (cameraAngleV > 89) { cameraAngleV = 89; }
    if (cameraAngleV < -89) { cameraAngleV = -89; }

    // compute position of camera in space
    cVector3d pos = cAdd(
        cameraPosition,
        cVector3d(
        cameraDistance * cCosDeg(cameraAngleH) * cCosDeg(cameraAngleV),
        cameraDistance * cSinDeg(cameraAngleH) * cCosDeg(cameraAngleV),
        cameraDistance * cSinDeg(cameraAngleV)
        )
        );

    // compute lookat position
    cVector3d lookat = cameraPosition;

    // define role orientation of camera
    cVector3d up(0.0, 0.0, 1.0);

    // set new position to camera
    camera->set(pos, lookat, up);

    // recompute global positions
    world->computeGlobalPositions(true);
}

//------------------------------------------------------------------------------

void keySelect(unsigned char key, int x, int y)
{
    // option ESC: exit
    if ((key == 27) || (key == 'x'))
    {
        close();
        exit(0);
    }

    // option '1': show/hide contact normals
    if (key == '1')
    {
        bool status = object->getShowContactNormalForces();
        object->setShowContactNormalForces(!status);
        ground->setShowContactNormalForces(!status);
    }

    // option f: toggle fullscreen
    if (key == 'f')
    {
        if (fullscreen)
        {
            windowPosX = glutGet(GLUT_INIT_WINDOW_X);
            windowPosY = glutGet(GLUT_INIT_WINDOW_Y);
            windowW = glutGet(GLUT_INIT_WINDOW_WIDTH);
            windowH = glutGet(GLUT_INIT_WINDOW_HEIGHT);
            glutPositionWindow(windowPosX, windowPosY);
            glutReshapeWindow(windowW, windowH);
            fullscreen = false;
        }
        else
        {
            glutFullScreen();
            fullscreen = true;
        }
    }

    // option m: toggle vertical mirroring
    if (key == 'm')
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }
}

//------------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }
}

//------------------------------------------------------------------------------

void graphicsTimer(int data)
{
    // inform GLUT to refresh graphics
    if (simulationRunning)
    {
        glutPostRedisplay();
    }

    glutTimerFunc(30, graphicsTimer, 0);
}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
    // render world
    camera->renderView(windowW, windowH);

    // swap buffers
    glutSwapBuffers();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error:  %s\n" << gluErrorString(err);
}

//------------------------------------------------------------------------------

void updateSimulation(void)
{
    // simulation clock
    cPrecisionClock simClock;
    simClock.start(true);
    simulationFinished = false;

    // main haptic simulation loop
    while(simulationRunning)
    {
        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // retrieve simulation time and compute next interval
        double time = simClock.getCurrentTimeSeconds();
        double nextSimInterval = cClamp(time, 0.00001, 0.001);

        // reset clock
        simClock.reset();
        simClock.start();

        /////////////////////////////////////////////////////////////////////////
        // INTERACTION FORCE
        /////////////////////////////////////////////////////////////////////////
        if ((lastForceProperty != NULL) && (lastDynObject != NULL))
        {
            lastDynObject->force.remove(lastForceProperty);
            lastDynObject = NULL;
            delete lastForceProperty;
            lastForceProperty = NULL;
        }

        if (flagApplyForce)
        {
            cVector3d pos0 = selectLink->getGlobalPos() + selectLink->getGlobalRot() * selectLocalPos;
            mouseSphere0->setLocalPos(pos0);
            mouseLine->m_pointA = mouseSphere0->getLocalPos();

            double kp = 100.0;
            double kr = 5.0;
            cVector3d force = kp * (mouseSphere1->getLocalPos() - mouseSphere0->getLocalPos());

            cDynVector3 dynLocalPos(selectLocalPos.x(), selectLocalPos.y(), selectLocalPos.z());
            cDynVector3 dynForce(force.x(), force.y(), force.z());

            lastForceProperty = new cDynPositionForceProperty(dynLocalPos, dynForce);
            lastForceProperty->global(true);
            lastForceProperty->state(true);
            lastDynObject = selectLink->m_dynObject;
            lastDynObject->force.add(lastForceProperty);

            cVector3d axis;
            double angle;
            cMatrix3d rotation = selectLink->getGlobalRot();
            cMatrix3d desiredRotation = cIdentity3d();

            cMatrix3d deltaRotation = cTranspose(rotation) * desiredRotation;
            deltaRotation.toAxisAngle(axis, angle);
            cVector3d torque = rotation * ((kr * angle) * axis);
            jointBodyS->setTorque(torque);
        }
        else
        {
            cVector3d torque(0,0,0);
            jointBodyS->setTorque(torque);
        }

        /////////////////////////////////////////////////////////////////////////
        // UPDATE DYNAMICS
        /////////////////////////////////////////////////////////////////////////

        // update simulation
        dynamicWorld->updateDynamics(nextSimInterval);
    }

    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------

void mouseClick(int button, int state, int x, int y)
{
    /////////////////////////////////////////////////////////////////////////
    // MOUSE DOWN
    /////////////////////////////////////////////////////////////////////////
    if (state == GLUT_DOWN)
    {
        cCollisionRecorder collisionRecorder;
        cCollisionSettings collisionSettings;

        bool result = camera->selectWorld(x, 
            y,
            windowW,
            windowH,
            collisionRecorder,
            collisionSettings);

        if (result)
        {
            cDynamicLink* link = dynamic_cast<cDynamicLink*> (collisionRecorder.m_nearestCollision.m_object->getOwner()->getOwner());
            if (link != NULL)
            {
                selectLink = link;
                selectLocalPos = link->getImageModel()->getLocalPos() + link->getImageModel()->getLocalRot() * collisionRecorder.m_nearestCollision.m_localPos;   
                flagApplyForce = true;
                cVector3d pos = selectLink->getGlobalPos() + selectLink->getGlobalRot() * selectLocalPos;
                mouseObject->setShowEnabled(true);
                mouseSphere0->setLocalPos(pos);
                mouseSphere1->setLocalPos(pos);
                mouseLine->m_pointA = mouseSphere0->getLocalPos();
                mouseLine->m_pointB = mouseSphere1->getLocalPos();
                mouseMode = C_SIMULATION_MOUSE_FORCE;
            }
            else
            {
                flagApplyForce = false;
                mouseMode = C_SIMULATION_MOUSE_CAMERA;
            }
        }
        else
        {
            mouseMode = C_SIMULATION_MOUSE_CAMERA;
            flagCameraInMotion = true;
        }
    }

    /////////////////////////////////////////////////////////////////////////
    // MOUSE UP
    /////////////////////////////////////////////////////////////////////////
    else if (state == GLUT_UP)
    {
        flagCameraInMotion = false;
        flagApplyForce = false;
        cVector3d zero(0,0,0);
        mouseSphere0->setLocalPos(zero);
        mouseSphere1->setLocalPos(zero);
        mouseLine->m_pointA = mouseSphere0->getLocalPos();
        mouseLine->m_pointB = mouseSphere1->getLocalPos();
        mouseObject->setShowEnabled(false, true);
    }

    mouseX = x;
    mouseY = y;
    mouseButton = button;
}

//------------------------------------------------------------------------------

void mouseMove(int x, int y)
{
    /////////////////////////////////////////////////////////////////////////
    // MOVE CAMERA
    /////////////////////////////////////////////////////////////////////////
    if  (mouseMode == C_SIMULATION_MOUSE_CAMERA)
    {
        if (mouseButton == GLUT_RIGHT_BUTTON)
        {
            cameraDistance = cameraDistance - 0.01 * (y - mouseY);
        }

        else if (mouseButton == GLUT_LEFT_BUTTON)
        {
            cameraAngleH = cameraAngleH - (x - mouseX);
            cameraAngleV = cameraAngleV + (y - mouseY);
        }

        updateCameraPosition();
    }

    /////////////////////////////////////////////////////////////////////////
    // APPLY FORCE 
    /////////////////////////////////////////////////////////////////////////
    else if (mouseMode == C_SIMULATION_MOUSE_FORCE)
    {
        if (flagApplyForce)
        {
            cVector3d pos0 = selectLink->getGlobalPos() + selectLink->getGlobalRot() * selectLocalPos;

            cVector3d pos1 = mouseSphere1->getLocalPos();
            if (mouseButton == GLUT_RIGHT_BUTTON)
            {
                pos1 = pos1 + 0.01 * (x - mouseX) * camera->getRightVector();
                pos1 = pos1 + 0.01 * (y - mouseY) * camera->getLookVector();
            }
            else if (mouseButton == GLUT_LEFT_BUTTON)
            {
                pos1 = pos1 + 0.01 * (x - mouseX) * camera->getRightVector();
                pos1 = pos1 - 0.01 * (y - mouseY) * camera->getUpVector();
            }

            mouseSphere0->setLocalPos(pos0);
            mouseSphere1->setLocalPos(pos1);
            mouseLine->m_pointA = mouseSphere0->getLocalPos();
            mouseLine->m_pointB = mouseSphere1->getLocalPos();
        }
    }

    mouseX = x;
    mouseY = y;
}

//------------------------------------------------------------------------------
