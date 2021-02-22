//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2013, Artificial Intelligence Laboratory,
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
// DECLARED CONSTANTS
//------------------------------------------------------------------------------

// enable/disable stereo. (supported on OpenGL stereo graphic cards only) 
const bool USE_STEREO_DISPLAY = false;

// mouse interaction state
enum cMouseMode
{
    C_SIMULATION_MOUSE_CAMERA,
    C_SIMULATION_MOUSE_FORCE
};

// haptic device interaction state
enum cHapticMode
{
    C_HAPTIC_PROXY,
    C_HAPTIC_CONTROL
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

// a haptic device handler
cHapticDeviceHandler* handler;

// number of haptic devices
int numHapticDevices = 0;

cDynamicLink* linkObject1;
cDynamicJoint* jointBodyX1;
cDynamicJoint* jointBodyY1;
cDynamicJoint* jointBodyZ1;
cDynamicJoint* jointBodyS1;

cDynamicLink* linkObject2;
cDynamicJoint* jointBodyX2;
cDynamicJoint* jointBodyY2;
cDynamicJoint* jointBodyZ2;
cDynamicJoint* jointBodyS2;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice1;
cGenericHapticDevicePtr hapticDevice2;

// a cursor like tool
cToolCursor* tool1 = NULL;
cToolCursor* tool2 = NULL;

// last forces
double force1 = 0.0;
double force2 = 0.0;

// haptic state
cHapticMode hapticMode1 = C_HAPTIC_PROXY;
cHapticMode hapticMode2 = C_HAPTIC_PROXY;

// information about haptic device
cHapticDeviceInfo info1;
cHapticDeviceInfo info2;

// mouse interaction
cGenericObject* mouseObject;
cShapeLine* mouseLine;
cShapeSphere* mouseSphere0;
cShapeSphere* mouseSphere1;

// mouse interaction
bool flagCameraInMotion = false;
bool flagApplyForce = false;
cMouseMode mouseMode = C_SIMULATION_MOUSE_CAMERA;
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

// a scope to monitor position values of haptic device
cScope* scope;

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
    DEMO:    03-insert.cpp

    This application illustrates how to control two objects with two haptic 
    devices.
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
    glutMouseFunc(mouseClick);
    glutMotionFunc(mouseMove);
    glutSetWindowTitle("CHAI3D");

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

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.005);
    camera->setStereoFocalLength(0.5);

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
    light->m_shadowMap->setResolutionHigh();

    // define cutoff angle of spotlight
    light->setCutOffAngleDeg(25);

    // create a background
    cBackground* background = new cBackground();
    camera->m_backLayer->addChild(background);

    // enable shadow casting
    camera->setUseShadowCasting(true);

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


    //--------------------------------------------------------------------------
    // HAPTIC DEVICES
    //--------------------------------------------------------------------------

    // tool radius
    double toolRadius = 0.01;

    // set workspace size
    double workspaceSize = 0.5;
    double workspaceScaleFactor = 1.0;

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get number of haptic devices
    numHapticDevices = handler->getNumDevices();


    // get first haptic device if available
    if (numHapticDevices > 0)
    {
        // get a handle to the first haptic device
        handler->getDevice(hapticDevice1, 0);

        // retrieve information about the current haptic device
        info1 = hapticDevice1->getSpecifications();

        // if the device has a gripper, enable the gripper to simulate a user switch
        hapticDevice1->setEnableGripperUserSwitch(true);

        // create tool
        tool1 = new cToolCursor(world);
        tool1->setRadius(toolRadius);
        world->addChild(tool1);
        tool1->setHapticDevice(hapticDevice1);
        tool1->enableDynamicObjects(true);
        tool1->setWorkspaceRadius(workspaceSize);
        tool1->start();

        // read the scale factor between the physical workspace of the haptic
        // device and the virtual workspace defined for the tool

        workspaceScaleFactor = tool1->getWorkspaceScaleFactor();
    }

    // get second haptic device if available
    if (numHapticDevices > 1)
    {
        // get a handle to the first haptic device
        handler->getDevice(hapticDevice2, 1);

        // retrieve information about the current haptic device
        info2 = hapticDevice2->getSpecifications();

        // if the device has a gripper, enable the gripper to simulate a user switch
        hapticDevice2->setEnableGripperUserSwitch(true);

        // create tool
        tool2 = new cToolCursor(world);
        tool2->setRadius(toolRadius);
        world->addChild(tool2);
        tool2->setHapticDevice(hapticDevice2);
        tool2->enableDynamicObjects(true);
        tool2->setWorkspaceRadius(workspaceSize);
        tool2->start();
    }

    // stiffness properties
    double maxStiffness	= info1.m_maxLinearStiffness / workspaceScaleFactor;


    //-----------------------------------------------------------------------
    // SETUP DYNAMIC3D ENVIRONMENT
    //-----------------------------------------------------------------------

    // create an dynamic world
    dynamicWorld = new cDynamicWorld(world);

    // add dynamic world as a node to the CHAI3D world
    world->addChild(dynamicWorld);

    // set some gravity
    dynamicWorld->setGravity(0.0, 0.0, -1.0);

    // collision settings
    double error = 0.0001;   // tolerance error for contact detection
    double radius = 0.0001;  // radius zone for contact detection

    // default material
    cDynamicMaterial* dynMaterial = new cDynamicMaterial();
    dynMaterial->setDynamicFriction(0.1);
    dynMaterial->setStaticFriction(0.1);


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
    cCreateBox(meshGround, sizeGround, 0.05, 0.05, cVector3d(0.0, -0.08, 0.025));
    cCreateBox(meshGround, sizeGround, 0.05, 0.05, cVector3d(0.0,  0.08, 0.025));
    cCreateBox(meshGround, 0.05, sizeGround, 0.05, cVector3d(-0.08, 0.0, 0.020));
    cCreateBox(meshGround, 0.05, sizeGround, 0.05, cVector3d( 0.08, 0.0, 0.020));

    // assign a material color to the mesh
    meshGround->m_material->setBlueCornflower();

    // create bounding box
    meshGround->createAABBCollisionDetector(1.01 * toolRadius);

    // set haptic properties
    meshGround->m_material->setStiffness(0.3 * maxStiffness);
    meshGround->m_material->setStaticFriction(0.8);
    meshGround->m_material->setDynamicFriction(0.5);

    // assign the object model as the collision model
    linkGround->setCollisionModel(groundModel);

    // assign the object model as the graphical model
    linkGround->setImageModel(groundModel);

    // create a convex hull to handle collision detection
    linkGround->buildCollisionTriangles(radius, error);

    // connect link to basenode
    ground->linkChild(linkGround, cVector3d(0,0,0), cIdentity3d());


    //-----------------------------------------------------------------------
    // CREATE BLOCK 1
    //-----------------------------------------------------------------------

    if (true)
    {
        // dimensions of our block
        double sizeX = 0.1;
        double sizeY = 0.1;
        double sizeZ = 0.1;

        // set position and orientation of object in scene
        cVector3d pos(0.0, 0.2, 0.2);
        cMatrix3d rot;
        rot.identity();
        rot.rotateAboutGlobalAxisDeg(0,0,1, 20);

        // create base node
        object = dynamicWorld->newBaseObject(cVector3d(0,0,0), cIdentity3d());

        // create link for our object
        linkObject1 = object->newLink(dynMaterial);

        // set mass properties
        cVector3d inertia(0.005, 0.005, 0.005);
        cVector3d poscofm(0.0, 0.0, 0.0);
        linkObject1->setMassProperties(0.1,        // mass
                                      inertia,    // inertia properties
                                      poscofm);   // position of center of mass

        // create 3 prismatic joints (x,y,z)
        jointBodyX1 = linkObject1->newJoint(DYN_JOINT_PRISMATIC, DYN_AXIS_X);
        jointBodyY1 = linkObject1->newJoint(DYN_JOINT_PRISMATIC, DYN_AXIS_Y);
        jointBodyZ1 = linkObject1->newJoint(DYN_JOINT_PRISMATIC, DYN_AXIS_Z);

        // create 1 spherical joint
        jointBodyS1 = linkObject1->newJoint(DYN_JOINT_SPHERICAL);

        // set some damping
        jointBodyX1->setDamping(5);
        jointBodyY1->setDamping(5);
        jointBodyZ1->setDamping(5);
        jointBodyS1->setDamping(0.1);

        // create a multimesh structure to describe the shape of our object
        cMultiMesh* objectModel = new cMultiMesh();

        // create new mesh inside the multimesh structure
        cMesh* meshObject = objectModel->newMesh();

        // create a block
        //cCreateBox(meshObject, sizeX, sizeY, sizeZ);
        cCreatePipe(meshObject, 0.2, 0.04, 0.05, 8, 1, cVector3d(0.0, 0.0, -0.1));
        meshObject->m_material->setWhite();
        //meshObject->setShowFrame(true);

        // assign the object model as the collision model
        linkObject1->setCollisionModel(objectModel);

        // assign the object model as the graphical model
        linkObject1->setImageModel(objectModel);

        // create a convex hull to handle collision detection
        linkObject1->buildCollisionTriangles(radius, error);

        // connect link to basenode
        object->linkChild(linkObject1, pos, rot);

        // create collision detector for interaction with graphical object
        meshObject->createAABBCollisionDetector(1.01 * toolRadius);

        // set haptic properties
        meshObject->m_material->setStiffness(0.3 * maxStiffness);
        meshObject->m_material->setStaticFriction(0.8);
        meshObject->m_material->setDynamicFriction(0.6);
    }

    //-----------------------------------------------------------------------
    // CREATE BLOCK 1
    //-----------------------------------------------------------------------

    if (true)
    {
        // dimensions of our block
        double sizeX = 0.1;
        double sizeY = 0.1;
        double sizeZ = 0.1;

        // set position and orientation of object in scene
        cVector3d pos(0.0, -0.2, 0.2);
        cMatrix3d rot;
        rot.identity();
        rot.rotateAboutGlobalAxisDeg(0,0,1, 20);

        // create base node
        object = dynamicWorld->newBaseObject(cVector3d(0,0,0), cIdentity3d());

        // create link for our object
        linkObject2 = object->newLink(dynMaterial);

        // set mass properties
        cVector3d inertia(0.005, 0.005, 0.005);
        cVector3d poscofm(0.0, 0.0, 0.0);
        linkObject2->setMassProperties(0.1,        // mass
            inertia,    // inertia properties
            poscofm);   // position of center of mass

        // create 3 prismatic joints (x,y,z)
        jointBodyX2 = linkObject2->newJoint(DYN_JOINT_PRISMATIC, DYN_AXIS_X);
        jointBodyY2 = linkObject2->newJoint(DYN_JOINT_PRISMATIC, DYN_AXIS_Y);
        jointBodyZ2 = linkObject2->newJoint(DYN_JOINT_PRISMATIC, DYN_AXIS_Z);

        // create 1 spherical joint
        jointBodyS2 = linkObject2->newJoint(DYN_JOINT_SPHERICAL);

        // set some damping
        jointBodyX2->setDamping(5);
        jointBodyY2->setDamping(5);
        jointBodyZ2->setDamping(5);
        jointBodyS2->setDamping(0.1);

        // create a multimesh structure to describe the shape of our object
        cMultiMesh* objectModel = new cMultiMesh();

        // create new mesh inside the multimesh structure
        cMesh* meshObject = objectModel->newMesh();

        // create a block
        //cCreateBox(meshObject, sizeX, sizeY, sizeZ);
        cCreateCylinder(meshObject, 0.2, 0.038, 8, 1, true, true, cVector3d(0.0, 0.0, -0.1));
        meshObject->m_material->setWhiteAliceBlue();
        //meshObject->setShowFrame(true);

        // assign the object model as the collision model
        linkObject2->setCollisionModel(objectModel);

        // assign the object model as the graphical model
        linkObject2->setImageModel(objectModel);

        // create a convex hull to handle collision detection
        linkObject2->buildCollisionHull(radius, error);

        // connect link to basenode
        object->linkChild(linkObject2, pos, rot);

        // create collision detector for interaction with graphical object
        meshObject->createAABBCollisionDetector(1.01 * toolRadius);

        // set haptic properties
        meshObject->m_material->setStiffness(0.3 * maxStiffness);
        meshObject->m_material->setStaticFriction(0.8);
        meshObject->m_material->setDynamicFriction(0.6);
    }

    // create a scope to plot haptic device position data
    scope = new cScope();
    camera->m_frontLayer->addChild(scope);
    scope->setSize(600, 180);
    scope->setLocalPos(100,60);
    scope->setRange(-1.0, 5.0);
    cColorf color;
    color.setBlack();
    scope->setColor(color);
    scope->setSignalEnabled(true, true, false, false);
    scope->setTransparencyLevel(0.4);
    scope->m_colorSignal1.setRed();
    scope->m_colorSignal0.setYellowGold();


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

    // update position of scope
    scope->setSize(windowW - 200, 180);
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
    // update scope
    scope->setSignalValues(force1, force2, 0.0, 0.0);

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
    // proxy interaction
    cDynPositionForceProperty* lastForcePropertyProxy1 = NULL;
    cDynObject* lastDynObjectProxy1 = NULL;
    cTransform tool_T_object1;
    cDynamicLink* toolLink1 = NULL;
    cDynPositionForceProperty* lastForcePropertyProxy2 = NULL;
    cDynObject* lastDynObjectProxy2 = NULL;
    cTransform tool_T_object2;
    cDynamicLink* toolLink2 = NULL;

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
        double nextSimInterval = 0.002;//cClamp(time, 0.00001, 0.001);

        // reset clock
        simClock.reset();
        simClock.start();

        /////////////////////////////////////////////////////////////////////
        // READ HAPTIC DEVICE
        /////////////////////////////////////////////////////////////////////

        cVector3d position1(0,0,0);
        cVector3d position2(0,0,0);
        cVector3d linearVelocity1(0,0,0);
        cVector3d linearVelocity2(0,0,0);
        bool button1 = false;
        bool button2 = false;

        if (numHapticDevices > 0)
        {
            tool1->updatePose();
            tool1->computeInteractionForces();
            tool1->m_hapticPoint->m_sphereProxy->setLocalRot(tool1->getDeviceGlobalRot());
            if (hapticMode1 == C_HAPTIC_CONTROL)
            {
                tool1->m_lastComputedGlobalForce.zero();
            }
        }
        if (numHapticDevices > 1)
        {
            tool2->updatePose();
            tool2->computeInteractionForces();
            tool2->m_hapticPoint->m_sphereProxy->setLocalRot(tool2->getDeviceGlobalRot());
            if (hapticMode2 == C_HAPTIC_CONTROL)
            {
                tool2->m_lastComputedGlobalForce.zero();
            }
        }


        /////////////////////////////////////////////////////////////////////////
        // MOUSE INTERACTION FORCE
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
            cVector3d force = kp * (mouseSphere1->getLocalPos() - mouseSphere0->getLocalPos());
            cDynVector3 dynLocalPos(selectLocalPos.x(), selectLocalPos.y(), selectLocalPos.z());
            cDynVector3 dynForce(force.x(), force.y(), force.z());

            lastForceProperty = new cDynPositionForceProperty(dynLocalPos, dynForce);
            lastForceProperty->global(true);
            lastForceProperty->state(true);
            lastDynObject = selectLink->m_dynObject;
            lastDynObject->force.add(lastForceProperty);
        }

        /////////////////////////////////////////////////////////////////////////
        // HAPTIC INTERACTION FORCE
        /////////////////////////////////////////////////////////////////////////

        double Kp = 80;
        double Kr = 1.0;

        // erase previous forces
        if ((lastForcePropertyProxy1 != NULL) && (lastDynObjectProxy1 != NULL))
        {
            lastDynObjectProxy1->force.remove(lastForcePropertyProxy1);
            lastDynObjectProxy1 = NULL;
            delete lastForcePropertyProxy1;
            lastForcePropertyProxy1 = NULL;
        }
        if ((lastForcePropertyProxy2 != NULL) && (lastDynObjectProxy2 != NULL))
        {
            lastDynObjectProxy2->force.remove(lastForcePropertyProxy2);
            lastDynObjectProxy2 = NULL;
            delete lastForcePropertyProxy2;
            lastForcePropertyProxy2 = NULL;
        }

        // detect new proxy forces
        if (numHapticDevices > 0)
        {
            cDynamicLink* link1 = NULL;

            /////////////////////////////////////////////////////////////////////////////////
            // PROXY MODE
            /////////////////////////////////////////////////////////////////////////////////
            if (hapticMode1 == C_HAPTIC_PROXY)
            {
                // check if at least one contact has occurred
                if (tool1->m_hapticPoint->getNumCollisionEvents() > 0)
                {
                    // get contact event
                    cCollisionEvent* collisionEvent = tool1->m_hapticPoint->getCollisionEvent(0);

                    // get object from contact event
                    cGenericObject* object = collisionEvent->m_object;

                    cDynamicLink* link = dynamic_cast<cDynamicLink*> (object->getOwner()->getOwner());
                    link1 = link;
                    if (link != NULL)
                    {
                        // get position from contact event
                        cVector3d pos = collisionEvent->m_localPos;

                        // get force
                        cVector3d force = -1.0 * tool1->m_lastComputedGlobalForce;

                        // apply reaction force
                        cDynVector3 dynLocalPos(pos.x(), pos.y(), pos.z());
                        cDynVector3 dynForce(force.x(), force.y(), force.z());

                        lastForcePropertyProxy1 = new cDynPositionForceProperty(dynLocalPos, dynForce);
                        lastForcePropertyProxy1->global(true);
                        lastForcePropertyProxy1->state(true);
                        lastDynObjectProxy1 = link->m_dynObject;
                        lastDynObjectProxy1->force.add(lastForcePropertyProxy1);

                        // check button
                        bool button = tool1->getUserSwitch(0);
                        if (button)
                        {
                            hapticMode1 = C_HAPTIC_CONTROL;
                            tool1->m_lastComputedGlobalForce.zero();
                            tool1->m_hapticPoint->m_sphereProxy->setShowFrame(true);
                            tool1->m_hapticPoint->m_sphereProxy->setFrameSize(0.1, 0.1);

                            // compute transformation from world to tool (haptic device)
                            cTransform world_T_tool1 = tool1->getDeviceGlobalTransform();

                            // get transformation from object
                            cTransform world_T_object = object->getGlobalTransform();

                            // compute inverse transformation from contact point to object 
                            cTransform tool1_T_world = world_T_tool1;
                            tool1_T_world.invert();

                            // store current transformation tool
                            tool_T_object1 = tool1_T_world * world_T_object;

                            // store link
                            toolLink1 = link;
                        }
                    }
                }
            }

            /////////////////////////////////////////////////////////////////////////////////
            // CONTROL MODE
            /////////////////////////////////////////////////////////////////////////////////
            if (hapticMode1 == C_HAPTIC_CONTROL)
            {
                // check button
                bool button = tool1->getUserSwitch(0);
                if (!button)
                {
                    tool1->setForcesOFF();
                    tool1->setForcesON();
                    hapticMode1 = C_HAPTIC_PROXY;

                    if (toolLink1 == linkObject1)
                    {
                        jointBodyX1->setForce(0.0);
                        jointBodyY1->setForce(0.0);
                        jointBodyZ1->setForce(0.0);
                        jointBodyS1->setTorque(cVector3d(0.0, 0.0, 0.0));
                    }
                    else if (toolLink1 == linkObject2)
                    {
                        jointBodyX2->setForce(0.0);
                        jointBodyY2->setForce(0.0);
                        jointBodyZ2->setForce(0.0);
                        jointBodyS2->setTorque(cVector3d(0.0, 0.0, 0.0));
                    }

                    tool1->m_hapticPoint->m_sphereProxy->setShowFrame(false);
                }
                else
                {
                    // compute desired configuration for object
                    cTransform world_T_tool = tool1->getDeviceGlobalTransform();
                    cTransform world_T_object1 = world_T_tool * tool_T_object1;

                    if (toolLink1 == linkObject1)
                    {
                        cVector3d pos = toolLink1->getLocalPos();
                        cMatrix3d rot = toolLink1->getLocalRot();
                        cVector3d pos_des = world_T_object1.getLocalPos();
                        cMatrix3d rot_des = world_T_object1.getLocalRot();

                        // compute linear force
                        cVector3d force = Kp * (pos_des - pos);

                        // compute angular torque
                        cVector3d axis;
                        double angle;
                        cMatrix3d deltaRotation = cTranspose(rot) * rot_des;
                        deltaRotation.toAxisAngle(axis, angle);
                        cVector3d torque = ((Kr * angle) * axis);

                        // apply forces
                        jointBodyX1->setForce(force.x());
                        jointBodyY1->setForce(force.y());
                        jointBodyZ1->setForce(force.z());
                        jointBodyS1->setTorque(torque);
                        jointBodyX1->setDamping(20);
                        jointBodyY1->setDamping(20);
                        jointBodyZ1->setDamping(20);
                        jointBodyS1->setDamping(10);

                        // apply forces to haptic device
                        tool1->m_lastComputedGlobalForce = -1.0 * force;
                        tool1->m_lastComputedGlobalTorque = -1.0 * torque;
                    }
                    else if (toolLink1 == linkObject2)
                    {
                        cVector3d pos = toolLink1->getLocalPos();
                        cMatrix3d rot = toolLink1->getLocalRot();
                        cVector3d pos_des = world_T_object1.getLocalPos();
                        cMatrix3d rot_des = world_T_object1.getLocalRot();

                        // compute linear force
                        cVector3d force = Kp * (pos_des - pos);

                        // compute angular torque
                        cVector3d axis;
                        double angle;
                        cMatrix3d deltaRotation = cTranspose(rot) * rot_des;
                        deltaRotation.toAxisAngle(axis, angle);
                        cVector3d torque = ((Kr * angle) * axis);

                        // apply forces
                        jointBodyX2->setForce(force.x());
                        jointBodyY2->setForce(force.y());
                        jointBodyZ2->setForce(force.z());
                        jointBodyS2->setTorque(torque);
                        jointBodyX2->setDamping(20);
                        jointBodyY2->setDamping(20);
                        jointBodyZ2->setDamping(20);
                        jointBodyS2->setDamping(10);

                        // apply forces to haptic device
                        tool1->m_lastComputedGlobalForce = -1.0 * force;
                        tool1->m_lastComputedGlobalTorque = -1.0 * torque;
                    }
                }
            }
        }
        if (numHapticDevices > 1)
        {
            cDynamicLink* link2 = NULL;

            /////////////////////////////////////////////////////////////////////////////////
            // PROXY MODE
            /////////////////////////////////////////////////////////////////////////////////
            if (hapticMode2 == C_HAPTIC_PROXY)
            {
                // check if at least one contact has occurred
                if (tool2->m_hapticPoint->getNumCollisionEvents() > 0)
                {
                    // get contact event
                    cCollisionEvent* collisionEvent = tool2->m_hapticPoint->getCollisionEvent(0);

                    // get object from contact event
                    cGenericObject* object = collisionEvent->m_object;

                    cDynamicLink* link = dynamic_cast<cDynamicLink*> (object->getOwner()->getOwner());
                    link2 = link;
                    if (link != NULL)
                    {
                        // get position from contact event
                        cVector3d pos = collisionEvent->m_localPos;

                        // get force
                        cVector3d force = -1.0 * tool2->m_lastComputedGlobalForce;

                        // apply reaction force
                        cDynVector3 dynLocalPos(pos.x(), pos.y(), pos.z());
                        cDynVector3 dynForce(force.x(), force.y(), force.z());

                        lastForcePropertyProxy2 = new cDynPositionForceProperty(dynLocalPos, dynForce);
                        lastForcePropertyProxy2->global(true);
                        lastForcePropertyProxy2->state(true);
                        lastDynObjectProxy2 = link->m_dynObject;
                        lastDynObjectProxy2->force.add(lastForcePropertyProxy2);

                        // check button
                        bool button = tool2->getUserSwitch(0);
                        if (button)
                        {
                            hapticMode2 = C_HAPTIC_CONTROL;
                            tool2->m_lastComputedGlobalForce.zero();
                            tool2->m_hapticPoint->m_sphereProxy->setShowFrame(true);
                            tool2->m_hapticPoint->m_sphereProxy->setFrameSize(0.1, 0.1);

                            // compute transformation from world to tool (haptic device)
                            cTransform world_T_tool2 = tool2->getDeviceGlobalTransform();

                            // get transformation from object
                            cTransform world_T_object = object->getGlobalTransform();

                            // compute inverse transformation from contact point to object 
                            cTransform tool2_T_world = world_T_tool2;
                            tool2_T_world.invert();

                            // store current transformation tool
                            tool_T_object2 = tool2_T_world * world_T_object;

                            // store link
                            toolLink2 = link;
                        }
                    }
                }
            }

            /////////////////////////////////////////////////////////////////////////////////
            // CONTROL MODE
            /////////////////////////////////////////////////////////////////////////////////
            if (hapticMode2 == C_HAPTIC_CONTROL)
            {
                // check button
                bool button = tool2->getUserSwitch(0);
                if (!button)
                {
                    tool2->setForcesOFF();
                    tool2->setForcesON();
                    hapticMode2 = C_HAPTIC_PROXY;

                    if (toolLink2 == linkObject1)
                    {
                        jointBodyX1->setForce(0.0);
                        jointBodyY1->setForce(0.0);
                        jointBodyZ1->setForce(0.0);
                        jointBodyS1->setTorque(cVector3d(0.0, 0.0, 0.0));
                    }
                    else if (toolLink2 == linkObject2)
                    {
                        jointBodyX2->setForce(0.0);
                        jointBodyY2->setForce(0.0);
                        jointBodyZ2->setForce(0.0);
                        jointBodyS2->setTorque(cVector3d(0.0, 0.0, 0.0));
                    }

                    tool2->m_hapticPoint->m_sphereProxy->setShowFrame(false);
                }
                else
                {
                    // compute desired configuration for object
                    cTransform world_T_tool = tool2->getDeviceGlobalTransform();
                    cTransform world_T_object2 = world_T_tool * tool_T_object2;

                    if (toolLink2 == linkObject1)
                    {
                        cVector3d pos = toolLink2->getLocalPos();
                        cMatrix3d rot = toolLink2->getLocalRot();
                        cVector3d pos_des = world_T_object2.getLocalPos();
                        cMatrix3d rot_des = world_T_object2.getLocalRot();

                        // compute linear force
                        cVector3d force = Kp * (pos_des - pos);

                        // compute angular torque
                        cVector3d axis;
                        double angle;
                        cMatrix3d deltaRotation = cTranspose(rot) * rot_des;
                        deltaRotation.toAxisAngle(axis, angle);
                        cVector3d torque = ((Kr * angle) * axis);

                        // apply forces
                        jointBodyX1->setForce(force.x());
                        jointBodyY1->setForce(force.y());
                        jointBodyZ1->setForce(force.z());
                        jointBodyS1->setTorque(torque);
                        jointBodyX1->setDamping(20);
                        jointBodyY1->setDamping(20);
                        jointBodyZ1->setDamping(20);
                        jointBodyS1->setDamping(10);

                        // apply forces to haptic device
                        tool2->m_lastComputedGlobalForce = -1.0 * force;
                        tool2->m_lastComputedGlobalTorque = -1.0 * torque;
                    }
                    else if (toolLink2 == linkObject2)
                    {
                        cVector3d pos = toolLink2->getLocalPos();
                        cMatrix3d rot = toolLink2->getLocalRot();
                        cVector3d pos_des = world_T_object2.getLocalPos();
                        cMatrix3d rot_des = world_T_object2.getLocalRot();

                        // compute linear force
                        cVector3d force = Kp * (pos_des - pos);

                        // compute angular torque
                        cVector3d axis;
                        double angle;
                        cMatrix3d deltaRotation = cTranspose(rot) * rot_des;
                        deltaRotation.toAxisAngle(axis, angle);
                        cVector3d torque = ((Kr * angle) * axis);

                        // apply forces
                        jointBodyX2->setForce(force.x());
                        jointBodyY2->setForce(force.y());
                        jointBodyZ2->setForce(force.z());
                        jointBodyS2->setTorque(torque);
                        jointBodyX2->setDamping(20);
                        jointBodyY2->setDamping(20);
                        jointBodyZ2->setDamping(20);
                        jointBodyS2->setDamping(10);

                        // apply forces to haptic device
                        tool2->m_lastComputedGlobalForce = -1.0 * force;
                        tool2->m_lastComputedGlobalTorque = -1.0 * torque;
                    }
                }
            }
        }


        /////////////////////////////////////////////////////////////////////////
        // UPDATE DYNAMICS
        /////////////////////////////////////////////////////////////////////////

        // update simulation
        dynamicWorld->updateDynamics(nextSimInterval);


        /////////////////////////////////////////////////////////////////////////
        // SEND HAPTIC FORCES
        /////////////////////////////////////////////////////////////////////////


        if (numHapticDevices > 0)
        {
            tool1->applyForces();
            force1 = tool1->m_lastComputedGlobalForce.length();
        }
        if (numHapticDevices > 1)
        {
            tool2->applyForces();
            force2 = tool2->m_lastComputedGlobalForce.length();
        }
    }

    // close tools
    if (numHapticDevices > 0)
    {
        tool1->stop();
    }
    if (numHapticDevices > 1)
    {
        tool2->stop();
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
                selectLocalPos = collisionRecorder.m_nearestCollision.m_localPos;   
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
