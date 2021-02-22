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
#include "Primitives.h"
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
// APPLICATION STATE
//------------------------------------------------------------------------------

const int STATE_IDLE            = 1;
const int STATE_MODIFY_MAP      = 2;
const int STATE_MOVE_CAMERA     = 3;


//---------------------------------------------------------------------------
// APPLICATION
//---------------------------------------------------------------------------

// camera position and orientation is spherical coordinates
double cameraAngleH;
double cameraAngleV;
double cameraDistance;
cVector3d cameraPosition;

// state machine 
int state = STATE_IDLE;

// mouse position and button status
int mouseX; 
int mouseY;
int mouseButton;

// camera status
bool flagCameraInMotion = false;

// status of the main simulation  loop
bool simulationRunning = false;

// has exited simulation thread
bool simulationFinished = true;

// status of the main haptics loop
bool hapticsRunning = false;

// has exited haptics thread
bool hapticsFinished = true;

// information about computer screen and GLUT display window
int screenW;
int screenH;
int windowW;
int windowH;
int windowPosX;
int windowPosY;


//---------------------------------------------------------------------------
// CHAI3D
//---------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera that renders the world in a window display
cCamera* camera;

// a directional light source to illuminate the scene
cDirectionalLight *light;

// a spot light to illuminate the scene
cSpotLight *spot;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
shared_ptr<cGenericHapticDevice> hapticDevice;

// information about haptic device
cHapticDeviceInfo info;

// a level widget to display velocity
cLevel* distanceLevels[ROVER_NUM_SENSORS];

// three dials to display position
cDial* directionDial;


//---------------------------------------------------------------------------
// DYNAMICS3D
//---------------------------------------------------------------------------

// dynamic world
cDynamicWorld* dynamicWorld;

// dynamic objects
cDynamicBase* ground;

// default material
cDynamicMaterial* dynMaterial;

// rover
cRover* rover;


//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// callback when the window display is resized
void resizeWindow(int w, int h);

// update camera position
void updateCameraPosition();

// callback when a keyboard key is pressed
void keySelect(unsigned char key, int x, int y);

// callback when special keys are pressed
void specialKeySelect(int key, int x, int y);

// callback when the right mouse button is pressed to select a menu item
void menuSelect(int value);

// callback to handle mouse click
void mouseClick(int button, int state, int x, int y);

// callback to handle mouse motion
void mouseMove(int x, int y);

// function called before exiting the application
void close(void);

// main graphics callback
void updateGraphics(void);

// main haptics loop
void updateHaptics(void);

// main simulation loop
void updateSimulation(void);



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
    cout << "Demo: 05-rover" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[UP]    - Forward" << endl;
    cout << "[DOWN]  - Backward" << endl;;
    cout << "[LEFT]  - Turn Left" << endl;
    cout << "[DOWN]  - Turn Right" << endl;
    cout << "[SPACE] - Engage Brake" << endl;
    cout << "[x]     - Exit application" << endl << endl;


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
    glutSpecialFunc(specialKeySelect);
    glutMouseFunc(mouseClick);
    glutMotionFunc(mouseMove);
    glutReshapeFunc(resizeWindow);
    glutSetWindowTitle("CHAI3D");

    // set fullscreen mode
    if (fullscreen)
    {
        glutFullScreen();
    }


    //-----------------------------------------------------------------------
    // 3D - SCENEGRAPH
    //-----------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    // the color is defined by its (R,G,B) components.
    world->setBackgroundColor(1.0, 1.0, 1.0);

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // define a default position of the camera (described in spherical coordinates)
    cameraAngleH = 5;
    cameraAngleV = 50;
    cameraDistance = 10.0;
    updateCameraPosition();

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.1, 100.0);
    // camera->setUseShadowCasting(true);
    camera->setUseMultipassTransparency(false);
    camera->setStereoFocalLength(1.0);
    camera->setStereoEyeSeparation(0.01);

    // create a light source and attach it to the camera
    spot = new cSpotLight(world);
    world->addChild(spot);                         // attach light to camera
    spot->setEnabled(true);                        // enable light source
    spot->setLocalPos(cVector3d( 5, 5, 5));        // position the light source
    spot->setDir(cVector3d(-1.0, -1.0, -1.5));     // define the direction of the light beam
    spot->m_ambient.set(0.3, 0.3, 0.3);
    spot->m_diffuse.set(0.4, 0.4, 0.4);
    spot->m_specular.set(0.4, 0.4, 0.4);
    // spot->m_shadowMap->setResolutionCustom(1024, 1024);
    spot->setShadowMapProperties(2.0, 10);
    spot->setCutOffAngleDeg(40);
    spot->setDisplaySettings(0.05, 4.0, false);

    // Enable/Disable shadows for performance
    spot->m_shadowMap->setEnabled(true);

    light = new cDirectionalLight(world);
    world->addChild(light);                         // attach light to camera
    light->setEnabled(true);                        // enable light source
    light->setDir(cVector3d(-1.0, -1.0, -1.0));     // define the direction of the light beam
    light->m_ambient.set(0.3, 0.3, 0.3);
    light->m_diffuse.set(0.4, 0.4, 0.4);
    light->m_specular.set(0.4, 0.4, 0.4);


    //-----------------------------------------------------------------------
    // WIDGETS
    //-----------------------------------------------------------------------

    // a level widget to display velocity
    for (int i=0; i<ROVER_NUM_SENSORS; i++)
    {
        cLevel* level = new cLevel();
        distanceLevels[i] = level;
        camera->m_frontLayer->addChild(level);
        level->setRange(0.0, 1.0);
        level->setWidth(50);
        level->setNumIncrements(50);
        level->setSingleIncrementDisplay(false);
        level->setTransparencyLevel(0.5);
        level->m_colorActive.setRed();
    }

    // setup center direction dial
    directionDial = new cDial();
    camera->m_frontLayer->addChild(directionDial);
    directionDial->setRange(-45, 45);
    directionDial->setSize(120);
    directionDial->setSingleIncrementDisplay(true);
    directionDial->m_colorActive.setRed();

    //-----------------------------------------------------------------------
    // COMPOSE DYNWORLD
    //-----------------------------------------------------------------------

    // create an NEWTON world to simulate dynamic bodies
    dynamicWorld = new cDynamicWorld(world);

    // add dynamic world as a node inside world
    world->addChild(dynamicWorld);

    // set some gravity
    dynamicWorld->setGravity(0.0, 0.0, -9.81);
  
    // dynamic settings
    double error = 0.001;
    double radius = 0.001;
    double damping = 10;

    // default material
    cDynamicMaterial* dynMaterial = new cDynamicMaterial();
    dynMaterial->setDynamicFriction(1.0);
    dynMaterial->setStaticFriction(1.0);


    //-----------------------------------------------------------------------
    // CREATE GROUND
    //-----------------------------------------------------------------------
    double sizeGround = 30.0;
    if (true)
    {
        // create base node
        ground = dynamicWorld->newBaseObject(cVector3d(0,0,0), cIdentity3d());
        cDynamicLink* link = ground->newLink(dynMaterial);
   
        cMultiMesh* collisionModel = new cMultiMesh();
        cMesh*  mesh;

        // assign collision model to ground
        link->setCollisionModel(collisionModel);

        // create large plane
        mesh = collisionModel->newMesh();
        cCreatePlane(mesh, sizeGround, sizeGround);
        mesh->m_material->setBlueCornflower();
      
        // create a graphic representation
        cMultiMesh* graphicModel = new cMultiMesh();
        mesh = graphicModel->newMesh();
        cCreateMap(mesh, sizeGround, sizeGround, 1, 1);
        mesh->m_material->setBlueCornflower();
        mesh->setUseDisplayList(true, true);
        link->setImageModel(graphicModel);

        // create collision hull
        link->buildCollisionHull(radius, error);
        //link->buildCollisionTriangles(radius, error);

        // link ground to basenode
        ground->linkChild(link, cVector3d(0,0,0), cIdentity3d());
    }


    //-----------------------------------------------------------------------
    // CREATE BLOCKS
    //-----------------------------------------------------------------------
    if (true)
    {
        // create block
        cBlock* block;
        cMatrix3d rot;

        block = new cBlock(dynamicWorld, 4.0, 0.3, 0.3, cVector3d(0.0, -0.8, 0.4));
        block = new cBlock(dynamicWorld, 4.0, 0.3, 0.3, cVector3d(0.0, 0.8, 0.4));

        rot.identity();
        rot.rotateAboutGlobalAxisDeg(0,0,1, 35);
        block = new cBlock(dynamicWorld, 1.5, 0.3, 0.3, cVector3d(-2.7,-1.3, 0.4), rot);


        rot.identity();
        rot.rotateAboutGlobalAxisDeg(0,0,1,-35);
        block = new cBlock(dynamicWorld, 1.5, 0.3, 0.3, cVector3d(-2.7, 1.3, 0.4), rot);
    }

    //-----------------------------------------------------------------------
    // CREATE CAR
    //-----------------------------------------------------------------------
    
    // build car
    double SIZE_ROVER = 1.0;
    cVector3d pos(2,2,1.6);
    cMatrix3d rot;
    rot.identity();
    rot.rotateAboutLocalAxisDeg(0,0,1, 45);   
    rover = new cRover(dynamicWorld, SIZE_ROVER, pos, rot);
    
    if (true)
    {
        bool result;

        // set graphic car body
        cMultiMesh* multiMesh;
        multiMesh = new cMultiMesh();
        result = multiMesh->loadFromFile("./resources/models/Honda/honda-body.3ds");
        if (!result)
        {
            multiMesh->loadFromFile("../../../bin/resources/models/Honda/honda-body.3ds");
        }

        multiMesh->scale(0.0065);
        multiMesh->setLocalPos(-0.02, 0.0, -0.05);
        multiMesh->setUseDisplayList(true, true);
        multiMesh->setUseCulling(false, true);
        multiMesh->addChild(rover->m_sensorNose);
        rover->m_dynamicBody->setImageModel(multiMesh);
    
        // set graphic car wheels
        cMultiMesh* wheel;
        wheel = new cMultiMesh();
        result = wheel->loadFromFile("./resources/models/Honda/honda-wheel.3ds");
        if (!result)
        {
            wheel->loadFromFile("../../../bin/resources/models/Honda/honda-wheel.3ds");
        }

        wheel->scale(0.0065);
        wheel->setUseDisplayList(true, true);
        wheel->setUseCulling(false, true);

        cMultiMesh* wheelFL = wheel->copy();
        wheelFL->rotateAboutLocalAxisDeg(0,0,1,180);
        wheelFL->setUseCulling(false, true);
        wheelFL->setUseDisplayList(true, true);
        wheelFL->deleteCollisionDetector(true);

        cMultiMesh* wheelFR = wheel->copy();
        wheelFR->setUseCulling(false, true);
        wheelFR->setUseDisplayList(true, true);
        wheelFR->deleteCollisionDetector(true);

        cMultiMesh* wheelBL = wheel->copy();
        wheelBL->rotateAboutLocalAxisDeg(0,0,1,180);
        wheelBL->setUseCulling(false, true);
        wheelBL->setUseDisplayList(true, true);
        wheelBL->deleteCollisionDetector(true);

        cMultiMesh* wheelBR = wheel->copy();
        wheelBR->setUseCulling(false, true);
        wheelBR->setUseDisplayList(true, true);
        wheelBR->deleteCollisionDetector(true);

        rover->m_dynamicFL->setImageModel(wheelFL);
        rover->m_dynamicFR->setImageModel(wheelFR);
        rover->m_dynamicBL->setImageModel(wheelBL);
        rover->m_dynamicBR->setImageModel(wheelBR);
    }
    
    //--------------------------------------------------------------------------
    // HAPTIC DEVICE
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get a handle to the first haptic device
    handler->getDevice(hapticDevice, 0);

    // open a connection to haptic device
    hapticDevice->open();

    // calibrate device (if necessary)
    hapticDevice->calibrate();

    // retrieve information about the current haptic device
    info = hapticDevice->getSpecifications();

    // if the device has a gripper, enable the gripper to simulate a user switch
    hapticDevice->setEnableGripperUserSwitch(true);


    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------

    // simulation in now running
    simulationRunning = true;

    // create a thread which starts the main haptics rendering loop
    cThread* simulationThread = new cThread();
    simulationThread->start(updateSimulation, CTHREAD_PRIORITY_GRAPHICS);


    //--------------------------------------------------------------------------
    // START HAPTICS
    //--------------------------------------------------------------------------

    if (handler->getNumDevices() > 0)
    {
        // simulation in now running
        hapticsRunning = true;

        // create a thread which starts the main haptics rendering loop
        cThread* hapticsThread = new cThread();
        hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);
    }

    // start the main graphics rendering loop
    glutMainLoop();

    // close everything
    close();

    // exit
    return (0);
}

//---------------------------------------------------------------------------

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

//---------------------------------------------------------------------------

void keySelect(unsigned char key, int x, int y)
{
    // escape key
    if ((key == 27) || (key == 'x'))
    {
        // close everything
        close();

        // exit application
        exit(0);
    }

    // option SPACE:
    if (key == ' ')
    {
        // engage brake at maximum level
        rover->setBrakingLevel(1.0);
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

void specialKeySelect(int key, int x, int y)
{
    // option FORWARD:
    if (key == GLUT_KEY_UP)
    {
        double val = rover->getVelocity();
        rover->setVelocity(val + 0.05);
        rover->setBrakingLevel(0.0);
    }

    // option BACKWARD:
    if (key == GLUT_KEY_DOWN)
    {
        double val = rover->getVelocity();
        rover->setVelocity(val - 0.05);
        rover->setBrakingLevel(0.0);
    }

    // option LEFT:
    if (key == GLUT_KEY_LEFT)
    {
        double val = rover->getDirectionDeg();
        rover->setDirectionDeg(val + 5.0);
    }

    // option RIGHT:
    if (key == GLUT_KEY_RIGHT)
    {
        double val = rover->getDirectionDeg();
        rover->setDirectionDeg(val - 5.0);
    }
}

//------------------------------------------------------------------------------

void mouseClick(int button, int state, int x, int y)
{
    // mouse button down
    if (state == GLUT_DOWN)
    {
        flagCameraInMotion = true;
        mouseX = x;
        mouseY = y;
        mouseButton = button;
    }

    // mouse button up
    else if (state == GLUT_UP)
    {
        flagCameraInMotion = false;
    }
}

//------------------------------------------------------------------------------

void mouseMove(int x, int y)
{
    if (flagCameraInMotion)
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
    }

    updateCameraPosition();

    mouseX = x;
    mouseY = y;
}

//---------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;
    hapticsRunning = false;

    // wait for graphics and haptics loops to terminate
    while ((!simulationFinished) || (!hapticsFinished)) { cSleepMs(100); }
}

//---------------------------------------------------------------------------

void updateGraphics(void)
{
    // update dial position
    directionDial->setLocalPos(0.5 * windowW, 120);
    directionDial->setValue(-rover->getDirectionDeg());

    // update level positions and values
    double HEIGHT = 20;
    double DELTA = 80;
    double OFFSET = -50;
    distanceLevels[0]->setLocalPos(0.5 * windowW - 7.0 * DELTA, HEIGHT);
    distanceLevels[1]->setLocalPos(0.5 * windowW - 6.0 * DELTA, HEIGHT);
    distanceLevels[2]->setLocalPos(0.5 * windowW - 5.0 * DELTA, HEIGHT);
    distanceLevels[3]->setLocalPos(0.5 * windowW - 4.0 * DELTA, HEIGHT);
    distanceLevels[4]->setLocalPos(0.5 * windowW - 3.0 * DELTA, HEIGHT);
    distanceLevels[5]->setLocalPos(0.5 * windowW - 2.0 * DELTA, HEIGHT);

    distanceLevels[6]->setLocalPos(0.5 * windowW + 2.0 * DELTA + OFFSET, HEIGHT);
    distanceLevels[7]->setLocalPos(0.5 * windowW + 3.0 * DELTA + OFFSET, HEIGHT);
    distanceLevels[8]->setLocalPos(0.5 * windowW + 4.0 * DELTA + OFFSET, HEIGHT);
    distanceLevels[9]->setLocalPos(0.5 * windowW + 5.0 * DELTA + OFFSET, HEIGHT);
    distanceLevels[10]->setLocalPos(0.5 * windowW + 6.0 * DELTA + OFFSET, HEIGHT);
    distanceLevels[11]->setLocalPos(0.5 * windowW + 7.0 * DELTA + OFFSET, HEIGHT);

    for(int i=0; i<ROVER_NUM_SENSORS; i++)
    {
        distanceLevels[i]->setValue(rover->m_sensorValues[i]);
    }

    // render world
    camera->renderView(windowW, windowH);

    // Swap buffers
    glutSwapBuffers();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));

    // inform the GLUT window to call updateGraphics again (next frame)
    if (simulationRunning)
    {
        glutPostRedisplay();
    }
}

//---------------------------------------------------------------------------

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
        double nextSimInterval = cClamp(time, 0.00001, 0.01);

        // reset clock
        simClock.reset();
        simClock.start();

        // update simulation
        dynamicWorld->updateDynamics(nextSimInterval);

        // update controller
        rover->updateController();

        // update sensor data
        rover->updateSensorData(world);
    }

    // exit haptics thread
    simulationFinished = true;
}

//---------------------------------------------------------------------------

void updateHaptics(void)
{
    // simulation clock
    cPrecisionClock simClock;
    simClock.start(true);
    hapticsFinished = false;

    cVector3d stickPosition;

    // main haptic simulation loop
    while(hapticsRunning)
    {
        /////////////////////////////////////////////////////////////////////
        // READ HAPTIC DEVICE
        /////////////////////////////////////////////////////////////////////

        // read position 
        cVector3d position;
        hapticDevice->getPosition(position);

        // read linear velocity 
        cVector3d linearVelocity;
        hapticDevice->getLinearVelocity(linearVelocity);

        // read user-switch status (button 0)
        bool button0 = false;
        hapticDevice->getUserSwitch(0, button0);


        /////////////////////////////////////////////////////////////////////
        // COMPUTE AND APPLY FORCES
        /////////////////////////////////////////////////////////////////////

        // desired position
        cVector3d desiredPosition;
        desiredPosition.set(0.0, 0.0, 0.0);

        // desired orientation
        cMatrix3d desiredRotation;
        desiredRotation.identity();

        // variables for forces    
        cVector3d force (0,0,0);
        cVector3d torque (0,0,0);
        double gripperForce = 0.0;
        double fx,fy,fz;
        fx = fy = fz = 0.0;

        // compute vertical contain force
        if (true)
        {
            double kp = 1000;
            double kv = 1.0 * info.m_maxLinearDamping;
            fz = (kp * (desiredPosition - position) - kv * linearVelocity).z();
        }

        // compute forward/backward force
        if (true)
        {
            cVector3d f = (desiredPosition - position);
            cVector3d force(0,0,0);

            for (int i=0; i<ROVER_NUM_SENSORS; i++)
            {
                cVector3d dir = cSub(rover->m_sensorPoints[i][1], rover->m_sensorPoints[i][0]);
                if (cAngle(dir, f) < cDegToRad(90))
                {
                    force = force + cMul((rover->m_sensorValues[i] * 300), cProject(f, dir));
                }
            }

            force = force + 50 * f;

            fx = force.x();
            fy = force.y();
        }

        // update force
        force.set(fx, fy, fz);

        // send computed force, torque, and gripper force to haptic device	
        hapticDevice->setForceAndTorqueAndGripperForce(force, torque, gripperForce);

        // temp variables for rover commands
        double roverVelocity, roverDirection, roverBrakingLevel;

        // braking command
        if (button0)
        {
            roverBrakingLevel = 0.6;
        }
        else
        {
            roverBrakingLevel = 0.0;
        }

        // velocity
        roverVelocity = -15 * position.x();

        // direction
        roverDirection = -(45.0/0.1) * position.y();

        // send commands to rover
        rover->setVelocity(roverVelocity);
        rover->setDirectionDeg(roverDirection);
        rover->setBrakingLevel(roverBrakingLevel);

        // update
        //directionDial->setValue(roverDirection);
    }

    // close haptic device
    hapticDevice->close();

    // exit haptics thread
    hapticsFinished = true;
}
