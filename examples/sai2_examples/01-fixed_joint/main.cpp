#include <iostream>
#include <string>
#include "Sai2Simulation.h"
#include "Sai2Model.h"
#include "Sai2Graphics.h"

#include "timer/LoopTimer.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

using namespace std;

const string world_fname = "resources/world1.urdf";
const string robot_fname = "resources/pbot1.urdf";
const string robot_name = "PBot1";
const string camera_name = "camera_fixed";

bool fSimulationRunning = false;

// sim
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);

// initialize window manager
GLFWwindow* glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

int main (int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_fname << endl;

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_fname, true);
	robot->updateModel();

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_fname, false);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_fname, false);
	graphics->_world->setBackgroundColor(0.3, 0.3, 0.3);

	cout << "Robot " << robot_name << " has " << robot->dof() << " joints in model." << endl;
	cout << "Robot " << robot_name << " has " << sim->dof(robot_name) << " joints in simulation." << endl;

	Eigen::VectorXd gravity_torques;
	robot->gravityVector(gravity_torques);
	cout << "gravity vector: " << gravity_torques.transpose() << endl;
	cout << "Mass matrix: " << robot->_M << endl;


	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

    // set callbacks
	glfwSetKeyCallback(window, keySelect);

	// start the simulation
	thread sim_thread(simulation, robot, sim);
	
	Eigen::Vector3d link1_pos;
	Eigen::VectorXd torques(1);
	const string link1_name = "link1";

    // while window is open:
    while (!glfwWindowShouldClose(window)) {
    	// apply joint torque
    	torques << 0.0;//2.0*9.81;
    	sim->setJointTorques(robot_name, torques);

		// update kinematic models
		sim->getJointPositions(robot_name, robot->_q);
		robot->updateModel();
		robot->position(link1_pos, link1_name, Eigen::Vector3d::Zero());
		// cout << "link 1 position" << link1_pos.transpose() << endl;

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->render(camera_name, width, height);
		glfwSwapBuffers(window);
		glFinish();

	    // poll for events
	    glfwPollEvents();
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}


//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim) {
	fSimulationRunning = true;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); //1kHz timer
	double last_time = timer.elapsedTime(); //secs

	bool fTimerDidSleep = true;
	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// integrate forward
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time;
		sim->integrate(loop_dt);

		// update last time
		last_time = curr_time;
	}
}


//------------------------------------------------------------------------------
GLFWwindow* glfwInitialize() {
		/*------- Set up visualization -------*/
    // set up error callback
    glfwSetErrorCallback(glfwError);

    // initialize GLFW
    glfwInit();

    // retrieve resolution of computer display and position window accordingly
    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary);

    // information about computer screen and GLUT display window
	int screenW = mode->width;
    int screenH = mode->height;
    int windowW = 0.8 * screenH;
    int windowH = 0.5 * screenH;
    int windowPosY = (screenH - windowH) / 2;
    int windowPosX = windowPosY;

    // create window and make it current
    glfwWindowHint(GLFW_VISIBLE, 0);
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "Force Jitter Test Sim", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	return window;
}


//------------------------------------------------------------------------------
void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    // option ESC: exit
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        // exit application
         glfwSetWindowShouldClose(window, 1);
    }
}

