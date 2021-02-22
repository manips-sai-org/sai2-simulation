#include <iostream>
#include <fstream>
#include <string>
#include <thread>

#include "Sai2Model.h"
#include "dynamics3d.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"

#include "timer/LoopTimer.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

using namespace std;

const string world_fname = "world.urdf";
const string robot_fname = "pbot.urdf";
const string robot_name = "PBot1";
const string camera_name = "camera_fixed";


// control torque
double vtorque = 0.0;
double rtorque = 0.0;
double contact_force = 0.0;
uint num_contacts = 0;
Eigen::VectorXd ddq(3);

fstream logfile;
// control kickup
bool fKickUp = false;
bool fKickLeft = false;
bool fKickRight = false;

// simulation loop
bool fSimulationRunning = false;
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);

bool f_global_sim_pause = false; // use with caution!
// bool f_global_sim_pause = true; // use with caution!

// initialize window manager
GLFWwindow* glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

int main (int argc, char ** argv) {
	cout << "Loading URDF world model file: " << world_fname << endl;

	// // load robots
	// auto robot = new Sai2Model::Sai2Model(robot_fname, false);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_fname, false);
	sim->setCollisionRestitution(0.0);
	sim->setCoeffFrictionStatic(0.05);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_fname, false);
	graphics->_world->setBackgroundColor(0.3, 0.3, 0.3);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_fname, false);

	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

    // set callbacks
	glfwSetKeyCallback(window, keySelect);

	// logging
	// fstream logfile;
	logfile.open(string("log.csv"), std::ios::out);
	logfile << "time, q0, q1, q2, dq0, dq1, dq2, ddq0, ddq1, ddq2, force, contacts" << endl;
	ddq.setZero();
	Eigen::VectorXd ddq_buff(3);
	ddq_buff.setZero();

	// start the simulation
	thread sim_thread(simulation, robot, sim);
	
    // while window is open:
    while (!glfwWindowShouldClose(window)) {
    	// apply joint torque
    	sim->setJointTorques(robot_name, Eigen::Vector3d(rtorque, vtorque, 0.0));
    	// sim->setJointTorque(robot_name, 0, rtorque);

    	// apply kick up
    	if (fKickUp) {
			sim->setJointVelocity(robot_name, 1, 3);
    		fKickUp = false;
    	}
   //  	if (fKickLeft) {
			// sim->setJointTorque(robot_name, 0, -1.0);
   //  		fKickLeft = false;
   //  	}
   //  	if (fKickRight) {
			// sim->setJointTorque(robot_name, 0, 1.0);
   //  		fKickRight = false;
   //  	}

    	// logging
    	ddq_buff = ddq;

		// update kinematic models
		sim->getJointPositions(robot_name, robot->_q);
		robot->updateModel();

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

	logfile.close();
	return 0;
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
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim) {
	fSimulationRunning = true;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); //1kHz timer
	double last_time = timer.elapsedTime(); //secs

	auto contact_list = sim->_world->getBaseNode("PBot1")->m_dynamicContacts;

	Eigen::Vector3d contact_force_vec;
	bool fTimerDidSleep = true;
	long counter = 0;
	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// integrate forward
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time;
		if (!f_global_sim_pause) {
			sim->integrate(loop_dt);
		}
		sim->getJointVelocities(robot_name, robot->_dq);
		sim->getJointAccelerations(robot_name, ddq);
		num_contacts = contact_list->getNumContacts();
		if(contact_list->getNumContacts() > 0) {
			contact_force_vec.setZero();
			for (uint i = 0; i < contact_list->getNumContacts(); i++) {
				contact_force_vec += contact_list->getContact(i)->m_globalNormalForce.eigen() +
				contact_list->getContact(i)->m_globalFrictionForce.eigen();
			}
			contact_force = contact_force_vec.norm();
		} else {
			contact_force = 0.0;
		}

		counter++;
		if (counter % 33 == 0) {
			logfile << sim->_world->m_time
				<< ", " << robot->_q[0] << ", " << robot->_q[1] << ", " << robot->_q[2]
				<< ", " << robot->_dq[0] << ", " << robot->_dq[1] << ", " << robot->_dq[2]
				<< ", " << ddq[0] << ", " << ddq[1] << ", " << ddq[2]
				<< ", " << contact_force
				<< ", " << num_contacts
				<< "\n";
		}

		// if (!fTimerDidSleep) {
		// 	cout << "Warning: timer underflow! dt: " << loop_dt << "\n";
		// }

		// update last time
		last_time = curr_time;
	}
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
    if ((key == 'P' || key == 'p') && action == GLFW_PRESS)
    {
        // pause simulation
        f_global_sim_pause = !f_global_sim_pause;
    }
    if ((key == GLFW_KEY_UP) && action == GLFW_PRESS) 
    {
    	vtorque += 0.1;
    }
    if ((key == GLFW_KEY_DOWN) && action == GLFW_PRESS) 
    {
    	vtorque -= 0.1;
    }
    if ((key == 'K' || key == 'k' ) && action == GLFW_PRESS) 
    {
    	fKickUp = true;
    }
    if ((key == GLFW_KEY_LEFT) && action == GLFW_PRESS) 
    {
    	rtorque -= 0.3;
    }
    if ((key == GLFW_KEY_RIGHT) && action == GLFW_PRESS) 
    {
    	rtorque += 0.3;
    }
}
