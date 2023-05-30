#include <iostream>
#include <string>
#include <thread>
#include "unistd.h"
#include "Sai2Simulation.h"
#include "Sai2Model.h"
#include "Sai2Graphics.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

using namespace std;

const string world_fname = "resources/world1.urdf";
const string robot_fname = "resources/pbot1.urdf";
const string robot_name = "PBot1";
const string camera_name = "camera_fixed";

bool fSimulationRunning = false;

// sim
void simulation(Sai2Model::Sai2Model* robot, Sai2Simulation::Sai2Simulation* sim);

int main (int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_fname << endl;

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_fname, true);
	robot->updateModel();

	// load simulation world
	auto sim = new Sai2Simulation::Sai2Simulation(world_fname, false);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_fname, "sai2-simulation example 01-fixed_joint", false);
	// graphics->_world->setBackgroundColor(0.3, 0.3, 0.3);

	cout << "Robot " << robot_name << " has " << robot->dof() << " joints in model." << endl;
	cout << "Robot " << robot_name << " has " << sim->dof(robot_name) << " joints in simulation." << endl;

	Eigen::VectorXd gravity_torques;
	robot->jointGravityVector(gravity_torques);
	cout << "gravity vector: " << gravity_torques.transpose() << endl;
	cout << "Mass matrix: " << robot->M() << endl;

	// start the simulation
	thread sim_thread(simulation, robot, sim);
	
	Eigen::Vector3d link1_pos;
	Eigen::VectorXd torques(1);
	const string link1_name = "link1";

    // while window is open:
    while (graphics->isWindowOpen()) {
    	// apply joint torque
    	torques << 0.0;//2.0*9.81;
    	sim->setJointTorques(robot_name, torques);

		// update kinematic models
		Eigen::VectorXd robot_q = Eigen::VectorXd::Zero(robot->dof());
		sim->getJointPositions(robot_name, robot_q);
		robot->set_q(robot_q);
		robot->updateModel();
		robot->position(link1_pos, link1_name, Eigen::Vector3d::Zero());
		// cout << "link 1 position" << link1_pos.transpose() << endl;

		// update graphics. this automatically waits for the correct amount of time
		graphics->updateRobotGraphics(robot_name, robot_q);
		graphics->updateDisplayedWorld(camera_name);
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	return 0;
}


//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot, Sai2Simulation::Sai2Simulation* sim) {
	fSimulationRunning = true;

	bool fTimerDidSleep = true;
	while (fSimulationRunning) {
		usleep(1000);
		// integrate forward
		sim->integrate(0.001);
	}
}