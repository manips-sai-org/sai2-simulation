#include <iostream>
#include <string>
#include <thread>
#include "unistd.h"
#include "Sai2Simulation.h"
#include "Sai2Graphics.h"

using namespace std;

const string world_fname = "resources/world.urdf";

bool fSimulationRunning = false;

// sim
void simulation(shared_ptr<Sai2Simulation::Sai2Simulation> sim);

int main (int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_fname << endl;

	// load simulation world
	auto sim = make_shared<Sai2Simulation::Sai2Simulation>(world_fname, false);
	const string robot_name = sim->getRobotNames()[0];

	// load graphics scene
	auto graphics = make_shared<Sai2Graphics::Sai2Graphics>(world_fname, "sai2-simulation example 01-fixed_joint", false);
	graphics->addUIForceInteraction(robot_name);

	// start the simulation
	thread sim_thread(simulation, sim);
	
	Eigen::VectorXd torques = Eigen::VectorXd::Zero(sim->dof(robot_name));

    // while window is open:
    while (graphics->isWindowOpen()) {
    	// apply joint torque
    	sim->setJointTorques(robot_name, torques);

		// get joint positions from simulation
		Eigen::VectorXd robot_q;
		sim->getJointPositions(robot_name, robot_q);

		// update graphics. this automatically waits for the correct amount of time
		graphics->updateRobotGraphics(robot_name, robot_q);
		graphics->updateDisplayedWorld();
		graphics->getUITorques(robot_name, torques);
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	return 0;
}


//------------------------------------------------------------------------------
void simulation(shared_ptr<Sai2Simulation::Sai2Simulation> sim) {
	fSimulationRunning = true;
	double timestep = sim->timestep();

	while (fSimulationRunning) {
		usleep(timestep * 1e6);
		// integrate forward
		sim->integrate();
	}
}