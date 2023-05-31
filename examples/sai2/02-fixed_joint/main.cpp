#include <iostream>
#include <string>
#include <thread>
#include "unistd.h"
#include "Sai2Simulation.h"
#include "Sai2Model.h"
#include "Sai2Graphics.h"

using namespace std;

const string world_fname = "resources/world.urdf";
const string robot_name_1 = "PBot_fixed";
const string robot_name_2 = "PPBot";
const string camera_name = "camera_fixed";

bool fSimulationRunning = false;

// sim
void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim);

int main (int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_fname << endl;

	// load simulation world
	auto sim = std::make_shared<Sai2Simulation::Sai2Simulation>(world_fname, false);

	// load graphics scene
	auto graphics = std::make_shared<Sai2Graphics::Sai2Graphics>(world_fname, "sai2-simulation example 01-fixed_joint", false);
	graphics->setBackgroundColor(0.3, 0.3, 0.3);

	// start the simulation
	thread sim_thread(simulation, sim);
	
    // while window is open:
    while (graphics->isWindowOpen()) {

		// update kinematic models
		Eigen::VectorXd robot1_q;
		Eigen::VectorXd robot2_q;
		sim->getJointPositions(robot_name_1, robot1_q);
		sim->getJointPositions(robot_name_2, robot2_q);

		// update graphics from latest simulation config
		graphics->updateRobotGraphics(robot_name_1, robot1_q);
		graphics->updateRobotGraphics(robot_name_2, robot2_q);
		graphics->updateDisplayedWorld(camera_name);
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	return 0;
}


//------------------------------------------------------------------------------
void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim) {
	fSimulationRunning = true;

	bool fTimerDidSleep = true;
	while (fSimulationRunning) {
		usleep(1000);
		// integrate forward
		sim->integrate(0.001);
	}
}