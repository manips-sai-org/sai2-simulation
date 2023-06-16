#include <iostream>
#include <string>
#include <thread>
#include "unistd.h"
#include "Sai2Simulation.h"
#include "Sai2Model.h"
#include "Sai2Graphics.h"

using namespace std;

const string world_fname = "resources/world.urdf";

bool fSimulationRunning = false;

// sim
void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim);

int main (int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_fname << endl;

	// load simulation world
	auto sim = std::make_shared<Sai2Simulation::Sai2Simulation>(world_fname, false);

	// load graphics scene
	auto graphics = std::make_shared<Sai2Graphics::Sai2Graphics>(world_fname, "sai2-simulation example 02-fixed_joint", false);
	graphics->setBackgroundColor(0.3, 0.3, 0.3);

	// start the simulation
	thread sim_thread(simulation, sim);
	
    // while window is open:
    while (graphics->isWindowOpen()) {
		for(const auto& robot_name : sim->getRobotNames()){
			Eigen::VectorXd robot_q;
			sim->getJointPositions(robot_name, robot_q); // read the joint values from simulation
			graphics->updateRobotGraphics(robot_name, robot_q); // update the graphics with those joint values
		}
		graphics->updateDisplayedWorld();
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	return 0;
}


//------------------------------------------------------------------------------
void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim) {
	fSimulationRunning = true;
	double timestep = sim->timestep();

	while (fSimulationRunning) {
		usleep(timestep * 1e6);
		sim->integrate();
	}
}