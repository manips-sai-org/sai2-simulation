#include <iostream>
#include <string>
#include <thread>

#include "SaiGraphics.h"
#include "SaiModel.h"
#include "SaiSimulation.h"
#include "unistd.h"

using namespace std;

const string world_fname =
	string(EXAMPLES_FOLDER) + "/02-fixed_joint/world.urdf";

bool fSimulationRunning = false;

// sim
void simulation(std::shared_ptr<SaiSimulation::SaiSimulation> sim);

int main(int argc, char** argv) {
	SaiModel::URDF_FOLDERS["EXAMPLE_02_FOLDER"] =
		string(EXAMPLES_FOLDER) + "/02-fixed_joint";
	cout << "Loading URDF world model file: " << world_fname << endl;

	// load simulation world
	auto sim =
		std::make_shared<SaiSimulation::SaiSimulation>(world_fname, false);
	sim->setCollisionRestitution(1.0);

	// load graphics scene
	auto graphics = std::make_shared<SaiGraphics::SaiGraphics>(
		world_fname, "sai-simulation example 02-fixed_joint", false);
	graphics->setBackgroundColor(0.3, 0.3, 0.3);

	cout << endl
		 << "This example simulates two robots bouncing on a table. They have "
			"two spheres that can move only vertically. The left robot had a "
			"moving joint between the two spheres and the right robot has a "
			"fixed joint between them"
		 << endl
		 << endl;

	// start the simulation
	thread sim_thread(simulation, sim);

	while (graphics->isWindowOpen()) {
		for (const auto& robot_name : sim->getRobotNames()) {
			graphics->updateRobotGraphics(robot_name,
										  sim->getJointPositions(robot_name));
		}
		graphics->renderGraphicsWorld();
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	return 0;
}

//------------------------------------------------------------------------------
void simulation(std::shared_ptr<SaiSimulation::SaiSimulation> sim) {
	fSimulationRunning = true;
	double timestep = sim->timestep();

	while (fSimulationRunning) {
		usleep(timestep * 1e6);
		sim->integrate();
	}
}