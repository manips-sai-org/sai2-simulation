// This example application loads a URDF world file and simulates two robots
// with physics and contact in a SaiSimulation virtual world. A graphics model
// of it is also shown using Chai3D.

#include <SaiGraphics.h>
#include <SaiSimulation.h>

#include <iostream>
#include <string>

using namespace std;

const string world_file_1 = string(EXAMPLES_FOLDER) + "/06-reset_world/world1.urdf";
const string world_file_2 = string(EXAMPLES_FOLDER) + "/06-reset_world/world2.urdf";

unsigned long long simulation_counter = 0;

int main() {
	// load simulation world
	auto sim = new SaiSimulation::SaiSimulation(world_file_1);
	auto robot_names = sim->getRobotNames();
	sim->setTimestep(0.01);

	// load graphics scene
	auto graphics = new SaiGraphics::SaiGraphics(world_file_1);

	cout << endl
		 << "In this example, the simulation and graphics are switching "
			"between two different worlds periodically"
		 << endl
		 << endl;

	// while window is open:
	while (graphics->isWindowOpen()) {
		// update simulation
		sim->integrate();

		// update graphics
		for (const auto name : robot_names) {
			graphics->updateRobotGraphics(name, sim->getJointPositions(name));
		}
		graphics->renderGraphicsWorld();

		if (simulation_counter % 700 == 350) {
			sim->resetWorld(world_file_2);
			graphics->resetWorld(world_file_2);
			robot_names = sim->getRobotNames();
		}
		if (simulation_counter % 700 == 0) {
			sim->resetWorld(world_file_1);
			graphics->resetWorld(world_file_1);
			robot_names = sim->getRobotNames();
		}

		simulation_counter++;
	}

	return 0;
}