// This example application loads a URDF world file and simulates two robots
// with physics and contact in a SaiSimulation virtual world. A graphics model
// of it is also shown using Chai3D.

#include <SaiGraphics.h>
#include <SaiSimulation.h>

#include <iostream>
#include <string>

using namespace std;

const string world_file =
	string(EXAMPLES_FOLDER) + "/05-pause_sim/world.urdf";

unsigned long long loop_counter = 0;

int main() {
	// load simulation world
	auto sim = new SaiSimulation::SaiSimulation(world_file);
	auto robot_names = sim->getRobotNames();
	sim->setTimestep(0.01);

	// load graphics scene
	auto graphics = new SaiGraphics::SaiGraphics(world_file);

	cout << endl
		 << "In this example, the simulation is paused, then unpaused, and "
			"then reset with a bigger timestep"
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

		if (loop_counter == 300) {
			cout << "elapsed sim time: " << sim->time() << " seconds" << endl;
			cout << "number of loops: " << loop_counter << endl;
			cout << "sim timestep was: " << sim->timestep() << " seconds"
				 << endl;
			cout << "Pausing simulation" << endl << endl;
			sim->pause();
		}
		if (loop_counter == 500) {
			cout << "elapsed sim time: " << sim->time() << " seconds" << endl;
			cout << "number of loops: " << loop_counter << endl;
			cout << "unpausing simulation" << endl << endl;
			sim->unpause();
		}
		if (loop_counter == 700) {
			sim->resetWorld(world_file);
			cout << endl
				 << "elapsed sim time: " << sim->time() << " seconds" << endl;
			cout << "number of loops: " << loop_counter << endl;
			cout << "resetting simulation and setting timestep to 50 ms" << endl
				 << endl;
			sim->setTimestep(0.05);
		}
		if (loop_counter == 900) {
			cout << "elapsed sim time: " << sim->time() << " seconds" << endl;
			cout << "number of loops: " << loop_counter << endl;
			cout << "sim timestep was: " << sim->timestep() << " seconds"
				 << endl
				 << endl;
		}

		loop_counter++;
	}

	return 0;
}