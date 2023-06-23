// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Sai2Simulation virtual world. A graphics model
// of it is also shown using Chai3D.

#include <Sai2Graphics.h>
#include <Sai2Simulation.h>

#include <iostream>
#include <string>

using namespace std;

const string world_file = "resources/world.urdf";

unsigned long long loop_counter = 0;

int main() {
	// load simulation world
	auto sim = new Sai2Simulation::Sai2Simulation(world_file);
	auto robot_names = sim->getRobotNames();
	sim->setTimestep(0.01);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file);

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
			Eigen::VectorXd q_robot;
			sim->getJointPositions(name, q_robot);
			graphics->updateRobotGraphics(name, q_robot);
		}
		graphics->updateDisplayedWorld();

		if (loop_counter == 700) {
			cout << "elapsed sim time: " << sim->time() << " seconds" << endl;
			cout << "number of loops: " << loop_counter << endl;
			cout << "sim timestep was: " << sim->timestep() << " seconds"
				 << endl;
			cout << "Pausing simulation" << endl << endl;
			sim->pause();
		}
		if (loop_counter == 1000) {
			cout << "elapsed sim time: " << sim->time() << " seconds" << endl;
			cout << "number of loops: " << loop_counter << endl;
			cout << "unpausing simulation" << endl << endl;
			sim->unpause();
		}
		if (loop_counter == 1500) {
			sim->resetWorld(world_file);
			cout << endl
				 << "elapsed sim time: " << sim->time() << " seconds" << endl;
			cout << "number of loops: " << loop_counter << endl;
			cout << "resetting simulation and setting timestep to 50 ms" << endl
				 << endl;
			sim->setTimestep(0.05);
		}
		if (loop_counter == 2000) {
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