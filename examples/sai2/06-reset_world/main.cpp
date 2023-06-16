// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Sai2Simulation virtual world. A graphics model of it is also shown using 
// Chai3D.

#include <Sai2Simulation.h>
#include <Sai2Graphics.h>

#include <iostream>
#include <string>

using namespace std;

const string world_file_1 = "resources/world.urdf";
const string world_file_2 = "resources/world2.urdf";

unsigned long long simulation_counter = 0;
Eigen::VectorXd q_robot;

int main() {
	// load simulation world
	auto sim = new Sai2Simulation::Sai2Simulation(world_file_1);
	auto robot_names = sim->getRobotNames();
	sim->setTimestep(0.01);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file_1);

    // while window is open:
    while (graphics->isWindowOpen())
	{
		// update simulation
		sim->integrate();

		// update graphics
		for(const auto name : robot_names) {
			sim->getJointPositions(name, q_robot);
			graphics->updateRobotGraphics(name, q_robot);

		}
		graphics->updateDisplayedWorld();

		if(simulation_counter % 700 == 350)
		{
			sim->resetWorld(world_file_2);
			graphics->resetWorld(world_file_2);
			robot_names = sim->getRobotNames();
		}
		if(simulation_counter % 700 == 0) {
			sim->resetWorld(world_file_1);
			graphics->resetWorld(world_file_1);
			robot_names = sim->getRobotNames();			
		}

		simulation_counter++;
	}

	return 0;
}