#include <iostream>
#include <string>
#include <thread>

#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include "unistd.h"

using namespace std;

const string world_fname = "resources/world.urdf";

bool fSimulationRunning = false;

// sim
void simulation(shared_ptr<Sai2Simulation::Sai2Simulation> sim);

int main(int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_fname << endl;

	// load simulation world
	auto sim = make_shared<Sai2Simulation::Sai2Simulation>(world_fname, false);
	const string robot_name = sim->getRobotNames()[0];

	// load graphics scene
	auto graphics = make_shared<Sai2Graphics::Sai2Graphics>(
		world_fname, "sai2-simulation example 01-fixed_joint", false);
	graphics->addUIForceInteraction(robot_name);

	cout << endl
		 << "This example simulates a double pendulum robot with joint limits. "
			"The robot will stop in its fall because of the limits, It is "
			"possible to interact with the robot using right click to bring "
			"it up to its upper limit as well. In addition. damping on "
			"the joints is enabled by setting the <dynamics damping=\" ... "
			"\"/> in the robot urdf file for the joints"
		 << endl
		 << endl;

	// start the simulation
	thread sim_thread(simulation, sim);

	Eigen::VectorXd torques = Eigen::VectorXd::Zero(sim->dof(robot_name));

	// while window is open:
	while (graphics->isWindowOpen()) {
		// apply joint torque
		sim->setJointTorques(robot_name, torques);

		// update graphics.
		graphics->updateRobotGraphics(robot_name, sim->getJointPositions(robot_name));
		graphics->renderGraphicsWorld();
		torques = graphics->getUITorques(robot_name);
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