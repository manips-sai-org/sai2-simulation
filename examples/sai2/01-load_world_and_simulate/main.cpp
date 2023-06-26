#include <iostream>
#include <string>
#include <thread>

#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include "unistd.h"

using namespace std;

const string world_fname = "resources/world.urdf";
const string robot_name = "RRBot";
const string camera_name = "camera_fixed";

bool fSimulationRunning = false;

// sim
void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim);

int main(int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_fname << endl;

	// load simulation world
	auto sim = make_shared<Sai2Simulation::Sai2Simulation>(world_fname);

	// load graphics scene
	auto graphics = make_shared<Sai2Graphics::Sai2Graphics>(world_fname);
	graphics->setBackgroundColor(0.2, 0.2, 0.2);

	cout << endl
		 << "This example loads a double pendulum and simulates gravity. The "
<<<<<<< HEAD
			"graphics display is updated by the simulated robot joint angles"
=======
			"graphics display is updated by the simulated robot joint angles. "
			"After a certain time, the simulated robot starts compensating for "
			"gravity. After some more time, the gravity compensation "
			"stops."
>>>>>>> master
		 << endl
		 << endl;

	// start the simulation
	thread sim_thread(simulation, sim);

<<<<<<< HEAD
=======
	unsigned long long counter = 0;

>>>>>>> master
	// while window is open:
	while (graphics->isWindowOpen()) {
		// read joint position from simulation
		Eigen::VectorXd robot_q;
		sim->getJointPositions(robot_name, robot_q);
		// update graphics. this automatically waits for the correct amount of
		// time
		graphics->updateRobotGraphics(robot_name, robot_q);
		graphics->updateDisplayedWorld();

		if(counter == 300) {
			cout << "\nenabling gravity compensation\n" << endl;
			sim->enableGravityCompensation(true);
		}
		if(counter == 500) {
			cout << "\ndisabling gravity compensation\n" << endl;
			sim->enableGravityCompensation(false);
		}
		counter++;
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
		// wait
		usleep(timestep * 1e6);
		// integrate forward
		sim->integrate();
	}
}