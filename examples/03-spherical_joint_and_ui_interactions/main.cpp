#include <iostream>
#include <string>
#include <thread>

#include "SaiGraphics.h"
#include "SaiModel.h"
#include "SaiSimulation.h"
#include "unistd.h"

using namespace std;

const string world_fname = string(EXAMPLES_FOLDER) +
						   "/03-spherical_joint_and_ui_interactions/world.urdf";

bool fSimulationRunning = false;

std::map<std::string, Eigen::VectorXd> ui_torques;

// sim
void simulation(std::shared_ptr<SaiSimulation::SaiSimulation> sim);

int main(int argc, char** argv) {
	SaiModel::URDF_FOLDERS["EXAMPLE_03_FOLDER"] =
		string(EXAMPLES_FOLDER) + "/03-spherical_joint_and_ui_interactions";
	cout << "Loading URDF world model file: " << world_fname << endl;

	// load simulation world
	auto sim = std::make_shared<SaiSimulation::SaiSimulation>(world_fname);
	std::vector<std::string> robot_names = sim->getRobotNames();
	for (const auto& robot_name : robot_names) {
		ui_torques[robot_name] = Eigen::VectorXd::Zero(sim->dof(robot_name));
	}

	// load graphics scene
	auto graphics = std::make_shared<SaiGraphics::SaiGraphics>(
		world_fname, "sai-simulation example 03-spherical_joint", false);
	// enable ui force interaction on all robots
	for (const auto& robot_name : robot_names) {
		graphics->addUIForceInteraction(robot_name);
	}

	cout << endl
		 << "This example simulates two 3D pendulum type robots. The left one "
			"is made using 3 revolute joints with intersecting axes, the right "
			"one is made using a spherical joint.\nIn addition, it is possible "
			"to interact with the robots using right click to apply a force on "
			"a point of the robots (use press also shoft to apply a moment)"
		 << endl
		 << endl;

	// start the simulation
	thread sim_thread(simulation, sim);

	// while window is open:
	while (graphics->isWindowOpen()) {
		// update graphics from latest simulation config
		for (const auto& robot_name : robot_names) {
			graphics->updateRobotGraphics(robot_name,
										  sim->getJointPositions(robot_name));
		}
		graphics->renderGraphicsWorld();

		// get the torques applied by the user via the ui
		for (const auto& robot_name : robot_names) {
			ui_torques[robot_name] = graphics->getUITorques(robot_name);
		}
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

	bool fTimerDidSleep = true;
	while (fSimulationRunning) {
		usleep(1000);
		// apply ui torques to the simulation
		for (const auto& pair : ui_torques) {
			sim->setJointTorques(pair.first, pair.second);
		}
		// integrate forward
		sim->integrate();
	}
}