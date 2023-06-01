#include <iostream>
#include <string>
#include <thread>
#include "unistd.h"
#include "Sai2Simulation.h"
#include "Sai2Model.h"
#include "Sai2Graphics.h"

using namespace std;

const string world_fname = "resources/world.urdf";
const string camera_name = "camera_fixed";

bool fSimulationRunning = false;

std::map<std::string, Eigen::VectorXd> ui_torques;
std::map<std::string, Eigen::VectorXd> robot_q;

// sim
void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim);

int main (int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_fname << endl;

	// load simulation world
	auto sim = std::make_shared<Sai2Simulation::Sai2Simulation>(world_fname);
	std::vector<std::string> robot_names = sim->getRobotNames();
	for(const auto& robot_name : robot_names) {
		ui_torques[robot_name] = Eigen::VectorXd::Zero(sim->dof(robot_name));
		robot_q[robot_name] = Eigen::VectorXd::Zero(sim->q_size(robot_name));
	}

	// load graphics scene
	auto graphics = std::make_shared<Sai2Graphics::Sai2Graphics>(world_fname, "sai2-simulation example 01-fixed_joint", false);
	for(const auto& robot_name : robot_names) {
		graphics->addUIForceInteraction(robot_name);
	}

	// start the simulation
	thread sim_thread(simulation, sim);
	
    // while window is open:
    while (graphics->isWindowOpen()) {

		// update graphics from latest simulation config
		for(auto& pair : robot_q) {
			sim->getJointPositions(pair.first, pair.second);
			graphics->updateRobotGraphics(pair.first, pair.second);
		}
		graphics->updateDisplayedWorld(camera_name);
		for(auto& pair : ui_torques) {
			graphics->getUITorques(pair.first, pair.second);
		}
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

		for(const auto& pair : ui_torques) {
			sim->setJointTorques(pair.first, pair.second);
		}

		// integrate forward
		sim->integrate(0.001);
	}
}