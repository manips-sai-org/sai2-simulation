#include <SaiGraphics.h>
#include <SaiSimulation.h>

#include <iostream>
#include <string>
#include <thread>

#include "unistd.h"

using namespace std;

const string world_file =
	string(EXAMPLES_FOLDER) + "/09-dynamic_objects/world.urdf";
bool fSimulationRunning = false;

void simrun(std::shared_ptr<SaiSimulation::SaiSimulation> sim) {
	const double sim_timestep = 0.001;
	sim->setTimestep(sim_timestep);
	fSimulationRunning = true;
	while (fSimulationRunning) {
		usleep(sim_timestep * 1e6);
		sim->integrate();
	}
}

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// load simulation world
	auto sim = make_shared<SaiSimulation::SaiSimulation>(world_file);
	sim->setCollisionRestitution(0.1);
	sim->setCoeffFrictionStatic(0.8);
	for (const auto& object_name : sim->getObjectNames()) {
		sim->addSimulatedForceSensor(object_name, Eigen::Affine3d::Identity(),
									 5.0);
	}

	// load graphics scene
	auto graphics = make_shared<SaiGraphics::SaiGraphics>(world_file);
	for (const auto& object_name : sim->getObjectNames()) {
		graphics->addUIForceInteraction(object_name);
	}
	for (const auto sensor_data : sim->getAllForceSensorData()) {
		graphics->addForceSensorDisplay(sensor_data);
	}

	std::thread sim_thread(simrun, sim);

	// while window is open:
	while (graphics->isWindowOpen()) {
		// update simulation
		for (const auto& object_name : sim->getObjectNames()) {
			sim->setObjectForceTorque(object_name,
									  graphics->getUITorques(object_name));
		}

		// update graphics.
		for (const auto& object_name : sim->getObjectNames()) {
			graphics->updateObjectGraphics(object_name,
										   sim->getObjectPose(object_name),
										   sim->getObjectVelocity(object_name));
		}
		for (const auto sensor_data : sim->getAllForceSensorData()) {
			graphics->updateDisplayedForceSensor(sensor_data);
		}

		graphics->renderGraphicsWorld();
	}

	fSimulationRunning = false;
	sim_thread.join();

	return 0;
}