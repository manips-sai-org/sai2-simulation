// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Sai2Simulation virtual world. A graphics model
// of it is also shown using Chai3D.

#include <Sai2Graphics.h>
#include <Sai2Simulation.h>

#include <iostream>
#include <string>

using namespace std;

const string world_file = "resources/world.urdf";

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// load simulation world
	auto sim = make_shared<Sai2Simulation::Sai2Simulation>(world_file);
	sim->setTimestep(0.01);
	sim->setCollisionRestitution(0);

	// load graphics scene
	auto graphics = make_shared<Sai2Graphics::Sai2Graphics>(world_file);
	for (const auto& object_name : sim->getObjectNames()) {
		graphics->addUIForceInteraction(object_name);
	}

	// add simulated force sensor
	// for (const auto& object_name : sim->getObjectNames()) {
	// 	Eigen::Affine3d T_link_sensor = Eigen::Affine3d::Identity();
	// 	T_link_sensor.translate(Eigen::Vector3d(0.0, 0.0, 0.0));
	// 	T_link_sensor.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
	// 	double sensor_filter_cutoff_freq = 30.0;
	// 	sim->addSimulatedForceSensor(object_name, "object_link", T_link_sensor,
	// 								 sensor_filter_cutoff_freq);
	// }

	// // add sensor display in the graphics
	// for (auto sensor_data : sim->getAllForceSensorData()) {
	// 	graphics->addForceSensorDisplay(sensor_data);
	// }

	// while window is open:
	while (graphics->isWindowOpen()) {
		// update simulation
		for(const auto& object_name : sim->getObjectNames()) {
			sim->setObjectForceTorque(object_name, graphics->getUITorques(object_name));
		}
		sim->integrate();

		// update graphics.
		for (const auto& object_name : sim->getObjectNames()) {
			graphics->updateObjectGraphics(object_name, sim->getObjectPose(object_name));
		}
		// for (const auto sensor_data : sim->getAllForceSensorData()) {
		// 	graphics->updateDisplayedForceSensor(sensor_data);
		// }
		graphics->renderGraphicsWorld();

	}

	return 0;
}