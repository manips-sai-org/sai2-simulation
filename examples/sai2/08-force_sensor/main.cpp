// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Sai2Simulation virtual world. A graphics model
// of it is also shown using Chai3D.

#include <Sai2Graphics.h>
#include <Sai2Simulation.h>

#include <iostream>
#include <string>

using namespace std;

const string world_file = "resources/world.urdf";
const string robot_name = "PPPBot";
const string link_name = "sensor_link";

unsigned long long simulation_counter = 0;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// load simulation world
	auto sim = new Sai2Simulation::Sai2Simulation(world_file);
	sim->setTimestep(0.01);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file);

	// add simulated force sensor
	Eigen::Affine3d T_link_sensor = Eigen::Affine3d::Identity();
	T_link_sensor.translate(Eigen::Vector3d(0.0, 0.0, 0.0));
	T_link_sensor.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
	sim->addSimulatedForceSensor(robot_name, link_name, T_link_sensor);

	// add sensor display in the graphics
	for (auto sensor_data : sim->getAllForceSensorData()) {
		graphics->addForceSensorDisplay(sensor_data);
	}

	// offset a joint initial condition
	sim->setJointPosition(robot_name, 0,
						  sim->getJointPositions(robot_name)[0] + 0.5);

	sim->setCollisionRestitution(0);

	// while window is open:
	while (graphics->isWindowOpen()) {
		// update simulation
		sim->integrate();

		// update graphics.
		graphics->updateRobotGraphics(robot_name,
									  sim->getJointPositions(robot_name));
		for (const auto sensor_data : sim->getAllForceSensorData()) {
			graphics->updateDisplayedForceSensor(sensor_data);
		}
		graphics->renderGraphicsWorld();

		if (simulation_counter == 150) {
			sim->setJointTorque(robot_name, 1, -70);
		}

		if (simulation_counter % 100 == 0) {
			std::cout << "force local frame:\t"
					  << sim->getSensedForce(robot_name, link_name).transpose()
					  << std::endl;
			std::cout
				<< "force world frame:\t"
				<< sim->getSensedForce(robot_name, link_name, false).transpose()
				<< std::endl;
			std::cout << "moment local frame:\t"
					  << sim->getSensedMoment(robot_name, link_name).transpose()
					  << std::endl;
			std::cout << "moment world frame:\t"
					  << sim->getSensedMoment(robot_name, link_name, false)
							 .transpose()
					  << std::endl;
			std::cout << std::endl;
		}

		simulation_counter++;
	}

	return 0;
}