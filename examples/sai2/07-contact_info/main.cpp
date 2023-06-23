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
Eigen::VectorXd q_robot;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// load simulation world
	auto sim = new Sai2Simulation::Sai2Simulation(world_file);
	sim->setTimestep(0.01);
	std::vector<Eigen::Vector3d> contact_points;
	std::vector<Eigen::Vector3d> contact_forces;

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file);

	// offset a joint initial condition
	sim->getJointPositions(robot_name, q_robot);
	sim->setJointPosition(robot_name, 0, q_robot[0] + 0.5);

	sim->setCollisionRestitution(0);

	// while window is open:
	while (graphics->isWindowOpen()) {
		// update simulation
		sim->integrate();

		// update kinematic models
		sim->getJointPositions(robot_name, q_robot);

		// update graphics. this automatically waits for the correct amount of
		// time
		graphics->updateRobotGraphics(robot_name, q_robot);
		for (const auto sensor_data : sim->getAllForceSensorData()) {
			graphics->updateDisplayedForceSensor(sensor_data);
		}
		graphics->updateDisplayedWorld();

		if (simulation_counter % 100 == 0) {
			sim->getContactList(contact_points, contact_forces, robot_name,
								"sensor_link");
			if (!contact_points.empty()) {
				std::cout << "contact at " << robot_name << " sensor_link"
						  << std::endl;
				int n = contact_points.size();
				for (int i = 0; i < n; i++) {
					std::cout << "contact point " << i << " : "
							  << contact_points[i].transpose() << "\n";
					std::cout << "contact force " << i << " : "
							  << contact_forces[i].transpose() << "\n";
				}
				std::cout << endl;
			}
		}

		simulation_counter++;
	}

	return 0;
}