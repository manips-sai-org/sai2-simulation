// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Sai2Simulation virtual world. A graphics model of it is also shown using 
// Chai3D.

#include <Sai2Simulation.h>
#include <Sai2Graphics.h>

#include <iostream>
#include <string>

using namespace std;

const string world_file = "resources/world.urdf";
const string robot_name = "PBot1";
const string link_name = "sensor_link";

unsigned long long simulation_counter = 0;
Eigen::VectorXd q_robot;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// load simulation world
	auto sim = new Sai2Simulation::Sai2Simulation(world_file);
	sim->setTimestep(0.01);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file);

	// add simulated force sensor
	Eigen::Affine3d T_link_sensor = Eigen::Affine3d::Identity();
	T_link_sensor.translate(Eigen::Vector3d(0.0, 0.05, 0.0));
	T_link_sensor.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
	sim->addSimulatedForceSensor(robot_name, link_name, T_link_sensor);

	// offset a joint initial condition
	sim->getJointPositions(robot_name, q_robot);
	sim->setJointPosition(robot_name, 0, q_robot[0] + 0.5);
	
	sim->setCollisionRestitution(0);

    // while window is open:
    while (graphics->isWindowOpen())
	{
		// update simulation
		sim->integrate();

		// update kinematic models
		sim->getJointPositions(robot_name, q_robot);

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		graphics->updateRobotGraphics(robot_name, q_robot);
		graphics->updateDisplayedWorld();

		if(simulation_counter %300 == 0)
		{
			std::cout << "force local frame:\t" << sim->getSensedForce(robot_name, link_name).transpose() << std::endl;
			std::cout << "force world frame:\t" << sim->getSensedForce(robot_name, link_name, false).transpose() << std::endl;
			std::cout << "moment local frame:\t" << sim->getSensedMoment(robot_name, link_name).transpose() << std::endl;
			std::cout << "moment world frame:\t" << sim->getSensedMoment(robot_name, link_name, false).transpose() << std::endl;
			std::cout << std::endl;
		}

		simulation_counter++;
	}

	return 0;
}