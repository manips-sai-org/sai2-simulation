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

	// while window is open:
	while (graphics->isWindowOpen()) {
		// update simulation
		for (const auto& object_name : sim->getObjectNames()) {
			sim->setObjectForceTorque(object_name,
									  graphics->getUITorques(object_name));
		}
		sim->integrate();

		// update graphics.
		for (const auto& object_name : sim->getObjectNames()) {
			graphics->updateObjectGraphics(object_name,
										   sim->getObjectPose(object_name),
										   sim->getObjectVelocity(object_name));
		}

		graphics->renderGraphicsWorld();
	}

	return 0;
}