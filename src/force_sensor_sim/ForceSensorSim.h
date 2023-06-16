// ForceSensorSim.h
// Force sensor for SAI2-Simulation
#ifndef FORCE_SENSOR_SIM_H
#define FORCE_SENSOR_SIM_H

#include <Sai2Model.h>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include "dynamics3d.h"
#include <memory>

namespace Sai2Simulation {

// Simulated force sensor type.
// Note that this implementation ignores the mass and inertia of the object
// attached beyond the force sensor.
// Note also that this assumes the force sensor to be located just between the
// link and the end-effector. That is, it cannot account for any joint reaction
// forces in case the sensor is to be attached on the link between two joints.
class ForceSensorSim {
public:
	// ctor
	ForceSensorSim(
		const std::string& robot_name,
		const std::string& link_name,
		const Eigen::Affine3d& transform_in_link,
		std::shared_ptr<Sai2Model::Sai2Model> model);

	//dtor
	~ForceSensorSim();

	// update force information
	void update(const std::shared_ptr<cDynamicWorld> dyn_world);

	// get force applied to sensor body in world coordinates
	Eigen::Vector3d getForce() const;

	// get force applied to sensor body in local sensor frame
	Eigen::Vector3d getForceLocalFrame() const;

	// get moment applied to sensor body in world coordinates
	Eigen::Vector3d getMoment() const;

	// get moment applied to sensor body in local sensor frame
	Eigen::Vector3d getMomentLocalFrame() const;

	// get force and moment applied to sensor body in world frame as a 6dof vector (force first moment second)
	Eigen::VectorXd getForceMoment() const;

	// get force and moment applied to sensor body in local sensor frame as a 6dof vector (force first moment second)
	Eigen::VectorXd getForceMomentLocalFrame() const;

	// Discretly remove spikes from the force data
	void enableSpikeRemoval(const double force_threshold);

private:
	// handle to model interface
	std::shared_ptr<Sai2Model::Sai2Model> _model;

	// last updated data
	Sai2Model::ForceSensorData _data;

	//spike removal
	bool _remove_spike;
	bool _first_iteration;
	double _force_threshold;
	Eigen::Vector3d _previous_force;
	Eigen::Vector3d _previous_torque;

};

} // namespace Sai2Simulation

#endif //FORCE_SENSOR_SIM_H
