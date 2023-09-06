// ForceSensorSim.h
// Force sensor for SAI2-Simulation
#ifndef FORCE_SENSOR_SIM_H
#define FORCE_SENSOR_SIM_H

#include <Sai2Model.h>
#include <filters/ButterworthFilter.h>
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
		std::shared_ptr<Sai2Model::Sai2Model> model,
		const double _filter_normalized_cutoff_freq = 0.0);

	//dtor
	~ForceSensorSim();

	// update force information
	void update(const std::shared_ptr<cDynamicWorld> dyn_world);

	// get force applied by sensor body to the environment in world coordinates
	const Eigen::Vector3d& getForceWorldFrame() const {return _data.force_world_frame;};

	// get force applied by sensor body to the environment in local sensor frame
	const Eigen::Vector3d& getForceLocalFrame() const {return _data.force_local_frame;};

	// get moment applied by sensor body to the environment in world coordinates
	const Eigen::Vector3d& getMomentWorldFrame() const {return _data.moment_world_frame;};

	// get moment applied by sensor body to the environment in local sensor frame
	const Eigen::Vector3d& getMomentLocalFrame() const {return _data.moment_local_frame;};

	// get full data
	Sai2Model::ForceSensorData getData() const {return _data;}

	// Discretly remove spikes from the force data
	void enableSpikeRemoval(const double force_threshold);

	// Enable filtering of the force and moment data
	void enableFilter(const double normalized_cutoff_freq);

private:
	// handle to model interface
	std::shared_ptr<Sai2Model::Sai2Model> _robot;

	// last updated data
	Sai2Model::ForceSensorData _data;

	//spike removal
	bool _remove_spike;
	bool _first_iteration;
	double _force_threshold;
	Eigen::Vector3d _previous_force_world_frame;
	Eigen::Vector3d _previous_moment_world_frame;

	// filter for force and moment
	bool _use_filter;
	std::unique_ptr<Sai2Common::ButterworthFilter> _filter_force;
	std::unique_ptr<Sai2Common::ButterworthFilter> _filter_moment;

};

} // namespace Sai2Simulation

#endif //FORCE_SENSOR_SIM_H
