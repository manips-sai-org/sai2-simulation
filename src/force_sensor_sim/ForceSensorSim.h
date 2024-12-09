// ForceSensorSim.h
// Force sensor for SAI-Simulation
#ifndef FORCE_SENSOR_SIM_H
#define FORCE_SENSOR_SIM_H

#include <SaiModel.h>
#include <filters/ButterworthFilter.h>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include "dynamics3d.h"
#include <memory>

namespace SaiSimulation {

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
		std::shared_ptr<SaiModel::SaiModel> model,
		const double filter_normalized_cutoff_freq = 0.0);

	ForceSensorSim(
		const std::string& object_name,
		const std::string& link_name,
		const Eigen::Affine3d& transform_in_link,
		std::shared_ptr<Eigen::Affine3d> object_pose,
		const double filter_normalized_cutoff_freq = 0.0);

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
	SaiModel::ForceSensorData getData() const {return _data;}

	// Enable filtering of the force and moment data
	void enableFilter(const double normalized_cutoff_freq);

	double getNormalizedCutoffFreq() const {
		return _filter_force->getNormalizedCutoffFreq();
	}

private:
	// handle to model interface if the sesnor is attached to a robot
	std::shared_ptr<SaiModel::SaiModel> _robot;

	// handle to object pose if the sensor is attached to an object
	std::shared_ptr<Eigen::Affine3d> _object_pose;

	// last updated data
	SaiModel::ForceSensorData _data;

	// filter for force and moment
	bool _use_filter;
	std::unique_ptr<SaiCommon::ButterworthFilter> _filter_force;
	std::unique_ptr<SaiCommon::ButterworthFilter> _filter_moment;

};

} // namespace SaiSimulation

#endif //FORCE_SENSOR_SIM_H
