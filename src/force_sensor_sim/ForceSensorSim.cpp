#include "ForceSensorSim.h"

namespace Sai2Simulation {

ForceSensorSim::ForceSensorSim(
	const std::string& robot_name,
	const std::string& link_name,
	const Eigen::Affine3d& transform_in_link,
	std::shared_ptr<Sai2Model::Sai2Model> robot)
:	_robot(robot)
{
	_data = Sai2Model::ForceSensorData();
	_data._robot_name = robot_name;
	_data._link_name = link_name;
	_data._transform_in_link = transform_in_link;

	_remove_spike = false;
	_first_iteration = true;
}

//dtor
ForceSensorSim::~ForceSensorSim() {
}

// update force information
void ForceSensorSim::update(const std::shared_ptr<cDynamicWorld> dyn_world) {
	// NOTE that this assumes that the robot model is updated
	// get the list of contact forces acting on the link
	std::vector<Eigen::Vector3d> force_list;
	std::vector<Eigen::Vector3d> point_list;
	dyn_world->getContactList(
		point_list,
		force_list,
		_data._robot_name,
		_data._link_name);
	// zero out current forces and moments
	_data._force_world_frame.setZero();
	_data._moment_world_frame.setZero();
	_data._force_local_frame.setZero();
	_data._moment_local_frame.setZero();
	// if list is empty, simply set forces and moments to 0
	if(point_list.empty()) {
		return;
	}
	// transform to sensor frame
	Eigen::Vector3d rel_pos;
	Eigen::Vector3d link_pos;
	_robot->positionInWorld(link_pos, _data._link_name, _data._transform_in_link.translation());
	for (uint pt_ind=0; pt_ind < point_list.size(); ++pt_ind) {
		_data._force_world_frame -= force_list[pt_ind];
		rel_pos = point_list[pt_ind] - link_pos;
		_data._moment_world_frame -= rel_pos.cross(force_list[pt_ind]);
	}

	if (_first_iteration)
	{
		_previous_force_world_frame = _data._force_world_frame;
		_previous_moment_world_frame = _data._moment_world_frame;
		_first_iteration=false;
	}

	// Discretly remove spikes if required
	if (_remove_spike)
	{
		if(fabs(_data._force_world_frame[0]-_previous_force_world_frame[0]) >= _force_threshold)
		{
			_data._force_world_frame = _previous_force_world_frame;
			_data._moment_world_frame = _previous_moment_world_frame;
		}
	}

	_previous_force_world_frame = _data._force_world_frame;
	_previous_moment_world_frame = _data._moment_world_frame;

	Eigen::Matrix3d R_base_sensor;
	_robot->rotationInWorld(R_base_sensor, _data._link_name);
	R_base_sensor = R_base_sensor * _data._transform_in_link.rotation();

	_data._force_local_frame = R_base_sensor.transpose() * _data._force_world_frame;
	_data._moment_local_frame = R_base_sensor.transpose() * _data._moment_world_frame;
}

void ForceSensorSim::enableSpikeRemoval(const double force_threshold)
{
	_remove_spike = true;
	_force_threshold = force_threshold;
}

} // namespace Sai2Simulation
