#include "ForceSensorSim.h"

namespace Sai2Simulation {

ForceSensorSim::ForceSensorSim(const std::string& robot_name,
							   const std::string& link_name,
							   const Eigen::Affine3d& transform_in_link,
							   std::shared_ptr<Sai2Model::Sai2Model> robot,
							   const double _filter_normalized_cutoff_freq)
	: _robot(robot) {
	_data = Sai2Model::ForceSensorData();
	_data.robot_name = robot_name;
	_data.link_name = link_name;
	_data.transform_in_link = transform_in_link;

	_use_filter = false;

	if(_filter_normalized_cutoff_freq > 0.0)
	{
		enableFilter(_filter_normalized_cutoff_freq);
	}

	_remove_spike = false;
	_first_iteration = true;
}

// dtor
ForceSensorSim::~ForceSensorSim() {}

// update force information
void ForceSensorSim::update(const std::shared_ptr<cDynamicWorld> dyn_world) {
	// NOTE that this assumes that the robot model is updated
	// get the list of contact forces acting on the link
	auto contact_list =
		dyn_world->getContactList(_data.robot_name, _data.link_name);
	// zero out current forces and moments
	_data.force_world_frame.setZero();
	_data.moment_world_frame.setZero();
	_data.force_local_frame.setZero();
	_data.moment_local_frame.setZero();
	// if list is empty, simply set forces and moments to 0
	if (contact_list.empty()) {
		return;
	}
	// transform to sensor frame
	Eigen::Vector3d rel_pos;
	Eigen::Vector3d link_pos = _robot->positionInWorld(
		_data.link_name, _data.transform_in_link.translation());
	for (uint pt_ind = 0; pt_ind < contact_list.size(); ++pt_ind) {
		_data.force_world_frame -= contact_list[pt_ind].second;
		rel_pos = contact_list[pt_ind].first - link_pos;
		_data.moment_world_frame -= rel_pos.cross(contact_list[pt_ind].second);
	}

	if (_first_iteration) {
		_previous_force_world_frame = _data.force_world_frame;
		_previous_moment_world_frame = _data.moment_world_frame;
		_first_iteration = false;
	}

	// Discretly remove spikes if required
	if (_remove_spike) {
		if (fabs(_data.force_world_frame[0] - _previous_force_world_frame[0]) >=
			_force_threshold) {
			_data.force_world_frame = _previous_force_world_frame;
			_data.moment_world_frame = _previous_moment_world_frame;
		}
	}

	// filter signals
	if (_use_filter) {
		_data.force_world_frame = _filter_force->update(_data.force_world_frame);
		_data.moment_world_frame =
			_filter_moment->update(_data.moment_world_frame);
	}

	_previous_force_world_frame = _data.force_world_frame;
	_previous_moment_world_frame = _data.moment_world_frame;

	Eigen::Matrix3d R_base_sensor = _robot->rotationInWorld(_data.link_name);
	R_base_sensor = R_base_sensor * _data.transform_in_link.rotation();

	_data.force_local_frame =
		R_base_sensor.transpose() * _data.force_world_frame;
	_data.moment_local_frame =
		R_base_sensor.transpose() * _data.moment_world_frame;
}

void ForceSensorSim::enableSpikeRemoval(const double force_threshold) {
	_remove_spike = true;
	_force_threshold = force_threshold;
}

void ForceSensorSim::enableFilter(const double normalized_cutoff_freq) {
	_use_filter = true;
	_filter_force.reset(
		new Sai2Common::ButterworthFilter(normalized_cutoff_freq));
	_filter_moment.reset(
		new Sai2Common::ButterworthFilter(normalized_cutoff_freq));
}

}  // namespace Sai2Simulation
