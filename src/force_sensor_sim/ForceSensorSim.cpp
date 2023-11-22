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

	_data.force_world_frame.setZero();
	_data.moment_world_frame.setZero();
	_data.force_local_frame.setZero();
	_data.moment_local_frame.setZero();

	_use_filter = false;

	if (_filter_normalized_cutoff_freq > 0.0) {
		enableFilter(_filter_normalized_cutoff_freq);
	}
}

// dtor
ForceSensorSim::~ForceSensorSim() {}

// update force information
void ForceSensorSim::update(const std::shared_ptr<cDynamicWorld> dyn_world) {
	// NOTE that this assumes that the robot model is updated
	// get the list of contact forces acting on the link
	auto contact_list =
		dyn_world->getContactList(_data.robot_name, _data.link_name);

	Vector3d force_world_frame = Vector3d::Zero();
	Vector3d moment_world_frame = Vector3d::Zero();

	if(!contact_list.empty()) {
		// transform to sensor frame
		Eigen::Vector3d rel_pos;
		Eigen::Vector3d link_pos = _robot->positionInWorld(
			_data.link_name, _data.transform_in_link.translation());
		for (uint pt_ind = 0; pt_ind < contact_list.size(); ++pt_ind) {
			force_world_frame -= contact_list[pt_ind].second;
			rel_pos = contact_list[pt_ind].first - link_pos;
			moment_world_frame -=
				rel_pos.cross(contact_list[pt_ind].second);
		}
	}

	// filter signals
	if (_use_filter) {
		_data.force_world_frame =
			_filter_force->update(force_world_frame);
		_data.moment_world_frame =
			_filter_moment->update(moment_world_frame);
	} else {
		_data.force_world_frame = force_world_frame;
		_data.moment_world_frame = moment_world_frame;
	}

	Eigen::Matrix3d R_base_sensor = _robot->rotationInWorld(_data.link_name);
	R_base_sensor = R_base_sensor * _data.transform_in_link.rotation();

	_data.force_local_frame =
		R_base_sensor.transpose() * _data.force_world_frame;
	_data.moment_local_frame =
		R_base_sensor.transpose() * _data.moment_world_frame;
}

void ForceSensorSim::enableFilter(const double normalized_cutoff_freq) {
	_use_filter = true;
	_filter_force.reset(
		new Sai2Common::ButterworthFilter(normalized_cutoff_freq));
	_filter_moment.reset(
		new Sai2Common::ButterworthFilter(normalized_cutoff_freq));
}

}  // namespace Sai2Simulation
