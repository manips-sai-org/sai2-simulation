#include "ForceSensorSim.h"

namespace Sai2Simulation {

ForceSensorSim::ForceSensorSim(
	const std::string& robot_name,
	const std::string& link_name,
	const Eigen::Affine3d& transform_in_link,
	std::shared_ptr<Sai2Model::Sai2Model> model)
:	_model(model)
{
	_data = std::make_shared<ForceSensorData>();
	_data->_robot_name = robot_name;
	_data->_link_name = link_name;
	_data->_transform_in_link = transform_in_link;

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
		_data->_robot_name,
		_data->_link_name);
	// zero out current forces and moments
	_data->_force.setZero();
	_data->_moment.setZero();
	// if list is empty, simply set forces and moments to 0
	if(point_list.empty()) {
		return;
	}
	// transform to sensor frame
	Eigen::Vector3d rel_pos;
	Eigen::Vector3d link_pos;
	_model->positionInWorld(link_pos, _data->_link_name, _data->_transform_in_link.translation());
	for (uint pt_ind=0; pt_ind < point_list.size(); ++pt_ind) {
		_data->_force += force_list[pt_ind];
		rel_pos = point_list[pt_ind] - link_pos;
		//unfortunately, it is defined in global frame
		_data->_moment += rel_pos.cross(force_list[pt_ind]);
	}

	if (_first_iteration)
	{
		_previous_force = _data->_force;
		_previous_torque = _data->_moment;
		_first_iteration=false;
	}

	// Discretly remove spikes if required
	if (_remove_spike)
	{
		if(fabs(_data->_force[0]-_previous_force[0]) >= _force_threshold)
		{
			_data->_force = _previous_force;
			_data->_moment = _previous_torque;
		}
	}

	_previous_force = _data->_force;
	_previous_torque = _data->_moment;
}

// get force
Eigen::Vector3d ForceSensorSim::getForce() const {
	return _data->_force;
}

Eigen::Vector3d ForceSensorSim::getForceLocalFrame() const {
	Eigen::Matrix3d R_base_sensor;
	_model->rotationInWorld(R_base_sensor, _data->_link_name);
	R_base_sensor = R_base_sensor * _data->_transform_in_link.rotation();

	return R_base_sensor.transpose() * _data->_force;
}

// get moment
Eigen::Vector3d ForceSensorSim::getMoment() const {
	return _data->_moment;
}

Eigen::Vector3d ForceSensorSim::getMomentLocalFrame() const {
	Eigen::Matrix3d R_base_sensor;
	_model->rotationInWorld(R_base_sensor, _data->_link_name);
	R_base_sensor = R_base_sensor * _data->_transform_in_link.rotation();

	return R_base_sensor.transpose() * _data->_moment;
}

// get force and moment
Eigen::VectorXd ForceSensorSim::getForceMoment() const {
	Eigen::VectorXd ret_force_moment = Eigen::VectorXd::Zero(6);

	ret_force_moment.head(3) = _data->_force;
	ret_force_moment.tail(3) = _data->_moment;

	return ret_force_moment;
}

Eigen::VectorXd ForceSensorSim::getForceMomentLocalFrame() const {
	Eigen::VectorXd ret_force_moment = Eigen::VectorXd::Zero(6);

	Eigen::Matrix3d R_base_sensor;
	_model->rotationInWorld(R_base_sensor, _data->_link_name);
	R_base_sensor = R_base_sensor * _data->_transform_in_link.rotation();

	ret_force_moment.head(3) = R_base_sensor.transpose() * _data->_force;
	ret_force_moment.tail(3) = R_base_sensor.transpose() * _data->_moment;

	return ret_force_moment;
}

void ForceSensorSim::enableSpikeRemoval(const double force_threshold)
{
	_remove_spike = true;
	_force_threshold = force_threshold;
}

} // namespace Sai2Simulation
