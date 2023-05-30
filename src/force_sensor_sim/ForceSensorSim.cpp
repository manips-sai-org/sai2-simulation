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
void ForceSensorSim::update(const cDynamicWorld* dyn_world) {
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

	Eigen::VectorXd force_raw = _data->_force;
	Eigen::VectorXd moment_raw = _data->_moment;

	if (_first_iteration)
	{
		_previous_force = force_raw;
		_previous_torque = moment_raw;
		_first_iteration=false;
	}

	// Discretly remove spikes if required
	if (_remove_spike)
	{
		std::cout << "F_diff" << fabs(force_raw[0]-_previous_force[0]) << std::endl;

		if(fabs(force_raw[0]-_previous_force[0]) >= _force_threshold)
		{
			force_raw = _previous_force;
			moment_raw = _previous_torque;
		}
	}

	_previous_force = _data->_force;
	_previous_torque = _data->_moment;
}

// get force
void ForceSensorSim::getForce(Eigen::Vector3d& ret_force) {
	ret_force = _data->_force;
}

void ForceSensorSim::getForceLocalFrame(Eigen::Vector3d& ret_force) {
	Eigen::Matrix3d R_base_sensor;
	_model->rotationInWorld(R_base_sensor, _data->_link_name);
	R_base_sensor = R_base_sensor * _data->_transform_in_link.rotation();

	ret_force = R_base_sensor.transpose() * _data->_force;
}

// get moment
void ForceSensorSim::getMoment(Eigen::Vector3d& ret_moment) {
	ret_moment = _data->_moment;
}

void ForceSensorSim::getMomentLocalFrame(Eigen::Vector3d& ret_moment) {
	Eigen::Matrix3d R_base_sensor;
	_model->rotationInWorld(R_base_sensor, _data->_link_name);
	R_base_sensor = R_base_sensor * _data->_transform_in_link.rotation();

	ret_moment = R_base_sensor.transpose() * _data->_moment;
}

// get force and moment
void ForceSensorSim::getForceMoment(Eigen::VectorXd& ret_force_moment) {
	ret_force_moment.setZero(6);

	ret_force_moment.head(3) = _data->_force;
	ret_force_moment.tail(3) = _data->_moment;
}

void ForceSensorSim::getForceMomentLocalFrame(Eigen::VectorXd& ret_force_moment) {
	ret_force_moment.setZero(6);

	Eigen::Matrix3d R_base_sensor;
	_model->rotationInWorld(R_base_sensor, _data->_link_name);
	R_base_sensor = R_base_sensor * _data->_transform_in_link.rotation();

	ret_force_moment.head(3) = R_base_sensor.transpose() * _data->_force;
	ret_force_moment.tail(3) = R_base_sensor.transpose() * _data->_moment;
}

void ForceSensorSim::enableSpikeRemoval(const double force_threshold)
{
	_remove_spike = true;
	_force_threshold = force_threshold;
}

} // namespace Sai2Simulation
