/*
 * Sai2Simulation.cpp
 *
 *  Created on: Dec 27, 2016
 *      Author: Shameek Ganguly
 *  Update: ported to sai2-simulation project on Nov 17, 2017
 *		By: Shameek Ganguly
 */

#include "Sai2Simulation.h"
#include "dynamics3d.h"
#include "parser/URDFToDynamics3d.h"
#include <math/CMatrix3d.h> // chai math library
#include <stdexcept>
#include <map>

#include <iostream>

using namespace std;
using namespace chai3d;

namespace Simulation {

// ctor
Sai2Simulation::Sai2Simulation(const std::string& path_to_world_file,
	            		bool verbose)
: _world(NULL)
{
	// create a dynamics world
	_world = new cDynamicWorld(NULL);
	assert(_world);

	URDFToDynamics3dWorld(path_to_world_file, _world, _dyn_object_base_pos, _dyn_object_base_rot, verbose);

	// enable dynamics for all robots in this world
	// TODO: consider pushing up to the API?
	for (auto robot: _world->m_dynamicObjects) {
		robot->enableDynamics(true);
	}

	// create a dof map
	for (auto robot: _world->m_dynamicObjects) {
		uint dof = 0;
		uint num_sph_joint = 0;\
		for (auto it: robot->m_dynamicJoints) {
			dof += (it->getJointType() == DYN_JOINT_SPHERICAL)? 3: 1;
			num_sph_joint += (it->getJointType() == DYN_JOINT_SPHERICAL)? 1: 0;
		}
		_dof_map[robot->m_name] = dof;
		_q_size_map[robot->m_name] = dof + num_sph_joint;
	}
}

// dtor
Sai2Simulation::~Sai2Simulation() {
	//TODO: ensure deallocation within cDynamicWorld class
	delete _world;
	_world = NULL;
}

// get dof
unsigned int Sai2Simulation::dof(const std::string& robot_name) const {
	assert(_dof_map.find(robot_name) != _dof_map.end());
	return _dof_map.at(robot_name);
}

// set joint positions
void Sai2Simulation::setJointPositions(const std::string& robot_name,
											const Eigen::VectorXd& q) {
	auto robot = _world->getBaseNode(robot_name);
	assert(robot);
	assert(q.size() == _q_size_map.at(robot_name));
	uint q_ind_counter = 0;
	uint sph_joint_counter = 0;
	uint dofs = dof(robot_name);
	chai3d::cQuaternion sph_quat;
	for (unsigned int i = 0; i < robot->m_dynamicJoints.size(); ++i) {
		if (robot->m_dynamicJoints[i]->getJointType() == DYN_JOINT_SPHERICAL) {
			sph_quat.w = q[dofs + sph_joint_counter];
			sph_quat.x = q[q_ind_counter];
			sph_quat.y = q[q_ind_counter+1];
			sph_quat.z = q[q_ind_counter+2];
			robot->m_dynamicJoints[i]->setPosSpherical(sph_quat);
			sph_joint_counter++;
			q_ind_counter += 3;
		} else {
			robot->m_dynamicJoints[i]->setPos(q[q_ind_counter++]);
		}
	}
}

// read joint positions
void Sai2Simulation::getJointPositions(const std::string& robot_name,
												Eigen::VectorXd& q_ret) const {
	auto robot = _world->getBaseNode(robot_name);
	assert(robot);
	q_ret.setZero(_q_size_map.at(robot_name));
	uint q_ind_counter = 0;
	uint sph_joint_counter = 0;
	uint dofs = dof(robot_name);
	chai3d::cQuaternion sph_quat;
	for (unsigned int i = 0; i < robot->m_dynamicJoints.size(); ++i) {
		if (robot->m_dynamicJoints[i]->getJointType() == DYN_JOINT_SPHERICAL) {
			sph_quat = robot->m_dynamicJoints[i]->getPosSpherical();
			q_ret[dofs + sph_joint_counter] = sph_quat.w;
			q_ret[q_ind_counter] = sph_quat.x;
			q_ret[q_ind_counter+1] = sph_quat.y;
			q_ret[q_ind_counter+2] = sph_quat.z;
			sph_joint_counter++;
			q_ind_counter += 3;
		} else {
			q_ret[q_ind_counter++] = robot->m_dynamicJoints[i]->getPos();
		}
	}
}

// read object pose
void Sai2Simulation::getObjectPosition(const std::string& object_name,
              Eigen::Vector3d& pos,
              Eigen::Quaterniond& ori) const {
	auto object = _world->getBaseNode(object_name);
	assert(object);
	pos << object->m_dynamicJoints[0]->getPos(), object->m_dynamicJoints[1]->getPos(), object->m_dynamicJoints[2]->getPos();
	pos = _dyn_object_base_rot.at(object_name).toRotationMatrix() * pos;
	pos += _dyn_object_base_pos.at(object_name);

	chai3d::cQuaternion quat = object->m_dynamicJoints[3]->getPosSpherical();
	ori = _dyn_object_base_rot.at(object_name) * Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
}

// set object pose
void Sai2Simulation::setObjectPosition(const std::string& object_name,
              const Eigen::Vector3d& pos,
              const Eigen::Quaterniond& ori) const {
	auto object = _world->getBaseNode(object_name);
	assert(object);

	Eigen::Vector3d object_pos_local = _dyn_object_base_rot.at(object_name).toRotationMatrix().transpose() * (pos - _dyn_object_base_pos.at(object_name));
	Eigen::Quaterniond object_rot_local = _dyn_object_base_rot.at(object_name).inverse() * ori;
	for(int i=0 ; i<3 ; i++)
	{
		object->m_dynamicJoints[i]->setPos(object_pos_local(i));
	}

	chai3d::cQuaternion quat = chai3d::cQuaternion(object_rot_local.w(), object_rot_local.x(), object_rot_local.y(), object_rot_local.z());	
	object->m_dynamicJoints[3]->setPosSpherical(quat);
}

// set joint position for a single joint
void Sai2Simulation::setJointPosition(const std::string& robot_name,
											unsigned int joint_id,
											double position) {
	auto robot = _world->getBaseNode(robot_name);
	assert(robot);
	assert(joint_id < robot->m_dynamicJoints.size());
	assert(robot->m_dynamicJoints[joint_id]->getJointType() != DYN_JOINT_SPHERICAL);
	robot->m_dynamicJoints[joint_id]->setPos(position);
}

// set joint velocities
void Sai2Simulation::setJointVelocities(const std::string& robot_name,
												const Eigen::VectorXd& dq) {
	auto robot = _world->getBaseNode(robot_name);
	assert(robot);
	assert(dq.size() == _dof_map.at(robot_name));
	uint q_ind_counter = 0;
	chai3d::cVector3d sph_vel;
	for (unsigned int i = 0; i < robot->m_dynamicJoints.size(); ++i) {
		if (robot->m_dynamicJoints[i]->getJointType() == DYN_JOINT_SPHERICAL) {
			sph_vel = Eigen::Vector3d(dq[q_ind_counter], dq[q_ind_counter+1], dq[q_ind_counter+2]);
			robot->m_dynamicJoints[i]->setVelSpherical(sph_vel);
			q_ind_counter += 3;
		} else {
			robot->m_dynamicJoints[i]->setVel(dq[q_ind_counter++]);
		}
	}
}

// set joint velocity for a single joint
void Sai2Simulation::setJointVelocity(const std::string& robot_name,
											unsigned int joint_id,
											double velocity) {
	auto robot = _world->getBaseNode(robot_name);
	assert(robot);
	assert(joint_id < robot->m_dynamicJoints.size());
	assert(robot->m_dynamicJoints[joint_id]->getJointType() != DYN_JOINT_SPHERICAL);
	robot->m_dynamicJoints[joint_id]->setVel(velocity);
}

// read joint velocities
void Sai2Simulation::getJointVelocities(const std::string& robot_name,
												Eigen::VectorXd& dq_ret) const {
	auto robot = _world->getBaseNode(robot_name);
	assert(robot);
	dq_ret.setZero(_dof_map.at(robot_name));
	uint q_ind_counter = 0;
	uint dofs = dof(robot_name);
	chai3d::cVector3d sph_vel;
	for (unsigned int i = 0; i < robot->m_dynamicJoints.size(); ++i) {
		if (robot->m_dynamicJoints[i]->getJointType() == DYN_JOINT_SPHERICAL) {
			sph_vel = robot->m_dynamicJoints[i]->getVelSpherical();
			dq_ret[q_ind_counter] = sph_vel.x();
			dq_ret[q_ind_counter+1] = sph_vel.y();
			dq_ret[q_ind_counter+2] = sph_vel.z();
			q_ind_counter += 3;
		} else {
			dq_ret[q_ind_counter++] = robot->m_dynamicJoints[i]->getVel();
		}
	}
}

// read object velocities
void Sai2Simulation::getObjectVelocity(const std::string& object_name,
              Eigen::Vector3d& lin_vel,
              Eigen::Vector3d& ang_vel) const {
	auto object = _world->getBaseNode(object_name);
	assert(object);
	lin_vel << object->m_dynamicJoints[0]->getVel(), object->m_dynamicJoints[1]->getVel(), object->m_dynamicJoints[2]->getVel();
	ang_vel = object->m_dynamicJoints[3]->getVelSpherical().eigen();
}

// set joint torques
void Sai2Simulation::setJointTorques(const std::string& robot_name,
											const Eigen::VectorXd& tau) {
	auto robot = _world->getBaseNode(robot_name);
	assert(robot);
	assert(tau.size() == _dof_map.at(robot_name));
	uint q_ind_counter = 0;
	chai3d::cVector3d sph_tau;
	for (unsigned int i = 0; i < robot->m_dynamicJoints.size(); ++i) {
		if (robot->m_dynamicJoints[i]->getJointType() == DYN_JOINT_SPHERICAL) {
			sph_tau = Eigen::Vector3d(tau[q_ind_counter], tau[q_ind_counter+1], tau[q_ind_counter+2]);
			robot->m_dynamicJoints[i]->setTorque(sph_tau);
			q_ind_counter += 3;
		} else {
			robot->m_dynamicJoints[i]->setForce(tau[q_ind_counter++]);
		}
	}
}

// set joint torque for a single joint
void Sai2Simulation::setJointTorque(const std::string& robot_name,
											unsigned int joint_id,
											double tau) {
	auto robot = _world->getBaseNode(robot_name);
	assert(robot);
	assert(joint_id < robot->m_dynamicJoints.size());
	assert(robot->m_dynamicJoints[joint_id]->getJointType() != DYN_JOINT_SPHERICAL);
	robot->m_dynamicJoints[joint_id]->setForce(tau);
	// NOTE: we don't support spherical joints currently
	// but in cDynamicJoint, spherical joints have a different function
	// for setting torque: void setTorque(chai3d::cVector3d&)
}

// read joint torques
// NOTE: currently unsupported due to lack of support in dynamics3d
void Sai2Simulation::getJointTorques(const std::string& robot_name,
											Eigen::VectorXd& tau_ret) const {
	cerr << "Unsupported function Sai2Simulation::getJointTorques" << endl;
	abort();
}

// read joint accelerations
void Sai2Simulation::getJointAccelerations(const std::string& robot_name,
												Eigen::VectorXd& ddq_ret) const {
	auto robot = _world->getBaseNode(robot_name);
	assert(robot);
	ddq_ret.setZero(_dof_map.at(robot_name));
	uint q_ind_counter = 0;
	chai3d::cVector3d sph_acc;
	for (unsigned int i = 0; i < robot->m_dynamicJoints.size(); ++i) {
		if (robot->m_dynamicJoints[i]->getJointType() == DYN_JOINT_SPHERICAL) {
			sph_acc = robot->m_dynamicJoints[i]->getAccelSpherical();
			ddq_ret[q_ind_counter] = sph_acc.x();
			ddq_ret[q_ind_counter+1] = sph_acc.y();
			ddq_ret[q_ind_counter+2] = sph_acc.z();
			q_ind_counter += 3;
		} else {
			ddq_ret[q_ind_counter++] = robot->m_dynamicJoints[i]->getAccel();
		}
	}
}

// integrate ahead
void Sai2Simulation::integrate(double timestep) {
	_world->updateDynamics(timestep);
}

void Sai2Simulation::showContactInfo()
{

    list<cDynamicBase*>::iterator i;
    for(i = _world->m_dynamicObjects.begin(); i != _world->m_dynamicObjects.end(); ++i)
    {
        cDynamicBase* object = *i;
    	int num_contacts = object->m_dynamicContacts->getNumContacts();
    	// consider only contacting objects
        if(num_contacts > 0)
        {
	    	std::cout << "object name : " << object->m_name << std::endl;
	    	std::cout << "num contacts : " << num_contacts << std::endl;
	    	for(int k=0; k < num_contacts; k++)
	    	{
	    		cDynamicContact* contact = object->m_dynamicContacts->getContact(k);
		    	std::cout << "contact " << k << " at link : " << contact->m_dynamicLink->m_name << std::endl;
		    	std::cout << "contact position : " << contact->m_globalPos << std::endl;
		    	std::cout << "contact normal : " << contact->m_globalNormal << std::endl;
		    	std::cout << "contact normal force : " << contact->m_globalNormalForce << std::endl;
		    	std::cout << "contact friction force : " << contact->m_globalFrictionForce << std::endl;
		    	std::cout << "contact force magnitude : " << contact->m_normalForceMagnitude << std::endl;
		    	std::cout << "time : " << contact->m_time << std::endl;
	    	}
	    	std::cout << std::endl;
        }
    }
}

void Sai2Simulation::getContactList(std::vector<Eigen::Vector3d>& contact_points, std::vector<Eigen::Vector3d>& contact_forces, 
	const::std::string& robot_name, const std::string& link_name) 
{
	contact_points.clear();
	contact_forces.clear();
	Eigen::Vector3d current_position = Eigen::Vector3d::Zero();
	Eigen::Vector3d current_force = Eigen::Vector3d::Zero();

	list<cDynamicBase*>::iterator i;
    for(i = _world->m_dynamicObjects.begin(); i != _world->m_dynamicObjects.end(); ++i)
    {
    	cDynamicBase* object = *i;
    	// only consider the desired object
    	if(object->m_name != robot_name)
    	{
    		continue;
    	}
    	int num_contacts = object->m_dynamicContacts->getNumContacts();
    	// only consider if the oject is contacting something
        if(num_contacts > 0)
        {
        	for(int k=0; k < num_contacts; k++)
	    	{
	        	cDynamicContact* contact = object->m_dynamicContacts->getContact(k);
	        	// only consider contacts at the desired link
                if(contact==NULL || contact->m_dynamicLink->m_name != link_name)
	        	{
	        		continue;
	        	}
	        	// copy chai3d vector to eigen vector
	        	for(int l=0; l<3; l++)
	        	{
		        	current_position(l) = contact->m_globalPos(l);
		        	// the friction force is inverted for some reason. Need to substract it to get a coherent result.
		        	current_force(l) = contact->m_globalNormalForce(l) - contact->m_globalFrictionForce(l);
	        	}
	        	// reverse the sign to get the list of forces applied to the considered object
	        	contact_points.push_back(current_position);
	        	contact_forces.push_back(-current_force);
	        }
        }


    }
}

// set restitution co-efficients: for all objects
void Sai2Simulation::setCollisionRestitution(double restitution) {
	// for all dynamic base
	for (cDynamicBase* base: _world->m_dynamicObjects) {
		// for all links in this base
		for (cDynamicLink* link: base->m_dynamicLinks) {
			cDynamicMaterial* mat = link->getDynamicMaterial();
			mat->setEpsilon(restitution);
		}
	}
}

// set restitution co-efficients: for a named object
void Sai2Simulation::setCollisionRestitution(const std::string& object_name,
		                                        double restitution) {
	setCollisionRestitution(object_name, "object_link", restitution);
}

// set restitution co-efficients: for a named link
void Sai2Simulation::setCollisionRestitution(const std::string& robot_name,
		                                        const std::string& link_name,
		                                        double restitution) {
	auto robot = _world->getBaseNode(robot_name);
	assert(robot);
	auto link = robot->getLink(link_name);
	assert(link);
	cDynamicMaterial* mat = link->getDynamicMaterial();
	mat->setEpsilon(restitution);
}

double Sai2Simulation::getCollisionRestitution(const std::string& object_name) const {
	return getCollisionRestitution(object_name, "object_link");
}


// get co-efficient of restitution: for a named robot and link
double Sai2Simulation::getCollisionRestitution(const std::string& robot_name,
		                                        const std::string& link_name) const {
	auto robot = _world->getBaseNode(robot_name);
	assert(robot);
	auto link = robot->getLink(link_name);
	assert(link);
	cDynamicMaterial* mat = link->getDynamicMaterial();
	return mat->getEpsilon();
}



// set co-efficient of static friction: for all objects
void Sai2Simulation::setCoeffFrictionStatic(double static_friction) {
	// for all dynamic base
	for (cDynamicBase* base: _world->m_dynamicObjects) {
		// for all links in this base
		for (cDynamicLink* link: base->m_dynamicLinks) {
			cDynamicMaterial* mat = link->getDynamicMaterial();
			mat->setStaticFriction(static_friction);
		}
	}
}

void Sai2Simulation::setCoeffFrictionStatic(const std::string& object_name,
                                   			double static_friction) {
	setCoeffFrictionStatic(object_name, "object_link", static_friction);
}

// set co-efficient of static friction: for a named robot and link
void Sai2Simulation::setCoeffFrictionStatic(const std::string& robot_name,
                                   			const std::string& link_name,
                                   			double static_friction) {
	auto robot = _world->getBaseNode(robot_name);
	assert(robot);
	auto link = robot->getLink(link_name);
	assert(link);
	cDynamicMaterial* mat = link->getDynamicMaterial();
	mat->setStaticFriction(static_friction);
}

double Sai2Simulation::getCoeffFrictionStatic(const std::string& object_name) const {
	return getCoeffFrictionStatic(object_name, "object_link");
}

// get co-efficient of static friction: for a named robot and link
double Sai2Simulation::getCoeffFrictionStatic(const std::string& robot_name,
												const std::string& link_name) const {
	auto robot = _world->getBaseNode(robot_name);
	assert(robot);
	auto link = robot->getLink(link_name);
	assert(link);
	cDynamicMaterial* mat = link->getDynamicMaterial();
	return mat->getStaticFriction();
}

// set co-efficient of dynamic friction: for all object
void Sai2Simulation::setCoeffFrictionDynamic(double dynamic_friction) {
	// for all dynamic base
	for (cDynamicBase* base: _world->m_dynamicObjects) {
		// for all links in this base
		for (cDynamicLink* link: base->m_dynamicLinks) {
			cDynamicMaterial* mat = link->getDynamicMaterial();
			mat->setDynamicFriction(dynamic_friction);
		}
	}
}

void Sai2Simulation::setCoeffFrictionDynamic(const std::string& object_name,
												double dynamic_friction) {
	setCoeffFrictionDynamic(object_name, "object_link", dynamic_friction);
}

// set co-efficient of dynamic friction: for a named robot and link
void Sai2Simulation::setCoeffFrictionDynamic(const std::string& robot_name,
												const std::string& link_name,
												double dynamic_friction) {
	auto robot = _world->getBaseNode(robot_name);
	assert(robot);
	auto link = robot->getLink(link_name);
	assert(link);
	cDynamicMaterial* mat = link->getDynamicMaterial();
	mat->setDynamicFriction(dynamic_friction);
}

double Sai2Simulation::getCoeffFrictionDynamic(const std::string& object_name) const {
	return getCoeffFrictionDynamic(object_name, "object_link");
}


// get co-efficient of dynamic friction: for a named robot and link
double Sai2Simulation::getCoeffFrictionDynamic(const std::string& robot_name,
		                                        const std::string& link_name) const {
	auto robot = _world->getBaseNode(robot_name);
	assert(robot);
	auto link = robot->getLink(link_name);
	assert(link);
	cDynamicMaterial* mat = link->getDynamicMaterial();
	return mat->getDynamicFriction();
}

// get pose of robot base in the world frame
Eigen::Affine3d Sai2Simulation::getRobotBaseTransform(const std::string& robot_name) const {
	Eigen::Affine3d gToRobotBase;
	const auto base = _world->getBaseNode(robot_name);
	gToRobotBase.translation() = base->getLocalPos().eigen();
	gToRobotBase.linear() = base->getLocalRot().eigen();
	return gToRobotBase;
}

}