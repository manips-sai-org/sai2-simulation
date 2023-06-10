//===========================================================================
/*
    This file is part of the Newton dynamics library.
    Copyright (C) 2011, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1048 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "chai3d/CDynamicWorld.h"
//---------------------------------------------------------------------------
using namespace chai3d;
using namespace std; 
//---------------------------------------------------------------------------

//==========================================================================
/*!
      Constructor of cDynamicWorld.

      \param    a_parentWorld  Pointer to parent CHAI3D world.
      \return   Return pointer to new cODEWorld instance.
*/
//===========================================================================
cDynamicWorld::cDynamicWorld(chai3d::cWorld* a_parentWorld)
{
    // set parent CHAI3D world
    m_parentWorld = a_parentWorld;

    // create dynamic world
    m_dynWorld = new cDynWorld();

    // set default collision step
    m_dynWorld->collisionStep(1.0/200.0);

    // set backup time
    m_dynWorld->backupLimit(0.05);

    // set default gravity
    m_dynWorld->gravity.set(0.0, 0.0, -9.81);
    m_time = 0.0;
}


//===========================================================================
/*!
      Destructor of cDynamicWorld.
*/
//===========================================================================
cDynamicWorld::~cDynamicWorld()
{
    delete m_dynWorld;
}


//===========================================================================
/*!
     Render world in OpenGL.

     \param    a_options  Rendering options.
*/
//===========================================================================
void cDynamicWorld::render(chai3d::cRenderOptions& a_options)
{
    list<cDynamicBase*>::iterator i;
    for(i = m_dynamicObjects.begin(); i != m_dynamicObjects.end(); ++i)
    {
        cDynamicBase* nextItem = *i;
        nextItem->renderSceneGraph(a_options);
        nextItem->m_dynamicContacts->render(a_options);
    }
}


//==========================================================================
/*!
    Extends cDynWorld to support collision detection.

    \param    a_segmentPointA  Start point of segment.  
    \param    a_segmentPointB  End point of segment.
    \param    a_recorder  Stores all collision events.
    \param    a_settings  Contains collision settings information.  
    \return   Return \b true if a collision has occurred.
*/
//===========================================================================
bool cDynamicWorld::computeOtherCollisionDetection(chai3d::cVector3d& a_segmentPointA,
                                               chai3d::cVector3d& a_segmentPointB,
                                               chai3d::cCollisionRecorder& a_recorder,
                                               chai3d::cCollisionSettings& a_settings)
{
    // check each Newton robot
    bool result = false;
    
    list<cDynamicBase*>::iterator i;
    for(i = m_dynamicObjects.begin(); i != m_dynamicObjects.end(); ++i)
    {
        cDynamicBase* nextItem = *i;
        bool collide = nextItem->computeCollisionDetection(a_segmentPointA, 
                                                           a_segmentPointB, 
                                                           a_recorder,
                                                           a_settings);
        if (collide) { result = true; }
    }

    // return result
    return (result);
}


//===========================================================================
/*!
    Compute globalPos and globalRot given the localPos and localRot
    of this object and its parents.  Optionally propagates to children.

    If \e a_frameOnly is set to \b false, additional global positions such as
    vertex positions are computed (which can be time-consuming).

    \param    a_frameOnly  If \b true then only the global frame is computed.
*/
//===========================================================================
void cDynamicWorld::updateGlobalPositions(const bool a_frameOnly)
{
    list<cDynamicBase*>::iterator i;
    for(i = m_dynamicObjects.begin(); i != m_dynamicObjects.end(); ++i)
    {
        cDynamicBase* nextItem = *i;
        nextItem->computeGlobalPositions(a_frameOnly,
                                         m_globalPos,
                                         m_globalRot);
    }
};


//===========================================================================
/*!
      Compute simulation for a_time time interval.

      \param    a_interval  Time increment.
*/
//===========================================================================
void cDynamicWorld::updateDynamics(double a_interval)
{
    // compute next time
   double time = m_time + a_interval;

    // clear contact points for non contacting objects
    list<cDynamicBase*>::iterator i;
    for(i = m_dynamicObjects.begin(); i != m_dynamicObjects.end(); ++i)
    {
        cDynamicBase* nextItem = *i;

        cDynContactList* contactList = nextItem->m_dynBaseNode->contact();
        if (contactList->n() == 0)
        {
            nextItem->m_dynamicContacts->clear();
        }
    }

    // integrate world
    m_dynWorld->update(time);
    m_time = m_dynWorld->time();
   
    // update CHAI3D positions for of all object
    copyDynamicFrameToGraphicFrame();
}


//===========================================================================
/*!
      Update position and orientation from dynamics models to graphical models.
*/
//===========================================================================
void cDynamicWorld::copyDynamicFrameToGraphicFrame()
{
    list<cDynamicBase*>::iterator i;

    double time = m_dynWorld->time();
    for(i = m_dynamicObjects.begin(); i != m_dynamicObjects.end(); ++i)
    {
        cDynamicBase* nextItem = *i;
        nextItem->copyDynamicFrameToGraphicFrame(time);
    }
}


//===========================================================================
/*!
      Create a new base object in the world.
*/
//===========================================================================
cDynamicBase* cDynamicWorld::newBaseObject(const cVector3d& a_pos, const cMatrix3d& a_rot)
{
    // create new base object
    cDynamicBase* newBaseObject = new cDynamicBase(this);

    // add it to the dynamic world
    m_dynWorld->frame.push();
    m_dynWorld->frame.translate(a_pos.x(), a_pos.y(), a_pos.z());

    cDynVector3 axis;
    double angle = 0.0;
    cVector3d caxis(0,0,0);
    a_rot.toAxisAngle(caxis, angle);
    axis.set(caxis.x(), caxis.y(), caxis.z());
    m_dynWorld->frame.rotate(axis, angle);
    cDynBaseNode* baseNode = m_dynWorld->insert(newBaseObject->m_dynBaseObject,"");
    m_dynWorld->frame.pop();

    newBaseObject->m_dynBaseNode = baseNode;   
    baseNode->contact()->callback( newBaseObject->m_dynamicContacts);
    baseNode->status(CDYN_ACTIVE);
    baseNode->integrationStep(1.0/10000.0);
    baseNode->integrator(CDYN_EULER_HEUN);
    
    // return pointer to new object
    return (newBaseObject);
}


//===========================================================================
/*!
    Get pointer to base node by passing string name.

    \param  a_name  Name of base node.

    \return Return pointer to base node if found, __NULL__ otherwise
*/
//===========================================================================
cDynamicBase* cDynamicWorld::getBaseNode(std::string a_name)
{
    list<cDynamicBase*>::iterator i;
    for(i = m_dynamicObjects.begin(); i != m_dynamicObjects.end(); ++i)
    {
        cDynamicBase* nextBaseNode = *i;
        if (nextBaseNode->m_name == a_name)
        {
            // return base node
            return (nextBaseNode);
        }
    }

    // no base node found, return NULL
    return (NULL);
}

//===========================================================================
/*!
    Get a list of contacts (location and force in world) for a given robot at a given list

    \param  contact_points  Return vector of contact locations in world
    \param  contact_forces  Return vector of contact forces in world
    \param  robot_name  Name of the object
    \param  link_name  Name of the link
*/
//===========================================================================
void cDynamicWorld::getContactList(std::vector<Eigen::Vector3d>& contact_points,
                                   std::vector<Eigen::Vector3d>& contact_forces,
                                   const ::std::string& robot_name,
                                   const std::string& link_name) const {
	contact_points.clear();
	contact_forces.clear();
	Eigen::Vector3d current_position = Eigen::Vector3d::Zero();
	Eigen::Vector3d current_force = Eigen::Vector3d::Zero();

    for(auto i = m_dynamicObjects.begin(); i != m_dynamicObjects.end(); ++i)
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