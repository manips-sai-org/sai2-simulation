//===========================================================================
/*
    This file is part of the Newton dynamics library.
    Copyright (C) 2011, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1048 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "chai3d/CDynamicContact.h"
//---------------------------------------------------------------------------
#include "chai3d/CDynamicLink.h"
//---------------------------------------------------------------------------
using namespace chai3d; 
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cDynContactList.

    \fn       cDynamicContactList::cDynamicContactList()
*/
//===========================================================================
cDynamicContactList::cDynamicContactList()
{
    m_contacts.clear();
    m_simTime = 0.0;
    m_numContacts = 0;

    m_displayColor.setRed();
    m_displayRadius = 0.01;
    m_displayScale = 0.1;
    m_show = false;
}


//===========================================================================
/*!
    Destructor of cDynamicContactList.
*/
//===========================================================================
cDynamicContactList::~cDynamicContactList()
{

}


//===========================================================================
/*!
    Callback from Dynamics Engine.
*/
//===========================================================================
void cDynamicContactList::handle()
{
    // if new time, then clear previous contacts
    double simTime = time();
 
    if (m_simTime != simTime)
    {
        m_simTime = simTime;
        m_numContacts = 0;
    }

    if (m_numContacts == m_contacts.size())
    {
        cDynamicContact contact;
        m_contacts.push_back(contact);
    }

    // extract information related to collision from dynamics engine
    cDynVector3 tmpVect;
    double tmpValue;
  
    // retrieve position of contact in global world coordinates.
    point(tmpVect);
    m_contacts[m_numContacts].m_globalPos.set(tmpVect[0], tmpVect[1], tmpVect[2]);

    // retrieve contact normal in global world coordinates.
    normal(tmpVect);
    m_contacts[m_numContacts].m_globalNormal.set(tmpVect[0], tmpVect[1], tmpVect[2]);

    // retrieve force magnitude at point of contact
    tmpValue = force();
    m_contacts[m_numContacts].m_normalForceMagnitude = tmpValue;
    m_contacts[m_numContacts].m_globalNormalForce.set(tmpValue*tmpVect[0], tmpValue*tmpVect[1], tmpValue*tmpVect[2]);

    // retrieve friction force at point of contact
    this->forceFriction(tmpVect);
    m_contacts[m_numContacts].m_globalFrictionForce.set(tmpVect[0], tmpVect[1], tmpVect[2]);
    
    // retrieve pointer to link
    cDynObject *obj = (cDynObject*) node();
    m_contacts[m_numContacts].m_dynamicLink = (cDynamicLink*)obj->data();

    // store time
    m_contacts[m_numContacts].m_time = simTime;

    // increase the number of contacts by one
    m_numContacts++;
}


//===========================================================================
/*!
    Return pointer to desired contact.

    \param  a_index  Index number of contact point.
    \return Return pointer to contact point.
*/
//===========================================================================
cDynamicContact* cDynamicContactList::getContact(unsigned int a_index)
{
    if ((int)(a_index) > (m_numContacts-1))
    {
        return (NULL);
    }
    else
    {
        return (&(m_contacts[a_index]));
    }
}


//===========================================================================
/*!
    Render contact points.

    \param  a_options  Rendering options
*/
//===========================================================================
void cDynamicContactList::render(cRenderOptions& a_options)
{
    if (!m_show) { return; }

    // render contact points
    m_displayColor.render();
    for (int i=0; i<m_numContacts; i++)
    {
        cDynamicContact* contact = &(m_contacts[i]);
        
        cVector3d pos0 = contact->m_globalPos;
        cVector3d pos1, v;
        contact->m_globalNormal.mulr(m_displayScale * contact->m_normalForceMagnitude, v);
        pos0.addr(v, pos1);

        if (m_displayRadius > 0.0)
        {
            glPushMatrix();
            glTranslated(pos0.x(), pos0.y(), pos0.z());
            cDrawSphere(m_displayRadius, 10, 10);
            glPopMatrix();
        }
       
        glDisable (GL_LIGHTING);
            glBegin(GL_LINES);
            glVertex3d(pos0.x(), pos0.y(), pos0.z());
            glVertex3d(pos1.x(), pos1.y(), pos1.z());
            glEnd();
        glEnable(GL_LIGHTING);
    }
}
