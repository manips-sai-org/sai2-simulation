//===========================================================================
/*
    This file is part of the Newton dynamics library.
    Copyright (C) 2011, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1088 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynamicContactH
#define CDynamicContactH
//---------------------------------------------------------------------------
#include "chai3d/CDynInternal.h"
#include "chai3d.h"
//---------------------------------------------------------------------------
class cDynamicLink;
//---------------------------------------------------------------------------


//===========================================================================
/*!
    \file       CDynamicContact.h

    \brief 
    <b> Dynamics Module </b> \n 
    Dynamic Contact.
*/
//===========================================================================


//===========================================================================
/*!
    \struct     cDynamicContact
    \ingroup    Dynamics

    \brief      
    cDynamicContact contains information related to a contact point.
*/
//===========================================================================
struct cDynamicContact
{
    double m_normalForceMagnitude;
    chai3d::cVector3d m_globalPos;
    chai3d::cVector3d m_globalNormal;
    chai3d::cVector3d m_globalNormalForce;
    chai3d::cVector3d m_globalFrictionForce;
    cDynamicLink* m_dynamicLink;
    double m_time;
};


//===========================================================================
/*!
    \class      cDynamicContactList
    \ingroup    Dynamics

    \brief      
    cDynContactList handle a list of contact points between the links
    of a cDynBase object and other objects in the envirionment.
*/
//===========================================================================
class cDynamicContactList : public cDynContactEvent
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

public:

    //! Constructor of cDynamicContactList.
    cDynamicContactList();

    //! Destructor of cDynamicContactList.
    virtual ~cDynamicContactList();


    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------

public:

    //! Get number of contacts.
    inline int getNumContacts() { return (m_numContacts); } 

    //! Get i-th contact.
    cDynamicContact* getContact(unsigned int a_index);

    //! Clear contact point list.
    inline void clear() { m_numContacts = 0; }

    //! Render object in OpenGL.
    void render(chai3d::cRenderOptions& a_options);


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //-----------------------------------------------------------------------

public:

    //! Color used to render forces.
    chai3d::cColorf m_displayColor;

    //! Radius used to render contact points.
    double m_displayRadius;

    //! Scale factor used for rendering the length of the contact force vectors.
    double m_displayScale;

    //! If __true__ then contact point are rendered graphically.
    bool m_show;


    //-----------------------------------------------------------------------
    // PROTECTED METHODS:
    //-----------------------------------------------------------------------

protected:

    void handle();


    //-----------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //-----------------------------------------------------------------------

protected:

    //! List of contacts.
    std::vector <cDynamicContact> m_contacts;

    //! Number of contacts.
    int m_numContacts;

    //! Simulation time.
    double m_simTime;
};


//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
