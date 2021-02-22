//===========================================================================
/*
    This file is part of the Newton dynamics library.
    Copyright (C) 2011, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1198 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynamicMaterialH
#define CDynamicMaterialH
//---------------------------------------------------------------------------
#include "chai3d/CDynInternal.h"
//---------------------------------------------------------------------------
class cDynamicLink;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CDynamicMaterial.h

    \brief 
    <b> Dynamics Module </b> \n 
    Dynamic Material.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cDynamicMaterial
    \ingroup    Dynamics

    \brief      
    cDynamicMaterial defines physiqual material properties for a 
    cDynamicLink object.
*/
//===========================================================================
class cDynamicMaterial
{
    friend class cDynamicLink;

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

public:

    //! Constructor of cDynamicMaterial.
    cDynamicMaterial();

    //! Destructor of cDynamicMaterial.
    virtual ~cDynamicMaterial();


    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------

public:

    //! Set static friction property.
    void setStaticFriction(double a_value) { m_dynMaterial->friction(CDYN_FRICTION_STATIC, a_value); }

    //! Get static friction property.
    double getStaticFriction() { return (m_dynMaterial->friction(CDYN_FRICTION_STATIC)); }

    //! Set dynamic friction property.
    void setDynamicFriction(double a_value) { m_dynMaterial->friction(CDYN_FRICTION_DYNAMIC, a_value); }

    //! Get dynamic friction property.
    double getDynamicFriction() { return (m_dynMaterial->friction(CDYN_FRICTION_DYNAMIC)); }

    //! Set viscous friction property.
    void setViscousFriction(double a_value) { m_dynMaterial->friction(CDYN_FRICTION_VISCOUS, a_value); }

    //! Get viscous friction property.
    double getViscousFriction() { return (m_dynMaterial->friction(CDYN_FRICTION_VISCOUS)); }

    //! Set grip friction property.
    void setGripFriction(double a_value) { m_dynMaterial->friction(CDYN_FRICTION_GRIP, a_value); }

    //! Get grip friction property.
    double getGripFriction() { return (m_dynMaterial->friction(CDYN_FRICTION_GRIP)); }

    //! Set elasticity property of material.
    void setEpsilon(double a_value) { m_dynMaterial->epsilon(a_value); }

    //! Get elasticity property of material.
    double getEpsilon() { return (m_dynMaterial->epsilon()); }

    //! Set all properties at once.
    void set(double a_staticFriction,
             double a_dynamicFriction,
             double a_viscousFriction,
             double a_gripFriction,
             double a_epsilon);


    //-----------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //-----------------------------------------------------------------------

protected:

    //! Material model
    cDynMaterial* m_dynMaterial;
};


//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
