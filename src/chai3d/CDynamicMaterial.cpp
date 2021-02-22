//===========================================================================
/*
    This file is part of the Newton dynamics library.
    Copyright (C) 2011, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1048 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "chai3d/CDynamicMaterial.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cDynamicMaterial.
*/
//===========================================================================
cDynamicMaterial::cDynamicMaterial()
{
    m_dynMaterial = new cDynMaterial();

    set(1.1,     // default static friction
        1.5,     // default dynamic friction
        0.0,     // default viscous friction
        0.0,     // default grip friction
        0.0);    // default epsilon
}


//===========================================================================
/*!
    Destructor of cDynamicMaterial.
*/
//===========================================================================
cDynamicMaterial::~cDynamicMaterial()
{
    delete m_dynMaterial;
}


//===========================================================================
/*!
    Set material properties by passing all parameters at once.
*/
//===========================================================================
void cDynamicMaterial::set(double a_staticFriction,
                       double a_dynamicFriction,
                       double a_viscousFriction,
                       double a_gripFriction,
                       double a_epsilon)
{
    m_dynMaterial->friction(CDYN_FRICTION_STATIC, a_staticFriction); 
    m_dynMaterial->friction(CDYN_FRICTION_DYNAMIC, a_dynamicFriction); 
    m_dynMaterial->friction(CDYN_FRICTION_VISCOUS, a_viscousFriction); 
    m_dynMaterial->friction(CDYN_FRICTION_GRIP, a_gripFriction); 
    m_dynMaterial->epsilon(a_epsilon);
}