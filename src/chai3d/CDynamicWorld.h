//===========================================================================
/*
    This file is part of the Newton dynamics library.
    Copyright (C) 2011, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1247 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynamicWorldH
#define CDynamicWorldH
//---------------------------------------------------------------------------
#include "chai3d/CDynamicBase.h"
#include "chai3d.h"
//---------------------------------------------------------------------------

//===========================================================================
/*!
    \file       CDynamicWorld.h

    \brief 
    <b> Dynamics Module </b> \n 
    Dynamic World.
*/
//===========================================================================

//===========================================================================
/*!
    \class      cDynamicWorld
    \ingroup    Dynamics

    \brief      
    cDynamicWorld is a class that handles cDynamicBase objects which combine a CHAI3D
    graphical representation and a dynamics model.
*/
//===========================================================================
class cDynamicWorld : public chai3d::cGenericObject
{
    friend class cDynamicBase;

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

public:

    //! Constructor of cDynamicWorld.
    cDynamicWorld(chai3d::cWorld* a_parentWorld);

    //! Destructor of cDynamicWorld.
    virtual ~cDynamicWorld();


    //-----------------------------------------------------------------------
    // PUBLIC METHODS: (Dynamics)
    //-----------------------------------------------------------------------

public:

    //! Create a new base object.
    cDynamicBase* newBaseObject(const chai3d::cVector3d& a_pos, const chai3d::cMatrix3d& a_rot);

    //! Get pointer to base node by passing string name.
    cDynamicBase* getBaseNode(std::string a_name);

    //! Set gravity field.
    void setGravity(double a_x, double a_y, double a_z) { m_dynWorld->gravity.set(a_x, a_y, a_z); }

    //! Set gravity field.
    void setGravity(chai3d::cVector3d a_gravity) { m_dynWorld->gravity.set(a_gravity.x(), a_gravity.y(), a_gravity.z()); }

    //! Get gravity field.
    chai3d::cVector3d getGravity() { return (chai3d::cVector3d(m_dynWorld->gravity[0], m_dynWorld->gravity[1], m_dynWorld->gravity[2])); }

    //! Update dynamic simulation.
    void updateDynamics(double a_interval);

    //! Set collision detection update rate.
    void setCollisionRate(const double a_collisionRate) { m_dynWorld->collisionStep(a_collisionRate); }

    //! Get collision detection update rate.
    double getCollisionRate() { return (m_dynWorld->collisionStep()); };
   

    //-----------------------------------------------------------------------
    // PUBLIC METHODS: (Graphics)
    //-----------------------------------------------------------------------

public:

    //! Update global position frames.
    void updateGlobalPositions(const bool a_frameOnly);

    //! Copy frame from dynamics model to graphical model.
    void copyDynamicFrameToGraphicFrame(void);


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS: (General)
    //-----------------------------------------------------------------------

public:

    //! simulation time
    double m_time;

    //! Dynamic objects.
    std::list<cDynamicBase*> m_dynamicObjects;

    //! Newton dynamic world.
    cDynWorld* m_dynWorld;


    //-----------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //-----------------------------------------------------------------------

protected:

    //! CHAI3D world.
    chai3d::cWorld* m_parentWorld;


    //-----------------------------------------------------------------------
    // PROTECTED METHODS:
    //-----------------------------------------------------------------------

protected:

    //! Render world (OpenGL).
    virtual void render(chai3d::cRenderOptions& a_options);

    //! CHAI3D collision detection.
    virtual bool computeOtherCollisionDetection(chai3d::cVector3d& a_segmentPointA,
                                                chai3d::cVector3d& a_segmentPointB,
                                                chai3d::cCollisionRecorder& a_recorder,
                                                chai3d::cCollisionSettings& a_settings);
};


//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
