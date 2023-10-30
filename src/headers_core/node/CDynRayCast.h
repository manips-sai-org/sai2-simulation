//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynRayCastH
#define CDynRayCastH
//---------------------------------------------------------------------------
#include <math.h>
//---------------------------------------------------------------------------
#include "matrix/CDynMathDefn.h"
#include "matrix/CDynVector3.h"
#include "matrix/CDynFrame.h"
#include "matrix/CDynTransform.h"
#include "object/CDynObject.h"
#include "node/CDynBaseNode.h"
#include "node/CDynWorld.h"
//---------------------------------------------------------------------------
class cDynObject;
//---------------------------------------------------------------------------

class cDynRayCast
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
public:

    //-----------------------------------------------------------------------
    /*!
        Define a ray starting at position \a pos and continuing in direction \a dir.
        Optionally a radius \r can be defined which will trigger a intersection
        to be marked if the ray travels within \a r distance of a given object
        geometry.

        \param p origin of the ray in global (world) space
        \param dir direction in which ray is cast in world space
        \param r radius around ray in which intersection is triggered.
    */
    //-----------------------------------------------------------------------
    cDynRayCast(cDynVector3& p, cDynVector3& dir, double r=0);


    //-----------------------------------------------------------------------
    // PUBLIC METHODS: (SETTINGS)
    //-----------------------------------------------------------------------
public:

    //! Set direction of ray.
    void direction(const cDynVector3& v) { dir_ = v; dir_.normalize(); update(); }

    //! Get direction of ray.
    const cDynVector3& direction() const { return(dir_); }

    //! Set position of ray.
    void position(const cDynVector3& p) { p_ = p; update(); }

    //! Get position of ray.
    const cDynVector3& position() const { return(p_); }

    //! Set radius of ray.
    double radius() const { return(r_); }

    //! Get radius of ray.
    void radius(double r) { r_ = fabs(r); }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS: (RESET AND TESTING)
    //-----------------------------------------------------------------------
public:
    
    //! Reset all intersections.
    void reset() { node_=NULL; prim_=NULL; max_=maximum_; min_=0; }
        
    //! Check node for ray intersection.
    void check(cDynObject* node, cDynTime& time, bool tree=false);

    //! Check entire basenode for ray intersection.
    void check(cDynBaseNode* base, cDynTime& time);
    
    //! Check entire world for ray intersection.
    void check(cDynWorld* world, cDynTime& time);


    //-----------------------------------------------------------------------
    // PUBLIC METHODS: (QUERY)
    //-----------------------------------------------------------------------
public:

    //! Return \a true if ray intersected any bodies specified by check().
    bool intersection() { return(prim_ != NULL); }
        
    //! Set \a v to first point of intersection between ray and specified geometry. Return \a false otherwise.
    bool intersection(cDynVector3& v) 
    {
        if (prim_ == NULL) return(false);
        v.multiply(dir_, max_);
        v+=p_;
        return(true);
    }

    //! Return distance to point of intersection of ray.
    double distance() { return(max_); }
        
    //! Return pointer to nearest node, \a NULL otherwise.
    cDynObject* node() { return(node_); }
    
    //! Return pointer to nearest prim in node node(), \a NULL otherwise.
    cDynPrim* prim() { return(prim_); }


    //-----------------------------------------------------------------------
    // PRIVATE METHODS:
    //-----------------------------------------------------------------------
private:

    const cDynTransform &transform() { return T_; }
    void update();
    void checkOne(const cDynBSphere* bs, cDynTransform& Tnr,double& z, double& range);
    void check(cDynObject* node, const cDynBSphere* bs, cDynTransform& Tnr, cDynPrim* cache[], int n);
    void check(cDynObject* node, cDynPrim* prim, cDynTransform& Tnr);


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
private:

    cDynVector3 p_;
    cDynVector3 dir_;
    double r_;
    cDynTransform T_;

    cDynObject* node_;
    cDynPrim* prim_;
    double maximum_;
    double min_;
    double max_;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------