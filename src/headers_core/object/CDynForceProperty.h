//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynForcePropertyH
#define CDynForcePropertyH
//---------------------------------------------------------------------------
#include "object/CDynProperty.h"
#include "object/CDynObject.h"
#include "object/CDynForce.h"
//---------------------------------------------------------------------------

struct cDynForceRecord
{
    cDynObject* obj;
    cDynVector6* accum;
};

//---------------------------------------------------------------------------

class cDynForceProperty: public cDynProperty
{
    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
public:

    //! Constructor of cDynProperty.   
    cDynForceProperty():cDynProperty(NULL) {}
    

    //----------------------------------------------------------------------
    // PUBLIC METHODS:
    //----------------------------------------------------------------------
public:

    cDynObject* node(void* arg) { return(((cDynForceRecord*)arg)->obj); }
    const cDynFrame &globalFrame(cDynObject* node) { return(node->globalFrame()); }

    // faster routine (one less copy)
    const cDynVector6 &localVelocity(cDynObject* node) { return(node->velocity()); } 

    // (faster routine. One less copy)
    void globalVelocity(cDynObject* node, cDynVector6& v) 
    {			   
        const cDynVector6& vlocal=localVelocity(node);
        v[0].multiply(globalFrame(node).rotation(),vlocal[0]);
        v[1].multiply(globalFrame(node).rotation(),vlocal[1]);
    }

    // (slower requires a copy)
    const cDynVector6 globalVelocity(cDynObject* node) 
    {				   
        cDynVector6 v; globalVelocity(node,v);
        return(v);
        
    }

    void force(void *arg, cDynVector6& f) 
    {
        *(((cDynForceRecord*)arg)->accum) += f;
    }
    
    void force(void *arg,cDynVector3& pos, cDynVector3& f,bool globalForce=false) 
    {
        cDynVector6 s;
        if (globalForce)
            s[0].inversedMultiply(globalFrame(node(arg)).rotation(),f);
        else
            s[0]=f;
        s[1].crossMultiply(pos,s[0]);
        force(arg,s);
    }
    
    void moment(void* arg,cDynVector3& pos, cDynVector3& f,bool globalMoment=false) 
    {
        cDynVector6 s;
        if (globalMoment)
            s[1].inversedMultiply(globalFrame(node(arg)).rotation(),f);
        else
            s[1]=f;
        s[0].crossMultiply(s[1],pos);
        force(arg,s);
    }
    
    //virtual void action(void* arg);
};

//---------------------------------------------------------------------------

class cDynPositionForceProperty: public cDynForceProperty
{
    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
public:

    //! Constructor of cDynPositionForceProperty. 
    cDynPositionForceProperty(cDynVector3& pos, cDynVector3& f):pos_(pos),force_(f),on_(true),global_(false) {}
    cDynPositionForceProperty():on_(true) { pos_.zero(); force_.zero(); }


    //----------------------------------------------------------------------
    // PUBLIC METHODS:
    //----------------------------------------------------------------------
public:

    cDynVector3& position() { return pos_; }
    void position(cDynVector3& p) { pos_=p; }
    cDynVector3& force() { return force_; }
    void force(cDynVector3& f) { force_=f; }

    // true force is on, false force is off
    void state(bool state=true) { on_=state; }
    bool state() { return on_; }

    // true force is defined in global coordinates like a field, false force is in local coordinates
    void global(bool state=true) { global_=state; }
    bool global() { return global_; }


    //----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //----------------------------------------------------------------------
private:

    cDynVector3 pos_;
    cDynVector3 force_;
    bool on_;
    bool global_;


    //----------------------------------------------------------------------
    // PRIVATE METHODS:
    //----------------------------------------------------------------------
private:

    virtual void action(void *arg) 
    { 
        if (on_) cDynForceProperty::force(arg,pos_,force_,global_);
    }

};

//---------------------------------------------------------------------------

class cDynSpacialForceProperty: public cDynForceProperty
{
    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
public:

    //! Constructor of cDynSpacialForceProperty.  
    cDynSpacialForceProperty(cDynVector6& s):s_(s),on_(true) {}
    cDynSpacialForceProperty():on_(true) { s_.zero(); }


    //----------------------------------------------------------------------
    // PUBLIC METHODS:
    //----------------------------------------------------------------------
public:

    cDynVector6& force() { return s_; }
    void force(cDynVector6& f) { s_=f; }
    void zerp() { s_.zero(); }


    //----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //----------------------------------------------------------------------
private:

    cDynVector6 s_;
    bool on_;


    //----------------------------------------------------------------------
    // PRIVATE METHODS:
    //----------------------------------------------------------------------
private:
    virtual void action(void *arg) 
    { 
        if (on_) cDynForceProperty::force(arg,s_);
    }
};

//---------------------------------------------------------------------------

class cDynDamperForceProperty: public cDynForceProperty
{
    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
public:

    //! Constructor of cDynDamperForceProperty.    
    cDynDamperForceProperty(cDynVector3& pos, double b):pos_(pos),b_(b) {}


    //----------------------------------------------------------------------
    // PUBLIC METHODS:
    //----------------------------------------------------------------------
public:
    cDynVector3& position() { return pos_; }
    void position(cDynVector3& p) { pos_=p; }
    double damping() { return b_; }
    void damping(double b) { b_=b; }


    //----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //----------------------------------------------------------------------
private:

    cDynVector3 pos_;
    double b_;


    //----------------------------------------------------------------------
    // PRIVATE METHODS:
    //----------------------------------------------------------------------
private:

    virtual void action(void *arg) 
    { 
        if (b_ == 0.0) return;
        cDynObject* n=node(arg);
        const cDynVector6& vs=localVelocity(n);
        cDynVector3 v;
        v.crossMultiply(pos_, vs[1]);
        v += vs[0];

        v *= -b_;
        cDynForceProperty::force(arg,pos_,v,false);
    }
};

//---------------------------------------------------------------------------


// class implements the basic functionality required to model a lifting body

class cDynLiftForceProperty: public cDynForceProperty
{
    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
public:

    //! Constructor of cDynLiftForceProperty. 
    cDynLiftForceProperty(cDynVector3& center, cDynVector3& n):center_(center),n_(n) { n_.normalize(); }
    cDynLiftForceProperty() { center_.zero(); n_.zero(); }


    //----------------------------------------------------------------------
    // PUBLIC METHODS:
    //----------------------------------------------------------------------
public:

    //! Get center of lift.
    cDynVector3& center() { return center_; }

    //! Set center of lift.
    void center(cDynVector3& center) { center_=center; }
    
    //! Get normal to lift plane.
    cDynVector3& normal() { return n_; }

     //! Set normal to lift plane.
    void normal(cDynVector3& normal) { n_=normal; n_.normalize(); }

    virtual void liftdrag(double cosAngle, double v, double& l, double& d) = 0;


    //----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //----------------------------------------------------------------------
private:

    //! Center of lift
    cDynVector3 center_;

    //! Normal to lift plane
    cDynVector3 n_;		


    //----------------------------------------------------------------------
    // PRIVATE METHODS:
    //----------------------------------------------------------------------
private:
    
    virtual void action(void *arg) 
    { 
        cDynObject* n=node(arg);
        const cDynVector6& vs=localVelocity(n);
        cDynVector3 v;

        // find local velocity at point
        v.crossMultiply(center_, vs[1]);
        v += vs[0];

        double vmag=v.magnitude();
        if (vmag < 1e-8) return;
        cDynVector3 dDir; dDir.multiply(v,1.0/vmag);
        cDynVector3 lDir;

        // find angle of attack
        double cosAngle=n_.dot(dDir); 
        if (fabs(cosAngle) < 1.0 - 1e-8) 
        { 
            //find lift direction
            cDynVector3 x; x.crossMultiply(v,n_);
            x.normalize();
            lDir.crossMultiply(x,v);
        } 
        else 
        { 
            // very high angle of attack
            lDir.zero();
        }
        double l, d;

        // find coefficients of lift and drag
        liftdrag(cosAngle,vmag,l,d);
        
        // find lift and drag vector
        lDir *= l;
        dDir *= d;
        
        // find total force
        cDynVector3 f; f.add(lDir,dDir);

        // apply force to body
        cDynForceProperty::force(arg,center_,f,false);
    }
};

//---------------------------------------------------------------------------

class cDynSimpleLiftForceProperty: public cDynLiftForceProperty
{
    //----------------------------------------------------------------------
    // PUBLIC METHODS:
    //----------------------------------------------------------------------
public:

    //! Get wing plane form area.
    double area() { return area_; }

    //! Set wing plane form area.
    void area(double a) { area_=a; m(); }
    
    //! Set fluid density ( is a function of temperature/pressure(height) )
    double density() { return p_; } // 1.2 kg/m^3 @ 20 C

    //! Get fluid density ( is a function of temperature/pressure(height) )
    void density(double p) { p_=p; m(); }

    virtual void liftdrag(double cosAngle, double v, double& l, double& d) 
    {
        double s=v*v*m_;
        l=Cl_*s;
        d=Cd_*s;
        
    };


    //----------------------------------------------------------------------
    // PRIVATE METHODS:
    //----------------------------------------------------------------------
private:

    void m() { m_=0.5*p_*area_; }
    
    
    //----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //----------------------------------------------------------------------
private:

    double area_;
    double p_;
    double m_;
    double Cl_;
    double Cd_;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------