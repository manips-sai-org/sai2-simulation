//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynContactEventH
#define CDynContactEventH
//---------------------------------------------------------------------------
#include "global/CDynGlobalDefn.h"
#include "node/CDynContact.h"
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
/*!
    This class from which others can be derived, allows information about the 
    nature, and type of contacts between bodies to be made available to the 
    application. The reasons for doing so are manifold but may include adding 
    sound, when an impact occurs, or taking some action when an object is 
    struct with a significant impact. Information available to the virtual 
    handle() method includes (point and time of contact, normal between 
    bodies, nodes and primitives involved, as well as dynamic information 
    such as impact, contact forces, and relative velocities, accelerations 
    after contact). For a full list see method list below.
*/
//---------------------------------------------------------------------------

class cDynContactEvent
{
    friend class cDynContactList;

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
public:

    //! Constructor of cDynContactEvent. Normally should only be called as part of derived class.
    cDynContactEvent(): time_(0), contact_(NULL) {}
        

    //-----------------------------------------------------------------------
    // PUBLIC METHODS: (CONTANT/COLLISION INFORMATION)
    //-----------------------------------------------------------------------
public:

    //! Point of contact/collision defined in global frame.
    void point(cDynVector3& p) const 
    { 
        if (contact_ != NULL) p=contact_->r_; 
    }

    //! Surface normal at contact/collision point defined in global frame.
    void normal(cDynVector3& n) const { if (contact_ != NULL) 
    {
        if (dir_<0) { n.negate(contact_->s_[0]);}
        else { n=contact_->s_[0]; } }
    }

    //! Impulse force resolved at collision point, may be zero if resting/sliding contact.
    double impulse() const { return((contact_!=NULL)?contact_->impulse_:0); }
    
    //! Contact force resolved at contact point, may be zero if instantaneous collision
    double force() const { return((contact_!=NULL)?contact_->force_:0); }
        
    double velocity() const { return((contact_!=NULL)?contact_->velocity_:0); }
    double velocityDelta() const { return((contact_!=NULL)?contact_->deltavelocity_:0); }
    double acceleration() const { return((contact_!=NULL)?contact_->acceleration_:0); }
    
    bool  friction() const { return((contact_!=NULL)?(contact_->type_ == CDYN_CONSTRAINT_NORMAL_FRICTION):false); }

    void  forceFriction(cDynVector3& f) const 
    {
        f.zero();
        if (friction()) 
        {
            cDynVector3 tmpV;
            for (cDynContactPoint* p=contact_->next_;p != NULL
            && (p->type_ == CDYN_CONSTRAINT_FRICTION_X
            || p->type_ == CDYN_CONSTRAINT_FRICTION_Y);
            p=p->next_) 
            {
                tmpV.multiply(p->s_[0], p->force_);
                f += tmpV;
            }
        }
    }

    void  impulseFriction(cDynVector3& i) const 
    {
        i.zero();
        if (friction()) 
        {
            cDynVector3 tmpV;
            for (cDynContactPoint* p=contact_->next_;p != NULL
            && (p->type_ == CDYN_CONSTRAINT_FRICTION_X
            || p->type_ == CDYN_CONSTRAINT_FRICTION_Y);
            p=p->next_) 
            {
                tmpV.multiply(p->s_[0], p->impulse_);
                i += tmpV;
            }
        }
    }

    //! Equivalent to nodeA().
    const cDynObject* node() const { return(nodeA()); }
    const cDynObject* nodeA() const { if (contact_ != NULL) return((dir_<0)?contact_->contact_->prim_->nodeB_:contact_->contact_->prim_->nodeA_); else return(NULL); }
    const cDynObject* nodeB() const { if (contact_ != NULL) return((dir_<0)?contact_->contact_->prim_->nodeA_:contact_->contact_->prim_->nodeB_); else return(NULL); }
    const cDynPrim* prim() const { return(primA()); }
    const cDynPrim* primA() const { if (contact_ != NULL) return((dir_<0)?contact_->contact_->prim_->primB_:contact_->contact_->prim_->primA_); else return(NULL); }
    const cDynPrim* primB() const { if (contact_ != NULL) return((dir_<0)?contact_->contact_->prim_->primA_:contact_->contact_->prim_->primB_); else return(NULL); }
    cDynTime time() const { return((contact_ != NULL)?time_:cDynTime(0.0)); }
    

    //-----------------------------------------------------------------------
    // PUBLIC METHODS: (HANDLE)
    //-----------------------------------------------------------------------
public:

    virtual void handle() {}
    

    //-----------------------------------------------------------------------
    // PRIVATE METHODS:
    //-----------------------------------------------------------------------
private:

    void set(cDynContactPoint* contact, cDynTime& time, int dir) 
    { 
        time_=time; 
        contact_=contact; 
        dir_=dir; 
    }


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
private:

    cDynTime time_;
    cDynContactPoint* contact_;
    int dir_;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
