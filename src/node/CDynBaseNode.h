//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynBaseNodeH
#define CDynBaseNodeH
//---------------------------------------------------------------------------
#include "utility/CDynLogger.h"
#include "matrix/CDynMathDefn.h"
#include "utility/CDynTimeEvent.h"
#include "utility/CDynQHeader.h"
#include "var/CDynState.h"
#include "utility/CDynTimeEvent.h"
#include "matrix/CDynVector3.h"
//---------------------------------------------------------------------------
class cDynContactList;
class cDynJoint;
class cDynJointLimitList;
class cDynJointNode;
class cDynNode;
class cDynObject;
class cDynCollision;
class cDynWorld;
//---------------------------------------------------------------------------
enum cDynamicsModelType {CDYN_FULL,CDYN_SIMPLIFIED,CDYN_NONE};
enum cDynStatusType {CDYN_ACTIVE, CDYN_SLEEP, CDYN_INACTIVE, CDYN_ISOLATED};
enum cDynIntegratorType {CDYN_EULER_EXPLICIT, CDYN_EULER_MODIFIED, CDYN_EULER_HEUN, CDYN_EULER_IMPLICIT};
//---------------------------------------------------------------------------
class cDynBaseNode
{
    friend class cDynWorld;			 

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
private:

    cDynBaseNode(cDynWorld* world);
    //! Constructor of cDynBaseNode.    

public:

    //! Destructor of cDynBaseNode.
    ~cDynBaseNode();


  //-----------------------------------------------------------------------
  // PUBLIC METHODS:
  //-----------------------------------------------------------------------
public:

    //! User data.
    char *data() const { return(data_); }
    void data(char* d) { data_=d; }

    void link(cDynObject* obj);
    void unlink();

    cDynStatusType status() { return(status_); }
    void status(cDynStatusType status);

    cDynVector3 gravity;

    cDynObject *root() const { return root_; }

    //#ifndef CDYN_DONT_DOCUMENT
    //  cDynNode *rootNode() const { return rootNode_; }
    //#endif

    cDynWorld* world() const { return(world_); }
    cDynState* state() const { return(state_); }

    //-----------------------------------------------------------------------
    #ifndef CDYN_DONT_DOCUMENT
    //-----------------------------------------------------------------------

    long callnum() const { return(callnum_); }
    void callinc() { callnum_++; }
    cDynTime queryTime() { return(queryTime_); }
    cDynTime dynamicsTime() { return(dynamicsTime_); }
    void queryTime(const cDynTime time) { queryTime_=time; }

    void update(const bool normalize, const bool err) 
    {
        state_->update(normalize, err);
    }

    void update(const cDynTime time) 
    {
        state_->update(time);
    }

    void advance(const cDynTime time) 
    {
        state_->advance(time);
        rollback_.advance(time);
    }

    cDynCollision* collision() 
    {
        return(collision_);
    }

    cDynJointLimitList* limit(int i) 
    {
        return(limit_[i]);
    }

    //-----------------------------------------------------------------------
    #endif // CDYN_DONT_DOCUMENT
    //-----------------------------------------------------------------------

    void beginChange();
    void endChange();

    void backup(const cDynTime time);
    const cDynQHeader* rollback() const { return(&rollback_); }

    //-----------------------------------------------------------------------
    /*!
        Establish a dependency between (this) and (base) which will
        force rollback if either base is backed up.
    */
    //-----------------------------------------------------------------------
    void dependent(cDynBaseNode* base); 

    double potentialEnergy(const cDynTime& time);
    double kineticEnergy(const cDynTime& time);

    //-----------------------------------------------------------------------
    #ifndef CDYN_DONT_DOCUMENT
    //-----------------------------------------------------------------------

    void fwdDynamics(const cDynTime& time, bool config);
    
    /*
    const double fwdIn(cDynJoint* joint) 
    {
        if (fwdInCallback_ != NULL)
            return((*fwdInCallback_)(joint));
        else
            return(0.0);
    }

    const cDynVector3& fwdInSphere(cDynJoint* joint) 
    {
        if (fwdInSphereCallback_ != NULL)
            return((*fwdInSphereCallback_)(joint));
        else
            return(cDynVector3Zero);
    }
    
    void fwdOut(cDynJoint* joint, const double x) 
    {
        if (fwdOutCallback_ != NULL)
            (*fwdOutCallback_)(joint,x);
    }

    void fwdOutSphere(cDynJoint* joint, const cDynVector3& x) 
    {
        if (fwdOutSphereCallback_ != NULL)
            (*fwdOutSphereCallback_)(joint,x);
    }

    void fwdInCallback(const double (*f)(cDynJoint* joint)) {fwdInCallback_=f;}
    void fwdInSphereCallback(const cDynVector3& (*f)(cDynJoint* joint)) {fwdInSphereCallback_=f; }
    void fwdOutCallback(void (*f)(cDynJoint* joint, const double x)) {fwdOutCallback_=f; }
    void fwdOutSphereCallback(void (*f)(cDynJoint* joint, const cDynVector3& x)) {fwdOutSphereCallback_=f; }
    void fwdCallbacks(
    const double (*fi)(cDynJoint* joint),
    const cDynVector3& (*fis)(cDynJoint* joint),
    void (*fo)(cDynJoint* joint, const double x),
    void (*fos)(cDynJoint* joint, const cDynVector3& x)
    ) { fwdInCallback_=fi; fwdInSphereCallback_=fis; fwdOutCallback_=fo; fwdOutSphereCallback_=fos; }
    */

    //-----------------------------------------------------------------------
    #endif // CDYN_DONT_DOCUMENT
    //-----------------------------------------------------------------------

    const cDynTime& updateTime() const { return(updateEvent_->time()); }
    const cDynTime& updateInc() const { return(updateEvent_->inc()); }
    cDynIntegratorType integrator() const { return(integrator_); }

    void integrator(const cDynIntegratorType integrator,const int iterNumMax=1,const double iterErrorMax=0.05f)
    { 
        integrator_=integrator; 
        iterNumMax_=iterNumMax; 
        iterErrorMax_=iterErrorMax;
        iterErrorMax2_=iterErrorMax_*iterErrorMax_; 
    }
    
    void integrationStep(const cDynTime& size) 
    {
        assert(size > 0.0);
        updateEvent_->increment(size);
    }
    
    cDynTime integrationStep() 
    {
        return(updateEvent_->increment());
    }

    //-----------------------------------------------------------------------
    #ifndef CDYN_DONT_DOCUMENT
    //-----------------------------------------------------------------------

    cDynContactList* contact() { return(contact_); }

    //void integrate(const cDynTime& time,cDynTime& inc, const bool discontinuity);
    void integrate(const cDynTime& time,cDynTime& inc);
    void discontinuity() { discontinuity_=true; }

    void checkInvalids(cDynTime& time);
    cDynTime checkJointLimits(const cDynTime& time);

    friend void cDynIntegrateCallback(cDynTimeEvent* event, void *arg);

    //-----------------------------------------------------------------------
    #endif // CDYN_DONT_DOCUMENT
    //-----------------------------------------------------------------------

    cDynBaseNode* next() { return next_; }
    cDynBaseNode* prev() { return prev_; }


    //-----------------------------------------------------------------------
    // PRIVATE METHODS:
    //-----------------------------------------------------------------------
private:

    void insert(cDynBaseNode** head, cDynBaseNode** tail);
    void remove(cDynBaseNode** head, cDynBaseNode** tail);
    friend class cDynContactList;  

    cDynWorld* world_;
    cDynObject* root_;
//	cDynNode *rootNode_;

    cDynamicsModelType model_;
    cDynStatusType status_;
    cDynIntegratorType integrator_;
    bool discontinuity_;

    cDynState *state_;

    //! Queue of events to execute in event of rollback.
    cDynQHeader rollback_;		


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS: (COLLISION CHECKING)
    //-----------------------------------------------------------------------
private:

    //-----------------------------------------------------------------------
    /*!
        If collisions are allowed between bodies in base this is the 
        collision record for them. Should only be world()->collision() or NULL
    */
    //-----------------------------------------------------------------------
    cDynCollision*   collision_; 

    //! List of active/invalid joint limits
    cDynJointLimitList* limit_[2];	
    cDynContactList* contact_;


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS: (SIMULATION AND TIME)
    //-----------------------------------------------------------------------
private:

    //! integrate event timer
    cDynTimeEvent *updateEvent_;

    long callnum_;
    cDynTime queryTime_;
    cDynTime dynamicsTime_;

    //! Callbacks to fwdDynamicsConfig.
    double (*fwdInCallback_)(cDynJoint* joint);
    const cDynVector3& (*fwdInSphereCallback_)(cDynJoint* joint);
    void (*fwdOutCallback_)(cDynJoint* joint, const double x);
    void (*fwdOutSphereCallback_)(cDynJoint* joint, const cDynVector3& x);

    //! Relative error for the iteration in integration.
    int ErrorRelative(double *xe,double *x,int n);

    //! Maximum number of iteration for integration.
    int iterNumMax_; 

    //! Maximum absolute error bound for iteration.
    double iterErrorMax_; 

    //! Maximum absolute error bound^2 for iteration.
    double iterErrorMax2_; 

    //! User data.
    char* data_;

    cDynBaseNode *next_;
    cDynBaseNode *prev_;
};

//---------------------------------------------------------------------------
#endif // CDynBaseNodeH
//---------------------------------------------------------------------------