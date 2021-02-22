//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynWorldH
#define CDynWorldH
//---------------------------------------------------------------------------
#include "node/CDynBaseNode.h"
#include "node/CDynCollision.h"
#include "node/CDynConstraintList.h"
#include "matrix/CDynFrameStack.h"
#include "object/CDynObject.h"
#include "utility/CDynTimeEvent.h"
#include "var/CDynState.h"
#include "node/CDynCollision.h"
#include "node/CDynContact.h"
#include "node/CDynConstraintList.h"
#include "node/CDynBaseNode.h"
//---------------------------------------------------------------------------
class cDynJointNode;
class cDynNode;
class cDynObject;
class cDynString;
//---------------------------------------------------------------------------
extern void cDynIntegrateCallback(cDynTimeEvent* event, void *arg);
extern void cDynObjectNameCallback(char* (*f)(cDynObject* obj, void* arg));
extern char *cDynObjectName(cDynObject* obj);
//---------------------------------------------------------------------------

class cDynWorld
{
    friend cDynBaseNode::~cDynBaseNode();

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
public:

    //! Constructor of cDynWorld. 
    cDynWorld();


    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    //-----------------------------------------------------------------------
    /*!
        Update the cDynWorld time.
        Will cause any scheduled events within the time change to happen 
        (collision, integration, etc).

        \param  time Absolute time to move cDynWorld time to.
    */
    //-----------------------------------------------------------------------
    void update(cDynTime &time) 
    {
        while (time_ < time) 
        {
            timer_.advance(time);
            collisionTest(time);
        }
    }

    
    //-----------------------------------------------------------------------
    /*!
        Backup the cDynWorld time.
        Will cause any scheduled events within the time change to be backed
        out also.
        
        \param  time Absolute time to move back cDynWorld time to.
        \sa     backuplimit()
    */
    //-----------------------------------------------------------------------

    void backup(const cDynTime &time);

    cDynBaseNode* firstBase() { return firstBase_; }

    cDynBaseNode* insert(cDynObject *obj, char* data);
    void remove(cDynBaseNode* base);

    void insert(cDynTimeEvent* event) { timer_.insert(event); }
    void remove(cDynTimeEvent* event) { timer_.remove(event); }

    void insert(cDynConstraint* c) { constraints_.insert(c); }
    void remove(cDynConstraint* c) { constraints_.remove(c); }

    const cDynTime &time() const { return(time_); }
    const cDynTime &backupTime() const { return(backupTime_); }
    const cDynTime &backupLimit() const { return(backupLimit_); }

    //-----------------------------------------------------------------------
    /*!
        Set the Backup Limit to the maximum time the simulation should be able to backup.
        The simulation will not be able to backup further than this time.  
        
        \param  period Time period to allow simulation backup.
    */
    //-----------------------------------------------------------------------
    void backupLimit(const cDynTime &period) { if (period > 0.0) backupLimit_ = period; }

    //! Default gravity vector
    cDynVector3 gravity;

    void ignore(cDynObject* a, cDynObject* b, const bool cond=true) { collision_.ignore(a,b,cond); }
    void invalid(cDynObject* a, cDynObject* b, const bool cond=true) { collision_.invalid(a,b,cond); }
    
    int pairState(cDynObject* a, cDynObject* b) { return(collision_.pairState(a,b)); }

    void collisionUpdate(const cDynTime& time);

    void collisionSet(const cDynTime& time) 
    {
        if (time < collisionEvent_->time())
            collisionEvent_->time(time);
    }

    void collisionStep(const cDynTime& size) 
    {
        assert(size > 0.0);
        collisionEvent_->increment(size);
    }

    cDynTime collisionStep() 
    {
        return (collisionEvent_->increment());
    }

    void collisionTest(const cDynTime &time);

    cDynCollision& collision() { return(collision_); }

    cDynFrameStack frame;
    void callback(void (*f)(cDynWorld* world)) { callback_=f; }

    
    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
private:

    cDynTime time_;
    cDynTime backupTime_;
    cDynTime backupLimit_;

    //! List of nodes in world
    cDynBaseNode* firstBase_;			
    cDynBaseNode* lastBase_;			

    //! Place to hold user defined constraints
    cDynConstraintList constraints_;	

    //! Place for collision information
    cDynCollision collision_;		

    //! Collision test event
    cDynTimeEvent* collisionEvent_;	

    //! Time event heap
    cDynTimeHeap timer_;				

    cDynFrameStack frame_;

    void (*callback_)(cDynWorld* world);
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------