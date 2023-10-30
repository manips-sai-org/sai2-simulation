//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynContactH
#define CDynContactH
//---------------------------------------------------------------------------
#include "global/CDynGlobalDefn.h"
#include "matrix/CDynMathDefn.h"
#include "matrix/CDynVector3.h"
#include "matrix/CDynVector6.h"
#include "object/CDynObject.h"
#include "node/CDynFrictionRecord.h"
#include "node/CDynPrimPair.h"
#include "distance/CDynDist.h"
//---------------------------------------------------------------------------
#ifndef CDYN_DONT_DOCUMENT
//---------------------------------------------------------------------------
enum cDynContactType { CDYN_JOINT_LOWER = 0, 
                       CDYN_JOINT_UPPER = 1, 
                       CDYN_CONTACT = 2 };

enum cDynContactPointType { CDYN_CONSTRAINT_NORMAL = 0, 
                            CDYN_CONSTRAINT_BILATERAL = 1, 
                            CDYN_CONSTRAINT_NORMAL_FRICTION = 2, 
                            CDYN_CONSTRAINT_FRICTION_X = 3, 
                            CDYN_CONSTRAINT_FRICTION_Y = 4 };

enum cDynSetupType { CDYN_SETUP_UNINITIALIZED = 0, 
                     CDYN_SETUP_INITIALIZED = 1, 
                     CDYN_SETUP_LISTED = 2, 
                     CDYN_SETUP_ACCELERATION = 3, 
                     CDYN_SETUP_UPDATE = 4, 
                     CDYN_SETUP_EVENTCALLBACK = 5 };
//---------------------------------------------------------------------------
#undef C_DYN_DEBUG_OPS
#ifdef C_DYN_DEBUG_OPS
extern void cDynDebugEnergyReset();
extern void cDynDebugEnergy(cDynBaseNode* b, int i);
extern double cDynDebugKE(int i);
extern double cDynDebugPE(int i);
extern double cDynDebugTotal(int i);
extern double cDynDebugDeltaEnergy(double e);
#endif
//---------------------------------------------------------------------------
class cDynNode;
class cDynBaseNode;
class cDynJoint;
class cDynContact;
class cDynContactList;
class cDynContactEvent;
class cDynConstraint;
class cDynConstraintInfo;
//---------------------------------------------------------------------------

class cDynContactPoint
{
    friend class cDynContact;
    friend class cDynConstraint;
    friend class cDynContactList;
    friend class cDynContactEvent;
    friend class cDynContactPointCache;

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
private:

    //! Constructor of cDynContactPoint.    
    cDynContactPoint(cDynContact* c, const cDynVector3& n,const cDynVector3& r,
        const double err, const double maxerr,
        const cDynContactPointType type,
        cDynFrictionRecord* fr, cDynConstraintInfo* info=NULL);

public:

    //! Destructor of cDynContactPoint.
    ~cDynContactPoint();


    //-----------------------------------------------------------------------
    // PUBLIC METHODS: (CALLBACK FUNCTIONS)
    //-----------------------------------------------------------------------
public:

    friend double cDynCallbackLimitIn(cDynJoint* joint);
    friend double cDynCallbackContactIn(cDynJoint* joint);
    friend const cDynVector3& cDynCallbackContactSphereIn(cDynJoint* joint);
    friend void cDynCallbackLambdaOut(cDynJoint* joint,const double x);
    friend void cDynCallbackLambdaSphereOut(cDynJoint* joint,const cDynVector3& x);
    friend double cDynCallbackLambdaUpdate(cDynJoint* joint);
    friend const cDynVector3& cDynCallbackLambdaSphereUpdate(cDynJoint* joint);
    friend double cDynCallbackJtIn(cDynJoint* joint);
    friend const cDynVector3& cDynCallbackJtSphereIn(cDynJoint* joint);
    friend void cDynCallbackPositionOut(cDynJoint* joint, const double x);
    friend void cDynCallbackPositionSphereOut(cDynJoint* joint, const cDynVector3& x);
    friend void cDynCallbackVelocityOut(cDynJoint* joint, const double x);
    friend void cDynCallbackVelocitySphereOut(cDynJoint* joint, const cDynVector3& x);
    friend void cDynCallbackAccelerationOut(cDynJoint* joint, const double x);
    friend void cDynCallbackAccelerationSphereOut(cDynJoint* joint, const cDynVector3& x);
    friend double cDynCallbackAfreeIn(cDynJoint* joint);
    friend const cDynVector3& cDynCallbackAfreeSphereIn(cDynJoint* joint);


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
// private:
    
    cDynContactPointType type_;
    cDynVector6 s_;
    cDynVector3 r_;
    double err_;
    double maxerr_;
    double force_;
    double impulse_;
    double velocity_;
    double deltavelocity_;
    double acceleration_;
 
    cDynFrictionRecord* fr_;
    cDynConstraintInfo* info_;
    cDynContact* contact_;
    cDynContactPoint* prev_;
    cDynContactPoint* next_;


    //-----------------------------------------------------------------------
    // PRIVATE STATIC MEMBERS:
    //-----------------------------------------------------------------------
 private:
    static cDynContactPoint* free_;


    //-----------------------------------------------------------------------
    // OPERATORS:
    //-----------------------------------------------------------------------
private:

    void* operator new(size_t s) 
    {
        if (free_==NULL) 
        {
            cDynContactPoint* tmp=(cDynContactPoint *)malloc(s);
//			if (cDynDebugLevel == -100)
//				cDynPrintf("cDynContactPoint\n");
            return(tmp);
        } 
        else 
        {
            cDynContactPoint* tmp=free_;
            free_=free_->next_;
            return(tmp);
        }
    }

    void operator delete(void *ptr) 
    {
        cDynContactPoint* tmp=(cDynContactPoint *)ptr;
        tmp->next_=free_;
        free_=tmp;
    }
};

//---------------------------------------------------------------------------

class cDynContact
{
    friend class cDynContactList;
    friend class cDynContactEvent;
    friend class cDynWorld; // debug only

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
private:

    //! Constructor of cDynContact.    
    cDynContact() 
    {
        numPoints_ = 0;

        type_ = CDYN_CONTACT;
        condition_ = -100;
        epsilon_ = -100;

        joint_ = NULL;
        tau_ = -100;

        prim_ = NULL;
        pair_ = NULL;
        constraint_ = NULL;
        head_ = NULL;
        tail_ = NULL;

        n_ = -100;

        list_ = NULL;
        index_ = -100;
    }

public:
    //! Destructor of cDynContact.
    //~cDynContact() {};

    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    void addPoint(const cDynVector3& n,const cDynVector3& r, const double err, const double maxerr, cDynFrictionRecord* fr);
    void resetPoints();
    void findPoints(const cDynTime& time);


    //-----------------------------------------------------------------------
    // PUBLIC METHODS: (CALLBACK FUNCTIONS)
    //-----------------------------------------------------------------------
public:

    friend double cDynCallbackLimitIn(cDynJoint* joint);
    friend void cDynCallbackLambdaOut(cDynJoint* joint,const double x);
    friend void cDynCallbackLambdaSphereOut(cDynJoint* joint,const cDynVector3& x);
    friend double cDynCallbackLambdaUpdate(cDynJoint* joint);
    friend const cDynVector3& cDynCallbackLambdaSphereUpdate(cDynJoint* joint);
    friend double cDynCallbackJtIn(cDynJoint* joint);
    friend const cDynVector3& cDynCallbackJtSphereIn(cDynJoint* joint);
    friend void cDynCallbackPositionOut(cDynJoint* joint, const double x);
    friend void cDynCallbackPositionSphereOut(cDynJoint* joint, const cDynVector3& x);
    friend void cDynCallbackVelocityOut(cDynJoint* joint, const double x);
    friend void cDynCallbackVelocitySphereOut(cDynJoint* joint, const cDynVector3& x);
    friend void cDynCallbackAccelerationOut(cDynJoint* joint, const double x);
    friend void cDynCallbackAccelerationSphereOut(cDynJoint* joint, const cDynVector3& x);
    friend double cDynCallbackAfreeIn(cDynJoint* joint);
    friend const cDynVector3& cDynCallbackAfreeSphereIn(cDynJoint* joint);
    friend class cDynContactPoint;
    friend class cDynContactPointCache;


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
private:

    cDynContactType type_;

    //! 0 = unilateral, 1 = bilateral.
    int condition_; 	
    double epsilon_;

    //! For joint limits.
    cDynJoint* joint_;
    double tau_;

    //! For contact points.
    cDynPrimPair* prim_;
    cDynContact* pair_;
    int numPoints_;
    cDynConstraint*    constraint_;
    cDynContactPoint* head_;
    cDynContactPoint* tail_;

    int n_;

    cDynContactList* list_;
    int index_;

    static cDynContact* free_;

    void* operator new(size_t s) 
    {
        if (free_==NULL) 
        {
            cDynContact* tmp=(cDynContact*)malloc(s);
            return(tmp);
        } 
        else 
        {
            cDynContact* tmp=free_;
            free_=free_->pair_;
            return(tmp);
        }
    }

    void operator delete(void *ptr) 
    {
        cDynContact* tmp=(cDynContact *)ptr;
        tmp->pair_=free_;
        free_=tmp;
    }
};

//---------------------------------------------------------------------------

class cDynContactList
{
    friend class cDynBaseNode;
    friend double cDynCallbackAfreeIn(cDynJoint* joint);
    friend const cDynVector3& cDynCallbackAfreeSphereIn(cDynJoint* joint);
    friend class cDynWorld; // debug only.

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------

private:
    //! Constructor of cDynContactList. cDynContactList can only be created by the cDynBaseNode class
    cDynContactList(cDynBaseNode* base, const int size=4);

public:
    //! Destructor of cDynContactList.
    ~cDynContactList();


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //-----------------------------------------------------------------------
public:

    cDynTime time() const { return(time_); }
    cDynTime inc() const { return(inc_); }

    void wakeForce(double f) { wakeForce_=f; }
    void wakeImpact(double p) { wakeImpulse_=p; }
    void wakeImpulse(double p) { wakeImpulse_=p; }

    double wakeForce() const { return(wakeForce_); }
    double wakeImpact() const { return(wakeImpulse_); }
    double wakeImpulse() const { return(wakeImpulse_); }

    void flush();
    void reset();
    int setup(const int n);
    void update();
    void event();
    void setupReset();
    cDynSetupType setup() const { return(setup_); }
    void jointLimit(const cDynTime& time, cDynJoint *joint,const cDynBoundType);
    void contact(const cDynTime& time, cDynPrimPair* pair, cDynConstraint* constraint=NULL);
    int n() const { return(n_); }
    void remove(const int i);
    void updatePersistent(const cDynTime time);
    bool resolve(const int m);

    void callback(cDynContactEvent* event) { contactEvent_=event; }
    cDynContactEvent* callback() { return(contactEvent_); }

    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
private:

    cDynBaseNode* base_;
    cDynSetupType setup_;
    cDynContactEvent* contactEvent_;
    cDynTime time_;
    cDynTime inc_;

    double wakeForce_;
    double wakeImpulse_;

    bool error_;

    int size_;
    int n_;
    cDynContact** list_;


    //-----------------------------------------------------------------------
    // PRIVATE METHODS:
    //-----------------------------------------------------------------------
private:

    void grow(const int size);
    void list(cDynContact** a);
    bool error() const { return(error_); };
    void error(const bool err) { error_=err; };

    void force(const int m, cDynContact** contact, double* f);
    void acceleration(const int m, cDynContact** contact, double* a);
    void updateLambda(double direction);

    void display(const int m, double *l, double *b);
    void displaySmall(const int m, double *l, double *b);
    void display(const int m, const int n, double* l, double *diag, double *b, int* index);
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------