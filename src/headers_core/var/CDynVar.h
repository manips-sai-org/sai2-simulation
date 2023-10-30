//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynVarH
#define CDynVarH
//---------------------------------------------------------------------------
#ifndef CDYN_DONT_DOCUMENT
//---------------------------------------------------------------------------
#include <cstdlib>
//---------------------------------------------------------------------------
#include "global/CDynGlobalDefn.h"
#include "matrix/CDynMathDefn.h"
#include "matrix/CDynVector3.h"
#include "matrix/CDynVector6.h"
#include "matrix/CDynQuaternion.h"
#include "var/CDynState.h"
//---------------------------------------------------------------------------
enum DeVarStatus 
{ 
    CDYN_VAR_FREE, 
    CDYN_VAR_BOUND 
};
//---------------------------------------------------------------------------
class cDynJointVar;
class cDynJointSphereVar;
//---------------------------------------------------------------------------

class cDynJointInterval
{
    friend class cDynJointVar;

    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
public:

    // Constructor.
    cDynJointInterval(cDynJointVar* var, const cDynTime t_,const cDynVector3& q_);

    // Destructor.
    ~cDynJointInterval();


    //--------------------------------------------------------------------------
    /*!

    */
    //--------------------------------------------------------------------------

    //----------------------------------------------------------------------
    // OPERATORS:
    //----------------------------------------------------------------------
    inline void* operator new(size_t s);
    inline void operator delete(void *ptr);


    //----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //----------------------------------------------------------------------
private:
    
    cDynJointVar* var_;
    cDynTime t_;
    cDynVector3 q_;
    double dt_;

    cDynJointInterval* prev_;
    cDynJointInterval* next_;

#ifdef CDYN_EXTENDED
    cDynVector6 a_;
    int valid_;
#endif


    //----------------------------------------------------------------------
    // PRIVATE STATIC MEMBERS:
    //----------------------------------------------------------------------

    //! Linked list of free cDynJointIntervals.
    static cDynJointInterval* free_; 
};

//--------------------------------------------------------------------------

class cDynJointSphereInterval
{
    friend class cDynJointSphereVar;

    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
public:
    
    ~cDynJointSphereInterval();


    //----------------------------------------------------------------------
    // OPERATORS:
    //----------------------------------------------------------------------
    void* operator new(size_t s) 
    {
        if (free_ == NULL) 
        {
            cDynJointSphereInterval *tmp = (cDynJointSphereInterval*)malloc(s);
            return tmp;
        } 
        else 
        {
            cDynJointSphereInterval *tmp = free_;
            free_ = free_->next_; 
            return tmp;
        }
    }

    void operator delete(void *ptr) 
    {
        cDynJointSphereInterval* tmp = (cDynJointSphereInterval*)ptr;
        tmp->next_ = free_;
        free_ = tmp;
    }


    //----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //----------------------------------------------------------------------
private:
    
    cDynJointSphereVar* var_;
    cDynTime t_;
    cDynQuaternion q_;
    cDynVector3 v_;
    cDynVector3 a_;
    double dt_;

    cDynJointSphereInterval* prev_;
    cDynJointSphereInterval* next_;


    //----------------------------------------------------------------------
    // PRIVATE STATIC MEMBERS:
    //----------------------------------------------------------------------
    static cDynJointSphereInterval* free_; // linked list of free cDynJointSphereIntervals

    cDynJointSphereInterval(cDynJointSphereVar* var, const cDynTime t_,const cDynQuaternion& q_, const cDynVector3& v_, const cDynVector3& a);
};

//--------------------------------------------------------------------------

class cDynJointVar
{
    friend cDynJointInterval::cDynJointInterval(cDynJointVar* var,const cDynTime t,const cDynVector3& q);
    friend cDynJointInterval::~cDynJointInterval();

    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
public:

    cDynJointVar(const char* str=NULL) 
    { 
        status_=CDYN_VAR_FREE;
        t_=CDYN_TIME_UNINITIALIZED;
        q_.zero();
        error_=0.0f;
        tau_=0.0f;
        tauExternal_=0.0f;
        tauKinematic_=0.0f;
        state_=NULL;
        head_=tail_=NULL;
    }


    //----------------------------------------------------------------------
    // PUBLIC METHODS:
    //----------------------------------------------------------------------
public:

    void state(cDynState* s);
    inline cDynStateEntry* state() const { return(state_); }

    void backup(const cDynTime t);
    void update(const cDynTime t);
    void advance(const cDynTime t);
    
    inline double q() const
    { 
        assert(state_ != NULL); return(state_->x(0)); 
    }
    
    inline double v() const
    { 
        assert(state_ != NULL); return(state_->x(1)); 
    }
    
    inline double a() const
    { 
        assert(state_ != NULL); return(state_->dxdt(1)); 
    }

    inline void q(double q)
    { 
        state_->x(0)=q; 
    }
    
    inline void v(double v)
    { 
        state_->x(1)=state_->dxdt(0)=v; 
    }
    
    inline void a(double a)
    { 
        state_->dxdt(1)=a; 
    }

    inline void qCurrent(double q) { q_[0]=q; }
    inline void vCurrent(double v) { q_[1]=v; }
    inline void aCurrent(double a) { q_[2]=a; }

    inline double qCurrent() const { return(q_[0]); }
    inline double vCurrent() const { return(q_[1]); }
    inline double aCurrent() const { return(q_[2]); }

    inline void error(const double err) { error_ += err; }
    inline void errorUpdate() { q(q() + error_); error_=0.0f; }

    inline double torque() const { return(tau_); }
    inline double torqueExternal() const { return(tauExternal_); }
    inline double torqueTotal() const { return(tau_+tauExternal_+tauKinematic_); }
    inline double torqueKinematic() const { return(tauKinematic_); }

    inline void torque(const double t) { tau_=t; }
    inline void torqueExternal(const double t) { tauExternal_ += t; }
    inline void torqueExternalZero() { tauExternal_=0.0f; }
    inline void torqueKinematic(const double t) { tauKinematic_ = t; }

    inline double position(cDynTime t) 
    {
        if (t != t_) lookup(t);
        return(q_[0]);
    }

    inline double velocity(cDynTime t) 
    {
        if (t != t_) lookup(t);
        return(q_[1]);
    }
    
    inline double acceleration(cDynTime t) 
    {
        if (t != t_) lookup(t);
        return(q_[2]);
    }

    cDynTime lastEqual(const double value,const double err, const cDynTime time, const int dir) const;

    inline size_t save(void* ptr=NULL, size_t=0);
    inline size_t restore(void* ptr, size_t=0, cDynTime offset=0);

#ifdef CDYN_DEBUG
    void display(const char* str);
#endif


    //----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //----------------------------------------------------------------------
private:
    
    DeVarStatus status_;

    cDynTime    t_;
    cDynVector3 q_;
    double	  error_;
    double   tau_;
    double	  tauExternal_;
    double	  tauKinematic_;

    cDynStateEntry* state_;

    cDynJointInterval* head_;
    cDynJointInterval* tail_;


    //----------------------------------------------------------------------
    // PRIVATE METHODS:
    //----------------------------------------------------------------------
    private:

    void lookup(cDynTime t);
};

//--------------------------------------------------------------------------

class cDynJointSphereVar
{
    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
public:

    cDynJointSphereVar(const char* str=NULL) 
    {
        status_=CDYN_VAR_FREE;
        t_=CDYN_TIME_UNINITIALIZED;
        q_.identity();
        v_.zero();
        a_.zero();
        error_.zero();
        errorExists_=false;
        tau_.zero();
        tauExternal_.zero();
        tauKinematic_.zero();
        tauTotal_.zero();
        state_=NULL;
        head_=tail_=NULL;
    }


    //----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //----------------------------------------------------------------------
public:
    void state(cDynState* s);
    inline cDynStateEntry* state() const { return(state_); }

    void backup(const cDynTime t);
    void update(const cDynTime t);
    void advance(const cDynTime t);
    
    inline const cDynQuaternion& q() const 
    {
        q_.set(
            state_->x(0),
            state_->x(1),
            state_->x(2),
            state_->x(3)
        );

        return(q_);
    };

    inline const cDynVector3& v() const 
    {
        v_.set(
            state_->x(4),
            state_->x(5),
            state_->x(6)
        );
        return(v_);
    }

    inline const cDynVector3& a() const 
    {
        a_.set(
            state_->dxdt(4),
            state_->dxdt(5),
            state_->dxdt(6)
        );
         return(a_);
    };

    inline void q(const cDynQuaternion& q) 
    {
        state_->x(0)=q[0];
        state_->x(1)=q[1];
        state_->x(2)=q[2];
        state_->x(3)=q[3];
    }

    inline void v(const cDynVector3& v) 
    {
        state_->x(4)=v[0];
        state_->x(5)=v[1];
        state_->x(6)=v[2];
        cDynQuaternion q;
        q.set(state_->x(0),state_->x(1),state_->x(2),state_->x(3));
        cDynQuaternion dq;
        dq.velocity(q, v);
        state_->dxdt(0)=dq[0]; 
        state_->dxdt(1)=dq[1]; 
        state_->dxdt(2)=dq[2]; 
        state_->dxdt(3)=dq[3];
    }

    inline void a(const cDynVector3& a) 
    {
        state_->dxdt(4)=a[0];
        state_->dxdt(5)=a[1];
        state_->dxdt(6)=a[2];
    }

    inline void qCurrent(const cDynQuaternion& q) { q_=q; }
    inline void vCurrent(const cDynVector3& v) { v_=v; }
    inline void aCurrent(const cDynVector3& a) { a_=a; }

    inline const cDynQuaternion& qCurrent() const { return(q_); }
    inline const cDynVector3& vCurrent() const { return(v_); }
    inline const cDynVector3& aCurrent() const { return(a_); }

    inline void error(const cDynVector3 err) { error_ += err; errorExists_=true; }
    void errorUpdate();

    inline const cDynVector3& torque() const { return(tau_); }
    inline const cDynVector3& torqueExternal() const { return(tauExternal_); }
    inline const cDynVector3& torqueTotal() { tauTotal_.add(tau_, tauExternal_); tauTotal_+=tauKinematic_; return(tauTotal_); }
    inline const cDynVector3& torqueKinematic() const { return(tauKinematic_); }
    
    inline void torque(const cDynVector3& tau) { tau_=tau; }
    inline void torqueExternal(const cDynVector3& tau) { tauExternal_ += tau; }
    inline void torqueExternalZero() { tauExternal_.zero(); }
    inline void torqueKinematic(const cDynVector3& tau) { tauKinematic_ = tau; }
    inline void torqueKinematicZero() { tauKinematic_.zero(); }

    inline const cDynQuaternion& position(const cDynTime t) const 
    {
        if (t != t_) lookup(t);
        return(q_);
    }
    
    inline  const cDynVector3& velocity(const cDynTime t) const
    {
        if (t != t_) lookup(t);
        return(v_);
    }
    
    inline const cDynVector3& acceleration(const cDynTime t) const
    {
        if (t != t_) lookup(t);
        return(a_);
    }

    size_t save(void* ptr=NULL, size_t=0);
    size_t restore(void* ptr, size_t=0, cDynTime offset=0);

#ifdef CDYN_DEBUG
    void display(const char* str);
#endif

    //----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //----------------------------------------------------------------------
private:

    friend cDynJointSphereInterval::cDynJointSphereInterval(cDynJointSphereVar* var, const cDynTime t_,const cDynQuaternion& q_, const cDynVector3& v_, const cDynVector3& a);
    friend cDynJointSphereInterval::~cDynJointSphereInterval();

    DeVarStatus status_;

    mutable cDynTime       t_;
    mutable cDynQuaternion q_;
    mutable cDynVector3    v_; 
    mutable cDynVector3    a_;

    cDynVector3 error_;
    bool errorExists_;
    cDynVector3 tau_;
    cDynVector3 tauExternal_;
    cDynVector3 tauKinematic_;
    cDynVector3	tauTotal_;

    cDynStateEntry* state_;

    cDynJointSphereInterval* head_;
    cDynJointSphereInterval* tail_;


    //----------------------------------------------------------------------
    // PRIVATE METHODS:
    //----------------------------------------------------------------------
private:

    void lookup(const cDynTime t) const;
};

//--------------------------------------------------------------------------

void* cDynJointInterval::operator new(size_t s)
{
    if (free_==NULL) 
    {
        cDynJointInterval *tmp = (cDynJointInterval *)malloc(s);
        return(tmp);
    }
    else 
    {
        cDynJointInterval* tmp=free_;
        free_=free_->next_; 
        return(tmp);
    }
}

//--------------------------------------------------------------------------

void cDynJointInterval::operator delete(void *ptr)
{
    cDynJointInterval* tmp=(cDynJointInterval *)ptr;
    tmp->next_=free_;
    free_=tmp;
}

//--------------------------------------------------------------------------
#endif
//--------------------------------------------------------------------------
#endif
//--------------------------------------------------------------------------