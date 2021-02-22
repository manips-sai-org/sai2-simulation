//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynJointH
#define CDynJointH
//---------------------------------------------------------------------------
#include "global/CDynGlobalDefn.h"
#include "matrix/CDynMathDefn.h"
#include "matrix/CDynVector6.h"
#include "var/CDynVar.h"
#include "dynamics/CDynDynamics.h"
//---------------------------------------------------------------------------

enum cDynJointType { CDYN_PRISMATIC = 0, 
                     CDYN_REVOLUTE  = 1, 
                     CDYN_SPHERICAL = 2 };

enum cDynBoundType { CDYN_LOWER = 0, 
                     CDYN_UPPER = 1 };

//---------------------------------------------------------------------------
const double CDYN_NOBOUND = FLT_MAX; // XXXX FIX
const double CDYN_MIN_JOINT_ERROR = 1e-8f;
//---------------------------------------------------------------------------
class cDynObject;
class cDynBaseNode;
class cDynPrim;
class cDynBSphere;
class cDynGeometry;
class cDynJoint;
class cDynJointLimit;
class cDynCollision;
class cDynState;
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
/*
    NOTE:
    At some time in the future it may be desirable the cDynJoint be split 
    into two class cDynJoint and cDynSphereJoint which inherit some 
    attributes and methods from a common base class until then this will be 
    a little more painful then necessary
*/
//---------------------------------------------------------------------------

class cDynJoint
{
    friend class cDynJointList;
    friend class cDynJointLimitList;

    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
private:

    //! Constructor of cDynJoint.
    cDynJoint(cDynObject* obj,cDynJointType type, cDynAxis axis, char* data);
    cDynJoint(cDynObject* obj,cDynJointType type, char* data);

    //! Destructor of cDynJoint.
    virtual ~cDynJoint();
  

    //----------------------------------------------------------------------
    // PUBLIC METHODS:
    //----------------------------------------------------------------------
public:

    //! Return type of joint: \a CDYN_PRISMATIC, \a  CDYN_REVOLUTE or \a CDYN_SPHERICAL.
    cDynJointType type() const { return(type_); }

    //! Return axis of action.
    cDynAxis axis() const { return(axis_); }

    //! Return node to which joint is associated.
    cDynObject* object() const { return(obj_); }


    //----------------------------------------------------------------------
    // PUBLIC METHODS: (STATE PROPERTIES - REVOLUTE/PRISMATIC JOINTS)
    //----------------------------------------------------------------------
public:

    void position(double q);
    void velocity(double v);
    
    void torque(const double t) 
    {
        assert(q_); q_->torque(t);
    }
    
    double positionAt(const cDynTime time) 
    {
        assert(q_); return(q_->position(time));
    }
    
    double velocityAt(const cDynTime time) 
    {
        assert(q_); return(q_->velocity(time));
    }
    
    double torque() const 
    {
        assert(q_); return(q_->torque());
    }


    //----------------------------------------------------------------------
    // PUBLIC METHODS: (STATE PROPERTIES - SPHERICAL JOINTS)
    //----------------------------------------------------------------------
public:

    void positionSpherical(cDynQuaternion& q);
    void velocitySpherical(cDynVector3& v);

    void torqueSpherical(const cDynVector3& x) 
    {
        assert(sq_); sq_->torque(x);
    }
    
    const cDynQuaternion& positionSphericalAt(const cDynTime time) 
    {
        assert(sq_); return(sq_->position(time));
    }
    
    const cDynVector3& velocitySphericalAt(const cDynTime time) 
    {
        assert(sq_); return(sq_->velocity(time));
    }
    
    const cDynVector3& torqueSpherical() const 
    {
        assert(sq_); return(sq_->torque());
    }


    //----------------------------------------------------------------------
    // PUBLIC METHODS: (MOTOR INERTIA/DAMPING)
    //----------------------------------------------------------------------
public:
    
    void inertia(double Im);
    double inertia() { return(inertia_); }
    
    void damping(double b);
    double damping() { return(damping_); }


    //----------------------------------------------------------------------
    // PUBLIC METHODS: (JOINT LIMITS)
    //----------------------------------------------------------------------
public:

    //! functions to get current joint limit, limit error, and limit elasticity.
    double bound(cDynBoundType type) const;
    double error(cDynBoundType type) const;
    double epsilon(cDynBoundType type) const;


    //----------------------------------------------------------------------
    // PUBLIC METHODS: (JOINT LIMITS)
    //----------------------------------------------------------------------
public:

    //! Functions to set joint limit and joint limit properties.
    void bound(cDynBoundType type, double value);
    void unbound(cDynBoundType type);

    void bound(double lower, double upper, double err) 
    {
        bound(CDYN_LOWER,lower); error(CDYN_LOWER,err);
        bound(CDYN_UPPER,upper); error(CDYN_UPPER,err);
    }
    
    void error(cDynBoundType type, double err);
    void epsilon(cDynBoundType type, double e);

    //! Function to mark joint limit as invalid (will not be active until position within joint limit.
    void invalid(const cDynBoundType type);


    //----------------------------------------------------------------------
    // PUBLIC METHODS: (LOW LEVEL - PRISMATIC/REVOLUTE JOINTS)
    //----------------------------------------------------------------------
public:

    //----------------------------------------------------------------------
    /* 
        Low-level methods to get/set current state of prismatic/revolute 
        joints (use with caution)
    */
    //----------------------------------------------------------------------

    cDynJointVar *var() const { return(q_); }
    
    double q() const 
    {
        assert(q_); return(q_->q());
    }
    
    double v() const 
    {
        assert(q_); return(q_->v());
    }
    
    double a() const 
    {
        assert(q_); return(q_->a());
    }
    
    void q(const double x) 
    {
        assert(q_); q_->q(x);
    }
    
    void v(const double x) 
    {
        assert(q_); q_->v(x);
    }
    
    void a(const double x) 
    {
        assert(q_); q_->a(x);
    }
    
    void qerr(const double err) 
    {
        assert(q_); q_->error(err);
    }


    //----------------------------------------------------------------------
    // PUBLIC METHODS: (LOW LEVEL - SPHERICAL JOINTS)
    //----------------------------------------------------------------------
public:

    //----------------------------------------------------------------------
    /* 
        Low-level methods to get/set current state of spherical joints 
        (use with caution)
    */
    //----------------------------------------------------------------------

    cDynJointSphereVar *svar() const { return(sq_); }

    const cDynQuaternion& sq() const 
    {
        assert(sq_); return(sq_->q());
    }
    
    const cDynVector3& sv() const 
    {
        assert(sq_); return(sq_->v());
    }
    
    const cDynVector3& sa() const 
    {
        assert(sq_); return(sq_->a());
    }
    
    void sq(const cDynQuaternion& x) 
    {
        assert(sq_); sq_->q(x);
    }
    
    void sv(const cDynVector3& x) 
    {
        assert(sq_); sq_->v(x);
    }
    
    void sa(const cDynVector3& x) 
    {
        assert(sq_); sq_->a(x);
    }

    void sqerr(const cDynVector3& err) 
    {
        assert(sq_); sq_->error(err);
    }


    //----------------------------------------------------------------------
    // PUBLIC METHODS: (NAVIGATION)
    //----------------------------------------------------------------------
public:

    cDynJoint* next() const { return(next_); }
    cDynJoint* prev() const { return(prev_); }


    //----------------------------------------------------------------------
    // PUBLIC METHODS:
    //----------------------------------------------------------------------
public:

#ifndef CDYN_DONT_DOCUMENT
    double torqueTotal() 
    {
        assert(q_); return(q_->torqueTotal());
    }
    
    double torqueExternal() const 
    {
        assert(q_); return(q_->torqueExternal());
    }
    
    void torqueExternal(const double x) 
    {
        assert(q_); q_->torqueExternal(x);
    }
    
    void torqueExternalZero() 
    {
        assert(q_); q_->torqueExternalZero();
    }
    
    void torqueKinematic(const double x) 
    {
        assert(q_); q_->torqueKinematic(x);
    }
    
    double torqueKinematic() const 
    {
        assert(q_); return(q_->torqueKinematic());
    }

    const cDynVector3& storqueTotal() const 
    {
        assert(sq_); return(sq_->torqueTotal());
    }
    
    const cDynVector3& storqueExternal() const 
    {
        assert(sq_); return(sq_->torqueExternal());
    }
    
    void storqueExternal(const cDynVector3& x) 
    {
        assert(sq_); sq_->torqueExternal(x);
    }
    
    void storqueExternalZero() 
    {
        assert(sq_); sq_->torqueExternalZero();
    }

    void storqueKinematic(const cDynVector3& x) 
    {
        assert(sq_); sq_->torqueKinematic(x);
    }
    
    const cDynVector3& storqueKinematic() const 
    {
        assert(sq_); return(sq_->torqueKinematic());
    }

    // functions to get and set current state context
    cDynStateEntry* state();
    void state(cDynState* s);

    cDynTime lastEqual(const double value, const double err, const cDynTime time, const int dir) const 
    {
        assert(q_);
        return(q_->lastEqual(value,err,time,dir));
    }

    cDynTime lastEqual(const double value, const double err, const cDynTime time) 
    {
        assert(q_);
        double q=positionAt(time);

        if (q < value-err)
                return(q_->lastEqual(value,err,time,-1));
        else if (q > value+err)
                return(q_->lastEqual(value,err,time, 1));
        else return(time);
    }

    cDynVector6& jcol() { return(*jcol_); }
    cDynVector6& hcol() { return(*hcol_); }
    cDynMatrix3* jsphere() { return(jsphere_); }

    double& tmp() { return(tmp_); }
    cDynVector3& tmp3() { return(*tmp3_); }

    void rotateHome2Local(cDynVector3& res, const cDynVector3& x) { res.inversedMultiply((localFrame().rotation()) , x); }
    void rotateLocal2Home(cDynVector3& res, const cDynVector3& x) { res.multiply((localFrame().rotation()), x); }
    void rotateHome2Local(cDynQuaternion& res, const cDynQuaternion& x) { res.inversedMultiply((localFrame().rotation()), x); }

#endif

    cDynFrame &localFrame() { return localFrame_; }
    void updateLocalTransformation(const cDynTime *time);

    //! Methods for specifying user defined data
    void *data() const { return(data_); }
    void data(void *d) { data_=d; }

    void *controlParam() { return controlParam_; }
    void controlParam(void *p) { controlParam_ = p; }

    cDynTransform& localX() { return _localX; }


    //----------------------------------------------------------------------
    // PRIVATE METHODS:
    //----------------------------------------------------------------------
private:

    //! Only used by cDynJointList
    void insert(cDynJoint** head, cDynJoint** tail); 

    //! Only used by cDynJointList
    void remove(cDynJoint** head, cDynJoint** tail); 

    cDynJointType type_;
    cDynAxis axis_;

    cDynJointVar*   q_;
    cDynJointSphereVar*   sq_;

    //! Pointer to joint limit record, NULL if no limit
    cDynJointLimit* bound_[2]; 
    double damping_;
    double inertia_;
    
    //! Column of global Jacobian for 1 dof joints
    cDynVector6* jcol_;     

    //! Column of global Hessian
    cDynVector6* hcol_;     

    //! 3 columns of global Jacobian for spherical joints.
    cDynMatrix3* jsphere_;  

    double tmp_;
    cDynVector3* tmp3_;

    cDynObject* obj_;

    //! BaseNode.
    cDynFrame localFrame_;  

    //! User data
    void *data_;  

    //! Control parameters
    void *controlParam_; 

    cDynTransform _localX;

    //! Next joint in series
    cDynJoint *prev_;
    cDynJoint *next_;
};

//---------------------------------------------------------------------------

class cDynJointList
{
    friend class cDynObject;

    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
private:

    //! Constructor of cDynJointList.
    cDynJointList();

    //! Destructor of cDynJointList.
    ~cDynJointList();


    //----------------------------------------------------------------------
    // PUBLIC METHODS:
    //----------------------------------------------------------------------
public:

    //----------------------------------------------------------------------
    /*!
        Create a revolute (hinge) joint between an object and its parent.
        multiple joints may be defined between an object and its parent so
        long as no singularity is introduced. A newly created hinge is defined
        to exist between any previously defined joint and the object.
        
        \param axis Axis about which the joint is allowed to rotate, can be
               any one of the following: \a CDYN_AXIS_X, \a CDYN_AXIS_Y, \a CDYN_AXIS_Z.
        \param data user defined data. returned by \a joint->data(). not used by the engine.
        
        \returns pointer to created joint of type cDynJoint.
    */
    //----------------------------------------------------------------------
    cDynJoint* revolute(cDynAxis axis, char* data=NULL);


    //----------------------------------------------------------------------
    /*!
        Create a prismatic (translational) joint between an object and its 
        parent. Multiple joints may be defined between an object and its 
        parent so long as no singularity is introduced. A newly created joint 
        is defined to exist between any previously defined joint and the object.

        \param axis Axis about which the joint is allowed to move, can be
                    any one of the following: \a CDYN_AXIS_X, \a CDYN_AXIS_Y, 
                    \a CDYN_AXIS_Z.
        \param data User defined data. returned by \a joint->data(). 
                    Not used by the engine.
    
        \return Returns pointer to created joint of type cDynJoint.
    */
    //----------------------------------------------------------------------
    cDynJoint* prismatic(cDynAxis axis, char* data=NULL);


    //----------------------------------------------------------------------
    /*!
        Create a spherical (ball) joint between an object and its parent.
        Only one spherical joint can exist between two objects without creating
        a singularity. A spherical joint can only be defined with at most three
        prismatic joints without creating a singularity.  A newly created 
        spherical joint is defined to exist between any previously defined 
        joint and the object.
    
        \param data user defined data. returned by \a joint->data(). 
                    not used by the engine.
    
        \return  Return pointer to created joint of type cDynJoint.
    */
    //----------------------------------------------------------------------
    cDynJoint* spherical(char* data=NULL);


    //----------------------------------------------------------------------
    /*!
        Remove a given joint \a joint from the list of joints between a pair 
        of objects. If the joint does not exist, fail on assertion.
    
        \note removing a joint on an active group whose position is non zero
        (identity in the case of a spherical joint) will cause an 
        instantaneous change.
    */
    //----------------------------------------------------------------------
    void remove(cDynJoint* joint);




    //----------------------------------------------------------------------
    /*!
        Deletes any joints defined on list. A fixed link would exist between 
        an object and its parent. Equalent to \code while (head()) 
        remove(head()); \endcode.
    */
    //----------------------------------------------------------------------
    void reset();


    //----------------------------------------------------------------------
    /*!
        Returns the first defined joint between the object and parent. 
        This joint is the closest to the parents node in the tree hierarchy.
    */
    //----------------------------------------------------------------------
    cDynJoint* head() { return(head_); }


    //----------------------------------------------------------------------
    /*!
        Returns the last defined joint between the object and parent. This joint is
        the nearest to the objects node in the tree hierarchy.
    */
    //----------------------------------------------------------------------
    cDynJoint* tail() { return(tail_); }
    

    //----------------------------------------------------------------------
    /*!
        Return object associated with this list.
    */
    //----------------------------------------------------------------------
    cDynObject* object() { return(obj_); }


    //----------------------------------------------------------------------
    // PRIVATE METHODS:
    //----------------------------------------------------------------------
private:

    void object(cDynObject* obj) { obj_=obj; }


    //----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //----------------------------------------------------------------------
private:

    cDynObject* obj_;

    //! Joint list;
    cDynJoint* head_;
    cDynJoint* tail_;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------