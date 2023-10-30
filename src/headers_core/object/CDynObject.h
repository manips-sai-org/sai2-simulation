//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynObjectH
#define CDynObjectH
//---------------------------------------------------------------------------
#include "global/CDynGlobalDefn.h"
#include "matrix/CDynMathDefn.h"
#include "matrix/CDynFrameStack.h"
#include "object/CDynMass.h"
#include "object/CDynGeometry.h"
#include "object/CDynJoint.h"
#include "object/CDynForce.h"
#include "var/CDynVar.h"
#include "dynamics/CDynDynamics.h"
//---------------------------------------------------------------------------
#include "dynamics/CDynABNode.h"
//---------------------------------------------------------------------------
class cDynBaseNode;
class cDynPrim;
class cDynBSphere;
class cDynCollision;
class cDynState;
class cDynForceProperty;
class cDynVector6;
//---------------------------------------------------------------------------

class cDynObject
{
    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
public:
    
    //! Constructor of cDynObject. Specify a single rigid body.
    cDynObject(void *d = NULL);

    //! Constructor of cDynObject. Specify a single rigid body.	
    cDynObject(char *d);

    //! Destructor of cDynObject.
    virtual ~cDynObject();


    //----------------------------------------------------------------------
    // PUBLIC METHODS:
    //----------------------------------------------------------------------
public:

    bool isRoot() 
    { 
        return (parent_ == NULL); 
    }
    
    bool isParentRoot() 
    { 
        return (isRoot() || parent_->isRoot()); 
    }

    //! Reference frame used when specifying geometry/dynamics and links.
    cDynFrameStack frame;
        
    //! Geometry class to store object geometry used for collision testing.
    cDynGeometry geometry;
        
    //! Class to specify the mass parameters for an object.
    cDynMass dynamics;
        
    //! Class to specify a serial set of joints to parent
    cDynJointList  joint;
        
    //! Class to specify a force properties applied to object
    cDynForce	force;
    
    //! Specify \a obj as a child of object.
    void link(cDynObject* obj);

    //! Orphan \a obj as a child of object.
    void unlink(cDynObject* obj);
        
    //! Orphan oneself from parent.
    void unlink();

    //! Return the home frame of the object. Defined as current frame of parent when object was linked to it by link().
    cDynFrame& homeFrame() { return(homeFrame_); }

    //! Return parent of object. \a NULL if not linked.
    cDynObject* parent() { return(parent_); }
        
    //! Return first child of object. \a NULL if no children exist.
    cDynObject* child() { return(child_); }

    //! Return next sibling of object. \a NULL if last sibling.	
    cDynObject* sibling() { return(sibling_); }

    cDynMass*  mass() { return(&dynamics); }
    void baseSet(cDynBaseNode* base,bool tree);
    void collisionUpdateTree(const cDynTime& min, const cDynTime& max);

    bool isFixed() const { return(fixed_); }
    void fixed(bool value) { fixed_=value; }

    // Velocity in local frame
    const cDynVector6& velocity() { return *abNode()->V(); }

    // Acceleration in local frame
    const cDynVector6& acceleration() { return *abNode()->A(); }

    //! Return current working frames/velocity
    cDynFrame &globalFrame() { return globalFrame_; };
    const cDynFrame &globalFrame(const cDynTime& time);

    void updateGlobalTransformation();

    long &callnum() { return callnum_; }

    unsigned long uid() const { return(uid_); }

    cDynBaseNode* baseNode() { return(baseNode_); }
    void baseNode(cDynBaseNode* base) { baseNode_ = base; }

    //! Methods for specifying user defined data
    void *data() const { return(data_); }
    void data(void *d) { data_=d; }

    cDynABNode* abNode() { return _abNode; }
    void abNode(cDynABNode* node) { _abNode = node; }

    cDynTransform& localX() { return _localX; }


    //----------------------------------------------------------------------
    // PRIVATE METHODS:
    //----------------------------------------------------------------------
private:

    cDynABNode* _abNode;

    void initialize(void *d);
    bool fixed_;

    //! Need to see about moving this pointer to cDynGeometry, but harder then it looks
    cDynCollision* collision_;

    cDynFrame homeFrame_;
    
    //! (global|local)Frame valid if matches with
    cDynFrame globalFrame_; 

    // globalFrame and localFrame are up to date.
    long callnum_;	

    static unsigned long UID_;
    unsigned long uid_;
    
    // Base node.
    cDynBaseNode *baseNode_;

    // User data.
    void *data_;  

    // Control parameters.
    void *controlParam_; 
    cDynTransform _localX;

    cDynObject* child_;
    cDynObject* sibling_;
    cDynObject* parent_;
        
    void _UpdateTransformationPath(const cDynTime *time);
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------