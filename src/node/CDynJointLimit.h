//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynJointLimitH
#define CDynJointLimitH
//---------------------------------------------------------------------------
#ifndef CDYN_DONT_DOCUMENT
//---------------------------------------------------------------------------
#include "object/CDynJoint.h"
//---------------------------------------------------------------------------
int const CDYN_LIMIT_ACTIVE=0;
int const CDYN_LIMIT_INVALID=1;
int const CDYN_LIMIT_NUMSTATES=2;
//---------------------------------------------------------------------------
class cDynJointLimitList;
//---------------------------------------------------------------------------
class cDynJointLimit
{
    friend class cDynJointLimitList;

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
public:

    //! Constructor of cDynJointLimit.    
    cDynJointLimit(int type, 
        double value, 
        double err, 
        double epsilon, 
        cDynJoint* joint): type_(type), state_(-1), value_(value), err_(err), epsilon_(epsilon), joint_(joint), list_(NULL), prev_(NULL), next_(NULL) {}
    
    
    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //-----------------------------------------------------------------------
public:    
    
    int type() const { return(type_); }

    int state() const { return(state_); }
    
    void  value(const double value) { value_=value; }
    double value() const { return(value_); }
    
    void  error(const double error) { err_=error; }
    double error() const { return(err_); }
    
    void  epsilon(const double epsilon) { epsilon_=epsilon; }
    double epsilon() const { return(epsilon_); }

    cDynJoint* joint() const { return(joint_); }

    int check(cDynTime t) 
    {
        double x=joint_->positionAt(t);
        if (type_ < 0) 
        {
            if (x > value_ + err_) return(1);
            if (x < value_ - err_) return(-1);
            return(0);
        } 
        else 
        {
            if (x < value_ - err_) return(1);
            if (x > value_ + err_) return(-1);
            return(0);
        }
    }

    
    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
private:

    //! -1 == lower bound, 1 == upper bound
    int type_;	
    int state_;	
    double value_;
    double err_;
    double epsilon_;

    //! Joint to which bound refers to
    cDynJoint* joint_;	

    //! List if any where limit is attached.
    cDynJointLimitList* list_; 

    //! Linked list pointers
    cDynJointLimit* prev_; 	
    cDynJointLimit* next_;
};

//---------------------------------------------------------------------------

class cDynJointLimitList
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
public:

    //! Constructor of cDynJointLimitList.   
    cDynJointLimitList(int state): state_(state), head_(NULL), tail_(NULL) {};


    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    void pop(cDynJointLimit* limit);
    void insert(cDynJointLimit* limit);
    void remove(cDynJointLimit* limit);
    void insert(cDynJoint* joint);
    cDynJointLimit* next(cDynJointLimit* limit=NULL) 
    {
        if (limit == NULL) return(head_);
        else return(limit->next_);
    }

    
    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
private:

    int state_;
    cDynJointLimit* head_;
    cDynJointLimit* tail_;
};

#endif
//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------