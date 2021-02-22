//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynStateH
#define CDynStateH
//---------------------------------------------------------------------------
#ifndef CDYN_DONT_DOCUMENT
//---------------------------------------------------------------------------
#include "global/CDynGlobalDefn.h"
#include "matrix/CDynMathDefn.h"
#include <cassert>
//---------------------------------------------------------------------------
class cDynVar;
class cDynJointVar;
class cDynJointSphereVar;
class cDynStateEntry;
class cDynBaseNode;
//---------------------------------------------------------------------------

class cDynState
{
    friend class cDynBaseNode;

    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
public:

    cDynState(const int size=8);
    ~cDynState();
    

    //----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //----------------------------------------------------------------------
public:

    inline cDynStateEntry* insert(cDynJointVar* var);
    inline cDynStateEntry* insert(cDynJointSphereVar* var);

    void remove(cDynStateEntry* entry);

    inline double& x(int i)
    {
        assert(i>=0 && i<n_);
        return(x_[i]);
    }
    
    inline double& dxdt(int i) 
    {
        assert(i>=0 && i<n_);
        return(dxdt_[i]);
    }

    //! Update dxdt and renomalize
    void update(const bool normalize, const bool err);	

    //! Backup f-curve
    void backup(const cDynTime& time);

    //! Update f-curve
    void update(const cDynTime& time);
    void advance(const cDynTime& time);

    //! Zero external torques
    void torqueExternalZero();

    //! Used to signal a change of variables
    inline void mark() { mark_++; }
    inline void unmark() { mark_=0; }
    inline int ismark() const { return(mark_); }

    bool check();


    //----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //----------------------------------------------------------------------
private:

    void grow(const int size);
    cDynStateEntry* insert(cDynVar* var, int s);

    //! Integrator type.
    int type_;	

    //! Size of full vector
    int size_;	

    //! Size used for state vector
    int n_;		

    //! State information
    double* x_;
    double* dxdt_;

    //! Indicate if there has been a change of variables
    int mark_;	

    //! Linked list of entries
    cDynStateEntry* head_;
    cDynStateEntry* tail_;
};

//--------------------------------------------------------------------------

class cDynStateEntry 
{
    friend class cDynState;
    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
public:

    cDynStateEntry(): ptr_(NULL), state_(NULL), index_(0), num_(0), next_(NULL), prev_(NULL) {}
    ~cDynStateEntry() { if (state_ != NULL) state_->remove(this); }
    

    //----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //----------------------------------------------------------------------
    inline double& x(int i) 
    {
        assert(i>=0 && i<num_);
        return(state_->x(index_+i));
    }
    
    inline double& dxdt(int i) 
    {
        assert(i>=0 && i<num_);
        return(state_->dxdt(index_+i));
    }
    
    inline cDynState* state() const { return(state_); }


    //----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //----------------------------------------------------------------------
private:

    cDynVar* ptr_;
    cDynState* state_;
    int index_;
    int num_;

    cDynStateEntry* next_;
    cDynStateEntry* prev_;
};

//---------------------------------------------------------------------------

inline cDynStateEntry* cDynState::insert(cDynJointVar* var)
{
    return(insert((cDynVar *)var, 2));
}

//---------------------------------------------------------------------------

inline cDynStateEntry* cDynState::insert(cDynJointSphereVar* var)
{
    return(insert((cDynVar *)var, 7));
}

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
#endif // CDynStateH
//---------------------------------------------------------------------------
