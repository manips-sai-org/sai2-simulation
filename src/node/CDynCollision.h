//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynCollisionH
#define CDynCollisionH
//---------------------------------------------------------------------------
#include "global/CDynGlobalDefn.h"
#include "matrix/CDynMathDefn.h"
#include "matrix/CDynFrame.h"
#include "matrix/CDynVector3.h"
//---------------------------------------------------------------------------
#ifndef CDYN_DONT_DOCUMENT
//---------------------------------------------------------------------------
class cDynBaseNode;
class cDynObject;
//---------------------------------------------------------------------------
#define CDYN_BOUND_HISTORY_SIZE	4
//---------------------------------------------------------------------------
enum cDynTestOption { CDYN_TESTOPTION_VALID, CDYN_TESTOPTION_IGNORE, CDYN_TESTOPTION_PRINT };
//---------------------------------------------------------------------------
// Size of pair hash.
const int DEPAIR_HASHSIZE = 4095;
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
/*!
    Pair States:

    CDYN_PAIR_IGNORE:  
    List of pairs which are to be ignored

    CDYN_PAIR_INACTIVE:
    List of pairs where 1 or 2 axes intersect
    
    CDYN_PAIR_ACTIVE:  
    List of pairs where bounding boxes intersect (3 axes)

    CDYN_PAIR_INVALID: 
    Violating constraints, tested periodically, when constraint no longer 
    invalid then moved to ACTIVE/INACTIVE state.

    CDYN_PAIR_SLEEP: 
    List of pairs in which both nodes are sleeping, returned to 
    ACTIVE/INACTIVE state when one of pair awakens.
*/
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
const int CDYN_PAIR_QUERY       = -1;
const int CDYN_PAIR_INACTIVE    =  0;
const int CDYN_PAIR_ACTIVE      =  1;
const int CDYN_PAIR_IGNORE      =  2;
const int CDYN_PAIR_INVALID     =  3;
const int CDYN_PAIR_SLEEP       =  4;
const int CDYN_PAIR_NUMSTATES   =  5;
//---------------------------------------------------------------------------
class cDynBaseNode;
class cDynCollision;
class cDynNode;
class cDynPrim;
//---------------------------------------------------------------------------

class cDynPair
{
    friend class cDynCollision;

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
private:

    //! Constructor of cDynPair.    
    cDynPair() 
    { 
        a_ = NULL;
        b_ = NULL;
        safe_ = -100;
        state_ = -100;
        count_ = -100;
        hash_ = NULL;
        prev_ = NULL;
        next_ = NULL;
    }

    //! Destructor of cDynPair.
    //~cDynPair();


    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    void flip() 
    {
        cDynObject* tmp=a_;a_=b_;b_=tmp;
    }


    //-----------------------------------------------------------------------
    // PRIVATE METHODS:
    //-----------------------------------------------------------------------
private:

    //! Node A in pair.
    cDynObject *a_;	

    //! Node B in pair.
    cDynObject *b_;

    //! Last safe time for pair.
    cDynTime safe_;

    //! Indicates which state list this pair belongs.
    int state_;	

    //! Number of intersecting axes (3 means bounding boxes intersect).
    int count_;	

    //! Hash table pointer.
    cDynPair *hash_;	

    //! Double linked pointers for state list.
    cDynPair *prev_;  
    cDynPair *next_;

    static cDynPair* free_;


    //-----------------------------------------------------------------------
    // OPERATORS:
    //-----------------------------------------------------------------------
public:

    void* operator new(size_t s) 
    {
        if (free_==NULL) 
        {
            cDynPair *tmp = (cDynPair *)malloc(s);
            return tmp;
        } 
        else 
        {
            cDynPair* tmp=free_;
            free_=free_->next_;
            return(tmp);
        }
    }

    void operator delete(void *ptr) 
    {
        cDynPair* tmp=(cDynPair *)ptr;
        tmp->next_=free_;
        free_=tmp;
    }
};

//---------------------------------------------------------------------------

class cDynBound
{
    friend class cDynCollision;

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
public:

    //! Constructor of cDynBound.    
    cDynBound()
    {
        type_ = 0;
        value_ = -100;
        index_ = -1;
        obj_ = NULL;
        
        left_ = NULL;
        right_ = NULL;
    }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    void update(double value, const cDynTime& tmin, const cDynTime& tmax);
    void set(double value, cDynTime& time);
    double value() const { return(value_); }


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
private:

    //void insertDirected(cDynBound* prev, cDynBound* next, int dir);

    //! -1 == lower bound, 1 == upper bound
    int type_;	
    double value_;

    //! Object of which bound refers to
    cDynObject* obj_;	
    
    
    // Primitive which witnesses the bound.
    // cDynPrim* prim_; 

    // Vertex index of witness int prim_.
    // int witness_;
    
    struct cDynHistory 
    {
        cDynTime time_;
        double value_;
    };

    cDynHistory history_[CDYN_BOUND_HISTORY_SIZE];

    //! History start and end position.
    int hs_;
    int he_;
    int index_;

    //! Linked list of bound for axes
    cDynBound* left_;
    cDynBound* right_;
};

//---------------------------------------------------------------------------
#undef min // XXX - bar
#undef max
//---------------------------------------------------------------------------

class cDynBoundRecord	// used in cDynGeometry
{
    friend class cDynCollision;

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
private:

    //! Constructor of cDynBoundRecord.    
    cDynBoundRecord() {}

public:

    //! Destructor of cDynBoundRecord.
    ~cDynBoundRecord() {}


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //-----------------------------------------------------------------------
public:

    void min(cDynVector3& m) { m.set(min_[0].value(),min_[1].value(),min_[2].value()); }
    void max(cDynVector3& m) { m.set(max_[0].value(),max_[1].value(),max_[2].value()); }
    

    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
private:

    cDynBound min_[3];
    cDynBound max_[3];
};

//---------------------------------------------------------------------------

class cDynCollision	// used in cDynWorld
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
public:

    //! Constructor of cDynCollision.    
    cDynCollision();

    //! Destructor of cDynCollision.
    ~cDynCollision();


    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    //! Methods to set or get the state of a pair of objects
    void ignore(cDynObject* a, cDynObject* b, const bool cond=true);
    void invalid(cDynObject* a, cDynObject* b, const bool cond=true);
    void activate(cDynObject* a, cDynObject* b, const bool cond=true);
    int pairState(cDynObject* a, cDynObject* b);

    //! Methods to add/remove/update the bounding regions for consideration by cDynCollision
    void add(cDynObject* obj);
    void remove(cDynObject* obj);
    void update(cDynObject* obj, const cDynTime& min, const cDynTime& max);

    //! Force an update of all objects
    void reset() { reset_=true; }

    int checkObjects(const cDynTime& time, cDynTestOption option, cDynBaseNode* a=NULL);
    void checkInvalids();
    void checkSleep(const cDynBaseNode* b);
    bool checkContacts(cDynPair* start=NULL, cDynPair* end=NULL, cDynBaseNode* a=NULL, cDynBaseNode* b=NULL);

#ifdef CDYN_DEBUG
    void display();
    void check();
#endif


    //-----------------------------------------------------------------------
    // PRIVATE METHODS:
    //-----------------------------------------------------------------------
private:

    void insert(const int axis, cDynBound* bound, cDynBound* prev, cDynBound* next, const int dir);
    cDynPair* pairUpdate(cDynObject* a, cDynObject* b, const int inc, const int state=0);
    void state(cDynPair *p, const int state); 
    void pop(cDynPair *p);

    bool reset_;

    //! Bound list for each axes.
    cDynBound* head_[3];
    cDynBound* tail_[3];

    //! Hash list of node pairs.
    cDynPair* hash_[DEPAIR_HASHSIZE];
    
    //! List of node pairs belonging to different state.
    int num_[CDYN_PAIR_NUMSTATES];
    cDynPair* list_[CDYN_PAIR_NUMSTATES];
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------