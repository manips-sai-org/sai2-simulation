//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynTimeEventH
#define CDynTimeEventH
//---------------------------------------------------------------------------
#include "matrix/CDynMathDefn.h"
#include "global/CDynGlobalDefn.h"
//---------------------------------------------------------------------------
//#define key(x)	(heap_[x]->time())
//---------------------------------------------------------------------------
const int CDYN_TIMEEVENT_FOREVER=0;
//---------------------------------------------------------------------------
static const int maxsize=1024;
//---------------------------------------------------------------------------
class cDynTimeEvent;
//---------------------------------------------------------------------------

class cDynTimeHeap
{
    friend class cDynTimeEvent;

    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
public:
    cDynTimeHeap(int size=maxsize): size_(0), maxsize_(size), active_(NULL)
    { heap_= new cDynTimeEvent*[size+1]; heap_[0]=NULL; 
    //	if (cDynDebugLevel == -100)
    //		cDynPrintf("cDynTimeHeap\n");
    }
    ~cDynTimeHeap() { delete heap_; }


    //----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //----------------------------------------------------------------------
public:

    void insert(cDynTimeEvent* event);
    void remove(cDynTimeEvent* event);
    void update(cDynTimeEvent* event);
    cDynTimeEvent* top() { return(heap_[1]); }
    void next();			//advance one step
    void advance(const cDynTime time);	//advance until given time
    void advance(const int iter);		//advance i steps

    void debug();


    //----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //----------------------------------------------------------------------
private:

    int parent(int x) { return(x>>1); }
    int left(int x) { return(x<<1); }
    int right(int x) { return((x<<1)+1); }
    
    inline cDynTime key(int x); // { return(heap_[x]->time()); } NEED TO FIX

    inline cDynTimeEvent* at(int x);
    inline bool less(cDynTimeEvent* event1, cDynTimeEvent* event2);
    inline bool greater(cDynTimeEvent* event1, cDynTimeEvent* event2);

    void Heapify(int i);

    // An array of heap pointers: FIX.
    cDynTimeEvent** heap_;

    // Size of current heap.
    int size_;	     

    // Maximum number of events.
    int maxsize_; 	     

    // Currently active call back event.
    cDynTimeEvent*	active_; 
};

//---------------------------------------------------------------------------

class cDynTimeEvent
{
    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
public:

    cDynTimeEvent(): heap_(NULL) {}
    
    cDynTimeEvent(const cDynTime time,const cDynTime inc,const int count, void (*f)(cDynTimeEvent* event, void* arg), void *arg=NULL)
    : time_(time), priority_(0), inc_(inc), count_(count), func_(f), arg_(arg), heap_(NULL) 
    { if (inc_ < 0.0) { inc_=0; count_=1;} }
    
    cDynTimeEvent(const cDynTime time,const int priority,const cDynTime inc,const int count, void (*f)(cDynTimeEvent* event, void* arg), void *arg=NULL)
    : time_(time), priority_(priority), inc_(inc), count_(count), func_(f), arg_(arg), heap_(NULL) 
    { if (inc_ < 0.0) { inc_=0; count_=1;} }
    
    ~cDynTimeEvent() { if (heap_ != NULL) heap_->remove(this); }


    //----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //----------------------------------------------------------------------
public:
    void insert(cDynTimeHeap* heap) { heap->insert(this); }
    void remove() { if (heap_ != NULL) heap_->remove(this); }
    void time(const cDynTime t) { time_=t; if (heap_ != NULL) heap_->update(this); }
    void priority(const int p) { priority_=p; if (heap_ != NULL) heap_->update(this); }
    void increment(const cDynTime inc) 
    { 
        inc_=inc;
        if (inc_ < 0.0) { inc_=0; count_=1;}
    }
    void iterations(const int count) { count_=count; }
    void always() { count_=0; }
    void callback(void (*f)(cDynTimeEvent* event, void* arg)) { func_=f; }
    void arg(void *arg) { arg_=arg; }
  
    const cDynTime& time() const {return(time_);}
    const cDynTime& increment() const {return(inc_);}
    int priority() const {return(priority_);}
    const cDynTime& inc() const {return(inc_);}
    int iterations() const {return(count_);}

    int attached() const { return(heap_ != NULL); }

    void callback() { if (func_ != NULL) (*func_)(this,arg_); }

    friend void cDynTimeHeap::Heapify(int i);
    friend void cDynTimeHeap::insert(cDynTimeEvent* event);
    friend void cDynTimeHeap::remove(cDynTimeEvent* event);
    friend void cDynTimeHeap::update(cDynTimeEvent* event);
    friend void cDynTimeHeap::next();
    friend void cDynTimeHeap::debug();

    
    //----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //----------------------------------------------------------------------
private:

    //! Time of next event.
    cDynTime time_;

    //! Priority of event (only used in case of tie).
    int priority_; 

    //! For repeating events interval between events.
    cDynTime inc_;

    //! Number of times event is called before removing from heap. count=0 means never.
    int count_;	

    //! Callback function when event is triggered
    void (*func_)(cDynTimeEvent* event, void* arg);
    
    //! Argument to callback function (can be anything)
    void *arg_;
  
    //! Heap on which event is attached if any
    cDynTimeHeap* heap_;

    //! Current position in heap
    int index_; 
};
  
//---------------------------------------------------------------------------

inline cDynTime cDynTimeHeap::key(int x)
{ 
    return(heap_[x]->time()); 
}

//---------------------------------------------------------------------------

inline cDynTimeEvent* cDynTimeHeap::at(int x)
{
    return(heap_[x]);
}

//---------------------------------------------------------------------------

inline bool cDynTimeHeap::less(cDynTimeEvent* event1, cDynTimeEvent* event2)
{
    if (event1->time() == event2->time()) return(event1->priority() < event2->priority());
    else return(event1->time() < event2->time());
}

//---------------------------------------------------------------------------

inline bool cDynTimeHeap::greater(cDynTimeEvent* event1, cDynTimeEvent* event2)
{
    if (event1->time() == event2->time()) return(event1->priority() > event2->priority());
    else return(event1->time() > event2->time());
}

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------



