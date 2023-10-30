//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynQEventH
#define CDynQEventH
//---------------------------------------------------------------------------
#include <stdlib.h>
#include "global/CDynGlobalDefn.h"
#include "matrix/CDynMathDefn.h"
#include "utility/CDynLogger.h"
//---------------------------------------------------------------------------
class cDynQEvent;
//---------------------------------------------------------------------------

class cDynQHeader
{
    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
public:

    cDynQHeader(): head_(NULL), tail_(NULL) {}
    ~cDynQHeader() { while (head_ != NULL) remove(head_); }


    //----------------------------------------------------------------------
    // PUBLIC METHODS:
    //----------------------------------------------------------------------
public:

    void advance(const cDynTime& time);
    void backup(const cDynTime& time);
    
    cDynQEvent* event(const cDynTime& time, 
        void (*forward)(const cDynTime& time, void* arg),
        void (*backward)(const cDynTime& time, void* arg),
        void* arg=NULL
    );
    
    cDynQEvent* find(const cDynTime& time, void* arg=NULL);
    void remove(cDynQEvent* event);


    //----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //----------------------------------------------------------------------
private:

    cDynQEvent* head_;
    cDynQEvent* tail_;
};

//---------------------------------------------------------------------------

class cDynQEvent
{
    friend class cDynQHeader;

    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
private:

    cDynQEvent(const cDynTime& time,
        void (*f)(const cDynTime& time, void* arg),
        void (*b)(const cDynTime& time, void* arg),
        void* arg=NULL
        ) : time_(time), arg_(arg), forward_(f), backward_(b), prev_(NULL), next_(NULL) {}

    ~cDynQEvent() {}


    //----------------------------------------------------------------------
    // OPERATORS:
    //----------------------------------------------------------------------
public:

    void* operator new(size_t s) 
    {
        if (free_==NULL) 
        {
            cDynQEvent *tmp = (cDynQEvent *)malloc(s);
            return(tmp);
        } 
        else 
        {
            cDynQEvent* tmp=free_;
            free_=free_->next_;
            return(tmp);
        }
    }

    void operator delete(void *ptr) 
    {
        cDynQEvent* tmp=(cDynQEvent *)ptr;
        tmp->next_=free_;
        free_=tmp;
    }


    //----------------------------------------------------------------------
    // PUBLIC METHODS:
    //----------------------------------------------------------------------
public:

    inline const cDynTime& time(void) const { return(time_); }
    inline void forward() { if (forward_ != NULL) (*forward_)(time_,arg_); }
    inline void backward() { if (backward_ != NULL) (*backward_)(time_,arg_); }
    

    //----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //----------------------------------------------------------------------
private:

    //! Time of event.
    cDynTime time_;

    //! User supplied arguments.
    void* arg_;  

    //! Pointers to link list of cDynQEvents.
    cDynQEvent* prev_; 
    cDynQEvent* next_;


    //----------------------------------------------------------------------
    // PRIVATE METHODS:
    //----------------------------------------------------------------------
public:

    //! Callback on advance.
    void (*forward_)(const cDynTime& time, void* arg);  

    //! Callback on backup.
    void (*backward_)(const cDynTime& time, void* arg); 


    //----------------------------------------------------------------------
    // STATIC MEMBERS:
    //----------------------------------------------------------------------
private:

    // Linked list of free cDynQEvent.
    static cDynQEvent* free_; 
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------