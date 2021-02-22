//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynPropertyH
#define CDynPropertyH
//---------------------------------------------------------------------------

class cDynProperty
{
    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
public:

    //! Constructor of cDynProperty.
    cDynProperty(cDynProperty** head=NULL) { if (head != NULL) insert(head); }
    
    //! Destructor of cDynProperty.
    virtual ~cDynProperty() {}


    //----------------------------------------------------------------------
    // PUBLIC METHODS:
    //----------------------------------------------------------------------
public:
        void insert(cDynProperty** head) { next_= *head; *head=this; }

        void remove(cDynProperty** head) 
        {
            cDynProperty *c,**p=head;
            for (c= *head, p=head;c != this && c != NULL;c=c->next_)
                    p= &(c->next_);
            assert(c != NULL);
            *p=c->next_;
            //delete c;
        }

        void call(void *arg) 
        {
            next_->call(arg);
            action(arg);
        }

        virtual void action(void* arg) = 0;


    //----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //----------------------------------------------------------------------
private:

    cDynProperty* next_;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------