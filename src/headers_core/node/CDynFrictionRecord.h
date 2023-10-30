//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynFrictionRecordH
#define CDynFrictionRecordH
//---------------------------------------------------------------------------
#ifndef CDYN_DONT_DOCUMENT
//---------------------------------------------------------------------------
#include "matrix/CDynVector3.h"
#include "object/CDynMaterial.h"
#include "node/CDynPrimPair.h"
//---------------------------------------------------------------------------

class cDynFrictionRecord 
{
    friend class cDynPrimPair;

    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
public:

    //! Constructor of cDynFrictionRecord.    
    cDynFrictionRecord* reference() { rc_++; return(this); }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    void unreference() { rc_--; if (rc_ == 0) delete(this); }
    bool friction() const { return(friction_); }
    double ugrip() const { return(ug_); }
    double uviscous() const { return(uv_); }
    double udynamic() const { return(ud_); }
    double ustatic() const { return(us_); }
    double vlimit() const { return(vl_); }
    const cDynVector3& x() const { return(xt_); }
    const cDynVector3& y() const { return(yt_); }


    //-----------------------------------------------------------------------
    // PRIVATE METHODS:
    //-----------------------------------------------------------------------
private:

    //! Reference count.
    int rc_;        

    //! Does friction exist between pairs.
    bool friction_;

    //! Grip friction coefficient.
    double ug_;    

    //! Viscous friction coefficient.
    double uv_;    

    //! Dynamic friction coefficient.
    double ud_;    

    //! Static friction coefficient.
    double us_; 

    //! Velocity limit.
    double vl_;	
    
    //! x-tangent direction.
    cDynVector3 xt_;  

    //! y-tangent direction.
    cDynVector3 yt_;  

    //! Used for memory management only
    cDynFrictionRecord* next_; 


    //-----------------------------------------------------------------------
    // PRIVATE STATIC MEMBERS:
    //-----------------------------------------------------------------------
private:

    static cDynFrictionRecord* free_;

    cDynFrictionRecord(const cDynVector3& n, cDynPrimPair* prim);


    //-----------------------------------------------------------------------
    // OPERATORS:
    //-----------------------------------------------------------------------
private:

    //! Operator new.
    void* operator new(size_t s)
    {
        if (free_==NULL) {
            cDynFrictionRecord *tmp = (cDynFrictionRecord *)malloc(s);
//			if (cDynDebugLevel == -100)
//				cDynPrintf("cDynFrictionRecord\n");
            return(tmp);
        } 
        else 
        {
            cDynFrictionRecord* tmp=free_;
            free_=free_->next_;
            return(tmp);
        }
    }

    //! Operator delete.
    void operator delete(void *ptr) 
    {
        cDynFrictionRecord* tmp=(cDynFrictionRecord *)ptr;
        tmp->next_=free_;
        free_=tmp;
    }
};

//---------------------------------------------------------------------------
#endif // CDYN_DONT_DOCUMENT
//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------