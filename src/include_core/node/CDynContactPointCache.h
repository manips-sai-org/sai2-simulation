//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynContactPointCacheH
#define CDynContactPointCacheH
//---------------------------------------------------------------------------
#ifndef CDYN_DONT_DOCUMENT
//---------------------------------------------------------------------------
#include "node/CDynContact.h"
//---------------------------------------------------------------------------
#define CDYN_POINTCACHESIZE	671
//---------------------------------------------------------------------------

class cDynContactPointCache 
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
public:

    //! Constructor of cDynContactPointCache.  
    cDynContactPointCache() 
    {
        callnum_=1;
        for (int i=0;i<CDYN_POINTCACHESIZE;i++) 
            num_[i]=0;
        miss_=hit_=collision_=0;
    };
    

    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    void reset() {callnum_++;};
    int check(const cDynContact* c, const cDynVector3& n, const cDynVector3& r, const double err, const double maxerr);
    void insert(cDynContactPoint* p, const int h);


    //-----------------------------------------------------------------------
    // PRIVATE METHODS:
    //-----------------------------------------------------------------------
private:

    unsigned int h(const cDynContact* c, const cDynVector3& n, const cDynVector3& r, const double err, const double maxerr);
    unsigned int callnum_;
    unsigned int num_[CDYN_POINTCACHESIZE];
    cDynContactPoint* point_[CDYN_POINTCACHESIZE];

    unsigned int miss_;
    unsigned int hit_;
    unsigned int collision_;
};

//---------------------------------------------------------------------------
extern cDynContactPointCache g_dynTheContactPointCache;
//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------