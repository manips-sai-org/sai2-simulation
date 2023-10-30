//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynCollisionCheckRecordH
#define CDynCollisionCheckRecordH
//---------------------------------------------------------------------------
#ifndef CDYN_DONT_DOCUMENT
//---------------------------------------------------------------------------
#include "matrix/CDynMathDefn.h"
#include "matrix/CDynFrame.h"
#include "node/CDynPrimPair.h"
//---------------------------------------------------------------------------
static const int CDYN_ALL_PENETRATION   = 1;
static const int CDYN_ALL_CONTACT       = 2;
//---------------------------------------------------------------------------
static const int CDYN_SAVE_PENETRATION  = 4;
static const int CDYN_SAVE_CONTACT      = 8;
//---------------------------------------------------------------------------

class cDynCollisionCheckRecord
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
public:

    //! Constructor of cDynCollisionCheckRecord.    
    cDynCollisionCheckRecord(int option) { bias_=np_=nc_=0; option_=option; plist_=NULL; clist_=NULL; }

    //! Destructor of cDynCollisionCheckRecord.
    inline ~cDynCollisionCheckRecord();


    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    inline void reset();
    inline void penetration(cDynPrimPair*& pair);
    inline void contact(cDynPrimPair*& pair);
    bool option(int flag) const { return((option_ & flag) != 0); }
    void optionSet(int flags) { option_ = flags; }
    int np() { return(np_); }
    int nc() { return(nc_); }
    cDynPrimPair* plist() { return(plist_); }
    cDynPrimPair* clist() { return(clist_); }
    void biasA() { bias_++; }
    void biasB() { bias_--; }
    int bias() { return(bias_); }

    cDynPrimPair* distance( cDynObject* nodeA, cDynObject* nodeB, const cDynTransform& Tab, const cDynBSphere* rootA=NULL, const cDynBSphere* rootB=NULL, double error=0.0f, double min=0.0f, double max=CDYN_MAXDEFLOAT);
    int checkPrim( cDynObject* nodeA, cDynObject* nodeB, const cDynTransform& Tab, const cDynPrim* a=NULL, const cDynPrim* b=NULL);
    int checkBS2( cDynObject* nodeA, cDynObject* nodeB, const cDynTransform& Tab, const cDynBSphere* rootA=NULL, const cDynBSphere* rootB=NULL, int threshold=5);
    int checkBS( cDynObject* nodeA, cDynObject* nodeB, const cDynTransform& Tab, const cDynBSphere* rootA=NULL, const cDynBSphere* rootB=NULL);
    void addContacts(const cDynTime& time);


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
private:

    int option_;
    int bias_;
    int np_;
    int nc_;
    cDynPrimPair* plist_;
    cDynPrimPair* clist_;
};

//---------------------------------------------------------------------------

inline void cDynCollisionCheckRecord::penetration(cDynPrimPair*& pair)
{
    if (option(CDYN_SAVE_PENETRATION)) 
    {
        pair->next_=plist_;
        plist_=pair;
        pair=NULL;
    }
    
    np_++;
}

//---------------------------------------------------------------------------

inline void cDynCollisionCheckRecord::contact(cDynPrimPair*& pair)
{
    if (option(CDYN_SAVE_CONTACT)) 
    {
        pair->next_=clist_;
        clist_=pair;
        pair=NULL;
    }
    
    nc_++;
}

//---------------------------------------------------------------------------

void inline cDynCollisionCheckRecord::reset()
{
    while (plist_ != NULL) 
    {
        cDynPrimPair* tmp=plist_;
        plist_=plist_->next_;
        delete tmp;
    }
    
    while (clist_ != NULL) 
    {
        cDynPrimPair* tmp=clist_;
        clist_=clist_->next_;
        delete tmp;
    }
    
    np_=0; nc_=0;
    bias_=0;
}

//---------------------------------------------------------------------------

inline cDynCollisionCheckRecord::~cDynCollisionCheckRecord()
{
    reset();
}

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------