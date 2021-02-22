//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynPrimPairH
#define CDynPrimPairH
//---------------------------------------------------------------------------
#ifndef CDYN_DONT_DOCUMENT
//---------------------------------------------------------------------------
#include "matrix/CDynMathDefn.h"
#include "matrix/CDynTransform.h"
#include "object/CDynMaterial.h"
#include "distance/CDynPrim.h"
#include "distance/CDynDist.h"
#include "object/CDynObject.h"
//---------------------------------------------------------------------------
class cDynContact;
class cDynContactList;
class cDynConstraint;
class cDynCollisionCheckRecord;
class cDynFrictionRecord;
//---------------------------------------------------------------------------
enum cDynPrimPairError {CDYN_NO_FAILURE, 
                        CDYN_UNKNOWN_FAILURE, 
                        CDYN_REGRESSION_FAILURE, 
                        CDYN_TOLERANCE_FAILURE};
//---------------------------------------------------------------------------
class cDynPrimPair 
{
    friend class cDynContact;
    friend class cDynContactList;
    friend class cDynContactEvent;
    friend class cDynConstraint;
    friend class cDynContactPointCache;
    friend class cDynCollisionCheckRecord;
    friend inline void cDynPrimPairCallnum();


    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
public:

    //! Constructor of cDynPrimPair. 
    cDynPrimPair() 
    {
        nodeA_=NULL;
        nodeB_=NULL;
        primA_=NULL;
        primB_=NULL;
        callnum_=0;
        CDYN_DISTW_NOV(&witness_)=0;
        dis_=0.0f;
    }

    //! Constructor of cDynPrimPair. 
    cDynPrimPair(cDynObject* nodeA,const cDynPrim* primA,cDynObject* nodeB, const cDynPrim* primB) 
    {
        nodeA_=nodeA; primA_=primA;
        nodeB_=nodeB; primB_=primB;
        callnum_=0;
        CDYN_DISTW_NOV(&witness_)=0;
        dis_=0.0f;
    }


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //-----------------------------------------------------------------------
public:

    double epsilon() const 
    {
        cDynMaterial* mA=primA_->material();
        cDynMaterial* mB=primB_->material();
        if (mA != NULL) return(mA->epsilon(mB));
        if (mB != NULL) return(mB->epsilon());
        return(0.0f);
    }

    double friction(const cDynFrictionType type) const 
    {
        cDynMaterial* mA=primA_->material();
        cDynMaterial* mB=primB_->material();
        if (mA != NULL) return(mA->friction(type,mB));
        if (mB != NULL) return(mB->friction(type));
        return(0.0f);
    }

    cDynObject* nodeA() { return(nodeA_); }
    cDynObject* nodeB() { return(nodeB_); }
    const cDynPrim* primA() { return(primA_); }
    const cDynPrim* primB() { return(primB_); }

    int checkPrim(const cDynTransform& Tab);
    cDynTime checkPrimTime(const cDynTime tl, const cDynTime tu, cDynTransform& Tab, cDynPrimPairError& status);
    int findPatch(const cDynVector3& x, const cDynVector3& n,
        const double min, const double max, const double maxerr,
        const double rA, const double rB,
        const cDynFrame& gA, const cDynFrame& gB,
        cDynContact* contact, cDynFrictionRecord* fr);
    void findContacts(const cDynTime& time, cDynContact* contact);
    double distance() const { return(dis_+0.5f*(primA_->err + primB_->err)); }
    cDynPrimPair* next() const { return(next_); }


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
private:

    long callnum_;

    cDynObject *nodeA_, *nodeB_;
    const cDynPrim *primA_, *primB_;
    cDynDistWitness witness_;
    double dis_;

    cDynPrimPair* next_;
    cDynPrimPair* pairCache() const;


    //-----------------------------------------------------------------------
    // PRIVATE STATIC MEMBERS:
    //-----------------------------------------------------------------------
private:

    static long CALLNUM; // value stored in cDynPrim.cpp
    static cDynPrimPair* free_;


    //-----------------------------------------------------------------------
    // OPERATORS:
    //-----------------------------------------------------------------------
private:

    void* operator new(size_t s) 
    {
        if (free_==NULL) 
        {
            cDynPrimPair *tmp = (cDynPrimPair *)malloc(s);
//			if (cDynDebugLevel == -100)
//				cDynPrintf("cDynPrimPair\n");
            return(tmp);
        } 
        else 
        {
            cDynPrimPair* tmp=free_;
            free_=free_->next_;
            return(tmp);
        }
    }

    void operator delete(void *ptr) 
    {
        cDynPrimPair* tmp=(cDynPrimPair *)ptr;
        tmp->next_=free_;
        free_=tmp;
    }
};

//---------------------------------------------------------------------------

inline void cDynPrimPairCallnum() { cDynPrimPair::CALLNUM++; }

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------