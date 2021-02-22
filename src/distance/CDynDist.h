//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynDistH
#define CDynDistH
//---------------------------------------------------------------------------
#include "matrix/CDynMathDefn.h"
//---------------------------------------------------------------------------
class cDynVector3;
//---------------------------------------------------------------------------
#define CDYN_DIST_EPS ((double)1e-10)
//---------------------------------------------------------------------------
#define CDYN_DISTW_NOV(dw)		    ((dw)->_nov)
#define CDYN_DISTW_INDEX(dw)		((dw)->_index)
#define CDYN_DISTW_LAMBDA(dw)		((dw)->_lambda)
//---------------------------------------------------------------------------
struct cDynDistWitness 
{
    int _nov;
    int _index[2][4];
    double _lambda[4];
};
//---------------------------------------------------------------------------

// useMax indicates if the routine uses "max" variable to return quickly.
double cDynDistDistance(const cDynVector3* va,const int na,const cDynVector3* vb,const int nb,cDynDistWitness *dw,const double max,const bool useMax);

// optimized: no backup
double cDynDistDistance2(const cDynVector3* va,const int na,const cDynVector3* vb,const int nb,cDynDistWitness *dw,const double max,const bool useMax);

// index = 0 --> pointA, index = 1 --> pointB
void cDynDistPoint(const cDynVector3 *vec, const cDynDistWitness *dw, const int index, cDynVector3 *x);
void cDynDistNormal(const cDynVector3 *veca, const cDynVector3 *vecb, const cDynDistWitness *dw, cDynVector3 *n);

//---------------------------------------------------------------------------
#endif // CDynDistH
//---------------------------------------------------------------------------
