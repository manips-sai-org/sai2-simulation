//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynCoverH
#define CDynCoverH
//---------------------------------------------------------------------------
#include "matrix/CDynMathDefn.h"
//---------------------------------------------------------------------------
class cDynGeometry;
class cDynPrim;
//---------------------------------------------------------------------------
void cDynCoverRadius(const double r);
void cDynCoverHull(cDynGeometry *geom, cDynPrim *prim);
void cDynCoverPoly(cDynGeometry *geom, cDynPrim *prim);
//---------------------------------------------------------------------------
#endif // CDynCoverH
//---------------------------------------------------------------------------
