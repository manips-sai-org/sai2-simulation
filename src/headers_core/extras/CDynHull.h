//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynHullH
#define CDynHullH
//---------------------------------------------------------------------------
class cDynObject;

//---------------------------------------------------------------------------
/*!
    cDynHull must be called after call to obj->geometry.end();
    to replace previous geometry with convex hull of that
    geometry.
*/
//---------------------------------------------------------------------------
void cDynHull(cDynObject* obj);

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------