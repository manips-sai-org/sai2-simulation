//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynHierachyH
#define CDynHierachyH
//---------------------------------------------------------------------------
class cDynBSphere;
//---------------------------------------------------------------------------
cDynBSphere* cDynHierarchy(cDynBSphere *list);
void cDynHierarchyFullCheck(cDynBSphere* root);
int cDynHierarchyLeafCount(cDynBSphere *root);
int cDynHierarchyCount(cDynBSphere *root);
//---------------------------------------------------------------------------
#endif // CDynHierachyH
//---------------------------------------------------------------------------
