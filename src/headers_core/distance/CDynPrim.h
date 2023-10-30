//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynPrimH
#define CDynPrimH
//---------------------------------------------------------------------------
#include "matrix/CDynVector3.h"
//---------------------------------------------------------------------------
class cDynMaterial;
//---------------------------------------------------------------------------
class cDynPrim 
{
    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
public:

    //! Constructor of cDynPrim.    
    cDynPrim(int n) : num(n), m(NULL)
    {
        v = new cDynVector3[n];
        data_=NULL;
        id= cDynPrim::ID++;
    };

    //! Destructor of cDynPrim.
    ~cDynPrim(void) 
    {
        delete []v;
    };


    //----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //----------------------------------------------------------------------
public:

    void remove(void) 
    {
        if (next != NULL) 
            next->remove();
        delete this;
    }

    cDynMaterial* material() const { return(m); }

    char *data() const { return(data_); }
    void data(char* d) { data_=d; }

    void display() const; 
    void displayall(void) const;


    //----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //----------------------------------------------------------------------
public:

    unsigned int id;
    int num;
    cDynVector3 *v;

    double r;
    double err;
    cDynVector3 normal;
    double d;
    cDynMaterial* m;
    char* data_;

    cDynPrim *next;


    //----------------------------------------------------------------------
    // PUBLIC STATIC MEMBERS:
    //----------------------------------------------------------------------
public:

    static unsigned int ID;
};
//---------------------------------------------------------------------------
#endif //_cDynPrimH
//---------------------------------------------------------------------------