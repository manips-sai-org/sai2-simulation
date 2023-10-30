//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================
//---------------------------------------------------------------------------
#ifndef CDynMatrix6H
#define CDynMatrix6H
//---------------------------------------------------------------------------
#include "matrix/CDynMatrix3.h"
#include "matrix/CDynTransform.h"
//---------------------------------------------------------------------------
class cDynVector6;
//---------------------------------------------------------------------------
class cDynMatrix6
{
    //-----------------------------------------------------------------------
    // OPERATORS:
    //-----------------------------------------------------------------------
public:

    //! Operator '='.
    void operator=(const cDynMatrix6 &m) 
    {
        mat3_[0] = m.mat3_[0];	mat3_[1] = m.mat3_[1];
        mat3_[2] = m.mat3_[2];	mat3_[3] = m.mat3_[3];
    }
    
    //! Operator '[]'.
    cDynMatrix3 *operator[](const int row) 
    { 
        return (mat3_+row*2); 
    }

    //! Operator '[]'.
    const cDynMatrix3 *operator[](const int row) const  
    { 
        return (mat3_+row*2); 
    }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    //! Get element (i,j).
    double &elementAt(const int i,const int j) 
    { 
        return mat3_[(i/3)*2+(j/3)].elementAt(i%3,j%3); 
    }
    
    //! Get element (i,j).
    const double &elementAt(const int i,const int j) const 
    { 
        return mat3_[(i/3)*2+(j/3)].elementAt(i%3,j%3); 
    }

    //! Initialize matrix to zero.
    void zero() 
    {
        mat3_[0].zero();	mat3_[1].zero();
        mat3_[2].zero();	mat3_[3].zero();
    }
    
    // Set identity matrix.
    void identity() 
    {
        mat3_[0].identity();	mat3_[1].zero();
        mat3_[2].zero();		mat3_[3].identity();
    }

    //! this = -m
    void negate(const cDynMatrix6 &m) 
    {
        mat3_[0].negate(m.mat3_[0]);	mat3_[1].negate(m.mat3_[1]);
        mat3_[2].negate(m.mat3_[2]);	mat3_[3].negate(m.mat3_[3]);
    }

    //! this = m1 + m2
    void add(const cDynMatrix6 &m1, const cDynMatrix6 &m2) 
    {
        mat3_[0].add(m1.mat3_[0], m2.mat3_[0]);	mat3_[1].add(m1.mat3_[1], m2.mat3_[1]);
        mat3_[2].add(m1.mat3_[2], m2.mat3_[2]);	mat3_[3].add(m1.mat3_[3], m2.mat3_[3]);
    }

    //! this = m1 - m2
    void subtract(const cDynMatrix6 &m1, const cDynMatrix6 &m2) 
    {
        mat3_[0].subtract(m1.mat3_[0], m2.mat3_[0]);	mat3_[1].subtract(m1.mat3_[1], m2.mat3_[1]);
        mat3_[2].subtract(m1.mat3_[2], m2.mat3_[2]);	mat3_[3].subtract(m1.mat3_[3], m2.mat3_[3]);
    }

    //! this[i] = m[i] * s
    void multiply(const cDynMatrix6 &m, const double s) 
    {
        mat3_[0].multiply(m.mat3_[0], s);	mat3_[1].multiply(m.mat3_[1], s);
        mat3_[2].multiply(m.mat3_[2], s);	mat3_[3].multiply(m.mat3_[3], s);
    }

    //! this += m
    void operator+=(const cDynMatrix6 &m) 
    {
        mat3_[0] += m.mat3_[0];	mat3_[1] += m.mat3_[1];
        mat3_[2] += m.mat3_[2];	mat3_[3] += m.mat3_[3];
    }

    //! this -= m
    void operator-=(const cDynMatrix6 &m) 
    {
        mat3_[0] -= m.mat3_[0];	mat3_[1] -= m.mat3_[1];
        mat3_[2] -= m.mat3_[2];	mat3_[3] -= m.mat3_[3];
    }

    //! this[i] -= s
    void operator*=(const double s) 
    {
        mat3_[0] *= s;	mat3_[1] *= s;
        mat3_[2] *= s;	mat3_[3] *= s;
    }
    //! this = m^T
    void transpose(const cDynMatrix6 &m) 
    {
        mat3_[0].transpose(m.mat3_[0]);	mat3_[1].transpose(m.mat3_[2]);
        mat3_[2].transpose(m.mat3_[1]);	mat3_[3].transpose(m.mat3_[3]);
    }

    //! this = m1 * m2
    void multiply(const cDynMatrix6 &m1, const cDynMatrix6 &m2);

    //! this = m1^T * m2
    void transposedMultiply(const cDynMatrix6 &m1, const cDynMatrix6 &m2);

    //! this = m1 * m2^T
    void multiplyTransposed(const cDynMatrix6 &m1, const cDynMatrix6 &m2);

    //! this = v1 * v2^T
    void multiplyTransposed(const cDynVector6 &v1, const cDynVector6 &v2);

    //! this = L * I * L^T : I is symmetric
    void similarityXform(const cDynMatrix6 &L,const cDynMatrix6 &I);

    //! this = L^T * I * L : I is symmetric
    void similarityXformT(const cDynMatrix6 &L,const cDynMatrix6 &I);

    //! this = m^-1
    void inverse(const cDynMatrix6 &m);

    //! this = m^-1 where m is SPD
    void inverseSPD(const cDynMatrix6 &m);

    //! this = LU decomposition of m
    void luDecomp(const cDynMatrix6 &m);

    //! this = LU decomposition of m where m is SPD
    void luDecompSPD(const cDynMatrix6 &m);

    //! this = X;
    //  X = [R 0; dxR R]
    void set(const cDynTransform &t) 
    {
        mat3_[0] = t.rotation();
        mat3_[1].zero();
        mat3_[2].crossMultiply(t.translation(), mat3_[0]);
        mat3_[3] = mat3_[0];
    }

    //! this = X * m
    void xform(const cDynTransform &t, const cDynMatrix6 &m);

    //! this = Li = R Li+1 Rt : L symmetric
    void similarityRform(const cDynTransform &t, const cDynMatrix6 &L);

    //! this =  Ii = X Ii+1 Xt : I symmetric
    void similarityXform(const cDynTransform &t, const cDynMatrix6 &I);

    //! this = Ii+1 = Xt Ii X : I symmetric
    void similarityXformT(const cDynTransform &t, const cDynMatrix6 &I);

    //! this =  Li+1 = Xinv Li Xinvt : L symmetric
    void similarityXformInv(const cDynTransform &t, const cDynMatrix6 &I);


#ifdef CDYN_DEBUG
    void display(char *str);
#endif // CDYN_DEBUG


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
private:
    
    cDynMatrix3 mat3_[4];
};

//---------------------------------------------------------------------------
#endif // CDynMatrix6H
//---------------------------------------------------------------------------
